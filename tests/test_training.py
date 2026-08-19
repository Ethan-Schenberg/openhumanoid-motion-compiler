from __future__ import annotations

import ast
import json
from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from ohmc.cli import build_parser
from ohmc.errors import OhmcError
from ohmc.profiles import load_yaml_object, validate_robot_profile
from ohmc.training import (
    TrainingStore,
    evaluate_run,
    execute_run,
    load_json_object,
    load_training_recipe,
    prepare_policy_bundle,
    prepare_run,
    schema_issues,
    verify_policy_bundle,
)

ROOT = Path(__file__).resolve().parents[1]
RECIPE_PATH = ROOT / "examples" / "training" / "x2_rgbd_rough_ppo_v1.yaml"
RECIPE_SCHEMA = ROOT / "schemas" / "training-recipe-v0.1.schema.json"
RUN_SCHEMA = ROOT / "schemas" / "run-manifest-v0.1.schema.json"
METRICS_SCHEMA = ROOT / "schemas" / "evaluation-metrics-v0.1.schema.json"
EVIDENCE_SCHEMA = ROOT / "schemas" / "evidence-bundle-v0.1.schema.json"
POLICY_SCHEMA = ROOT / "schemas" / "policy-bundle-v0.1.schema.json"
PROFILE_SCHEMA = ROOT / "schemas" / "robot-profile-v0.1.schema.json"
PROFILE_PATH = ROOT / "profiles" / "agibot_x2_ultra_locomotion_29dof_v1.yaml"
PASSING_METRICS = ROOT / "examples" / "training" / "passing_metrics.json"


def fixture_recipe(tmp_path: Path) -> tuple[Path, dict]:
    recipe = load_yaml_object(RECIPE_PATH)
    recipe["backend"] = {
        "id": "fixture",
        "runtime": "fixture",
        "command": [],
        "versions": {"fixture": "1"},
    }
    path = tmp_path / "fixture-recipe.yaml"
    path.write_text(yaml.safe_dump(recipe, sort_keys=False), encoding="utf-8")
    return path, recipe


def make_store(tmp_path: Path) -> TrainingStore:
    return TrainingStore(tmp_path / "runs", run_schema_path=RUN_SCHEMA)


def test_x2_locomotion_contract_has_29_joints_and_excludes_head() -> None:
    profile = load_yaml_object(PROFILE_PATH)
    assert validate_robot_profile(profile, load_json_object(PROFILE_SCHEMA)) == []
    assert len(profile["control"]["joint_order"]) == 29
    assert not any("head" in name for name in profile["control"]["joint_order"])
    assert {item["name"] for item in profile["control"]["excluded_joints"]} == {
        "head_yaw_joint",
        "head_pitch_joint",
    }
    assert profile["control"]["hardware_transport"] == "disabled"


def test_training_commands_accept_profile_alias_without_schema_ambiguity() -> None:
    parser = build_parser()
    doctor = parser.parse_args(["doctor", "--profile", str(PROFILE_PATH)])
    train = parser.parse_args(
        ["train", str(RECIPE_PATH), "--profile", str(PROFILE_PATH)]
    )

    assert doctor.robot == PROFILE_PATH
    assert doctor.profile_schema == PROFILE_SCHEMA
    assert train.robot == PROFILE_PATH
    assert train.profile_schema == PROFILE_SCHEMA


def test_training_recipe_locks_random_rgbd_asymmetric_contract() -> None:
    recipe, issues = load_training_recipe(RECIPE_PATH, RECIPE_SCHEMA)
    assert issues == []
    assert recipe["initialization"] == "random"
    assert recipe["algorithm"]["actor_critic"] == "asymmetric"
    assert recipe["backend"]["versions"]["isaac_lab_revision"] == (
        "ffff603eafc6b74264a5261cc0183d6a65390d78"
    )
    assert recipe["control"]["action_dimensions"] == 29
    assert recipe["perception"]["rgb_channels"] == 3
    assert recipe["perception"]["depth_channels"] == 1
    assert recipe["authority"]["hardware_transport"] == "disabled"

    invalid = deepcopy(recipe)
    invalid["initialization"] = "pretrained"
    assert schema_issues(invalid, load_json_object(RECIPE_SCHEMA))


def test_state_machine_rejects_skipping_from_preflight_to_hardware(
    tmp_path: Path,
) -> None:
    recipe_path, recipe = fixture_recipe(tmp_path)
    store = make_store(tmp_path)
    manifest = store.create(recipe, recipe_path)
    store.transition(manifest["run_id"], "preflight", "test")
    with pytest.raises(OhmcError, match="invalid run transition"):
        store.transition(manifest["run_id"], "hardware_candidate", "unsafe skip")


def test_fixture_run_evaluation_and_policy_bundle_are_auditable(tmp_path: Path) -> None:
    recipe_path, recipe = fixture_recipe(tmp_path)
    store = make_store(tmp_path)
    manifest, report = prepare_run(
        store,
        recipe,
        recipe_path,
        profile_path=PROFILE_PATH,
        profile_schema_path=PROFILE_SCHEMA,
    )
    run_id = manifest["run_id"]
    assert report["ready"] is True
    assert store.get(run_id)["execution"]["status"] == "ready"

    assert execute_run(store, run_id) == 0
    assert store.get(run_id)["state"] == "evaluating"
    evidence = evaluate_run(
        store,
        run_id,
        PASSING_METRICS,
        metrics_schema_path=METRICS_SCHEMA,
        evidence_schema_path=EVIDENCE_SCHEMA,
    )
    assert evidence["status"] == "pass"
    assert evidence["authority"] == {
        "label": "simulation_passed",
        "hardware_transport": False,
        "operator_review_required": True,
    }
    assert store.get(run_id)["state"] == "awaiting_hardware_review"

    run_dir = store.run_dir(run_id)
    output = tmp_path / "policy-bundle"
    bundle = prepare_policy_bundle(
        store,
        run_id,
        policy_path=run_dir / "policy.fixture.onnx",
        output_dir=output,
        profile_path=PROFILE_PATH,
        profile_schema_path=PROFILE_SCHEMA,
        policy_schema_path=POLICY_SCHEMA,
    )
    assert len(bundle["joint_order"]) == 29
    assert bundle["authority"]["hardware_execution"] is False
    assert (
        verify_policy_bundle(output, policy_schema_path=POLICY_SCHEMA)["status"]
        == "pass"
    )

    (output / "policy.onnx").write_bytes(b"tampered")
    verification = verify_policy_bundle(output, policy_schema_path=POLICY_SCHEMA)
    assert verification["status"] == "fail"
    assert verification["issues"] == ["artifact hash mismatch: policy.onnx"]


def test_failed_gate_never_reaches_sim2sim_or_hardware_review(tmp_path: Path) -> None:
    recipe_path, recipe = fixture_recipe(tmp_path)
    store = make_store(tmp_path)
    manifest, report = prepare_run(
        store,
        recipe,
        recipe_path,
        profile_path=PROFILE_PATH,
        profile_schema_path=PROFILE_SCHEMA,
    )
    assert report["ready"]
    run_id = manifest["run_id"]
    execute_run(store, run_id)
    metrics = json.loads(PASSING_METRICS.read_text())
    metrics["metrics"]["flat"]["no_fall_rate"] = 0.4
    metrics_path = tmp_path / "failing-metrics.json"
    metrics_path.write_text(json.dumps(metrics), encoding="utf-8")
    evidence = evaluate_run(
        store,
        run_id,
        metrics_path,
        metrics_schema_path=METRICS_SCHEMA,
        evidence_schema_path=EVIDENCE_SCHEMA,
    )
    assert evidence["status"] == "fail"
    assert evidence["authority"]["label"] == "simulation_failed"
    assert store.get(run_id)["state"] == "failed"


def test_manifest_and_events_survive_store_reopen(tmp_path: Path) -> None:
    recipe_path, recipe = fixture_recipe(tmp_path)
    store = make_store(tmp_path)
    manifest = store.create(recipe, recipe_path)
    run_id = manifest["run_id"]
    store.transition(run_id, "preflight", "persist me")

    reopened = make_store(tmp_path)
    assert reopened.get(run_id)["state"] == "preflight"
    assert [event["kind"] for event in reopened.events(run_id)] == ["state", "state"]


def test_interrupted_run_resumes_its_own_fixture_checkpoint(tmp_path: Path) -> None:
    recipe_path, recipe = fixture_recipe(tmp_path)
    store = make_store(tmp_path)
    manifest = store.create(recipe, recipe_path)
    run_id = manifest["run_id"]
    store.transition(run_id, "preflight", "ready")
    store.set_execution(run_id, "ready")
    store.transition(run_id, "training", "started before host interruption")
    store.set_execution(
        run_id,
        "blocked",
        error="interrupted",
        message="host interruption fixture",
    )

    assert execute_run(store, run_id, resume=True) == 0
    assert store.get(run_id)["state"] == "evaluating"
    assert (store.run_dir(run_id) / "checkpoint.fixture").is_file()


def test_isaaclab_extension_matches_recipe_and_29_joint_profile() -> None:
    integration = ROOT / "integrations" / "isaaclab"
    lock = load_yaml_object(integration / "integration-lock.yaml")
    recipe = load_yaml_object(RECIPE_PATH)
    assert (
        lock["isaac_lab"]["revision"]
        == recipe["backend"]["versions"]["isaac_lab_revision"]
    )
    assert lock["rsl_rl"]["version"] == recipe["backend"]["versions"]["rsl_rl"]
    assert lock["authority"]["hardware_transport"] == "disabled"

    assets_source = (integration / "source" / "ohmc_x2" / "assets.py").read_text()
    module = ast.parse(assets_source)
    assignment = next(
        node
        for node in module.body
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "LOCOMOTION_JOINTS"
            for target in node.targets
        )
    )
    integration_joints = ast.literal_eval(assignment.value)
    profile = load_yaml_object(PROFILE_PATH)
    assert integration_joints == profile["control"]["joint_order"]

    runner = (
        integration / "source" / "ohmc_x2" / "tasks" / "agents" / "rsl_rl_ppo_cfg.py"
    ).read_text()
    assert '"actor": ["policy", "rgb", "depth"]' in runner
    assert '"critic": ["policy", "privileged"]' in runner
    assert "output_channels=[8, 16, 32]" in runner


def test_isaaclab_curriculum_is_ordered_and_random_initialization_only() -> None:
    source = (
        ROOT / "integrations" / "isaaclab" / "source" / "ohmc_x2" / "train.py"
    ).read_text()
    assert source.index('("00_stand",') < source.index('("01_flat",')
    assert source.index('("01_flat",') < source.index('("02_slope",')
    assert source.index('("02_slope",') < source.index('("03_uneven",')
    assert source.index('("03_uneven",') < source.index('("04_low_obstacle",')
    assert source.index('("04_low_obstacle",') < source.index('("05_stairs",')
    assert 'recipe.get("initialization") != "random"' in source
    assert "--use_pretrained_checkpoint" not in source
