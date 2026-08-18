import json
import math
from pathlib import Path

import pytest

from ohmc.bvh import load_bvh
from ohmc.canonical import BVH_Y_UP_CONVENTION, bvh_to_canonical_motion
from ohmc.cli import main
from ohmc.errors import OhmcError
from ohmc.ik import (
    build_ik_problem,
    ik_result_to_motion_ir,
    solve_ik_problem,
    validate_ik_problem,
    validate_ik_result,
    validate_ik_task_map,
)
from ohmc.ir import validate_motion_ir
from ohmc.normalization import normalize_canonical_motion
from ohmc.profiles import load_yaml_object
from ohmc.quality import derive_motion_kinematics
from ohmc.replay import replay_mujoco


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "examples" / "ik_contract_fixture.xml"
CANONICAL_SCHEMA = json.loads(
    (ROOT / "schemas" / "canonical-motion-v0.1.schema.json").read_text()
)
TASK_MAP_SCHEMA = json.loads(
    (ROOT / "schemas" / "ik-task-map-v0.1.schema.json").read_text()
)
PROBLEM_SCHEMA = json.loads(
    (ROOT / "schemas" / "ik-problem-v0.1.schema.json").read_text()
)
RESULT_SCHEMA = json.loads(
    (ROOT / "schemas" / "ik-result-v0.1.schema.json").read_text()
)
MOTION_SCHEMA = json.loads(
    (ROOT / "schemas" / "motion-ir-v0.1.schema.json").read_text()
)


def inputs() -> tuple[dict, dict, dict]:
    source = ROOT / "examples" / "simple_motion.bvh"
    canonical = bvh_to_canonical_motion(
        load_bvh(source),
        source_bytes=source.read_bytes(),
        source_name=source.name,
        source_license="CC0-1.0",
        source_convention=BVH_Y_UP_CONVENTION,
        source_length_unit="m",
    )
    canonical = normalize_canonical_motion(
        canonical, morphology_scale=1.0, rate_hz=50.0
    )
    profile = load_yaml_object(ROOT / "profiles" / "unitree_g1_29dof.yaml")
    task_map = load_yaml_object(ROOT / "examples" / "ik_task_map_fixture.yaml")
    return canonical, profile, task_map


def test_reference_ik_solves_analytic_roll_fixture() -> None:
    canonical, profile, task_map = inputs()
    assert validate_ik_task_map(task_map, TASK_MAP_SCHEMA) == []

    problem = build_ik_problem(canonical, profile, task_map, MODEL)
    assert validate_ik_problem(problem, PROBLEM_SCHEMA) == []
    result = solve_ik_problem(problem, MODEL)

    assert validate_ik_result(result, RESULT_SCHEMA) == []
    assert result["status"] == "pass"
    assert result["summary"]["failed_frame_count"] == 0
    assert result["summary"]["peak_residual_m"] <= 1e-6
    assert [frame["positions"][0] for frame in result["frames"]] == pytest.approx(
        [0.0, -math.radians(2.0), -math.radians(4.0)], abs=3e-6
    )
    assert all(frame["active_joint_limits"] == [] for frame in result["frames"])


def test_failed_ik_is_explicit_and_cannot_be_compiled() -> None:
    canonical, profile, task_map = inputs()
    problem = build_ik_problem(canonical, profile, task_map, MODEL)
    for frame in problem["frames"]:
        frame["targets"][0]["position_m"][0] += 1.0

    result = solve_ik_problem(problem, MODEL)

    assert validate_ik_result(result, RESULT_SCHEMA) == []
    assert result["status"] == "fail"
    assert result["summary"]["failed_frame_count"] == 3
    with pytest.raises(OhmcError, match="failed IK result"):
        ik_result_to_motion_ir(problem, result, canonical, profile)


def test_unreachable_task_reports_active_joint_limit() -> None:
    canonical, profile, task_map = inputs()
    problem = build_ik_problem(canonical, profile, task_map, MODEL)
    for frame in problem["frames"]:
        frame["targets"][0]["position_m"] = [0.0, 1.0, 0.5]

    result = solve_ik_problem(problem, MODEL)

    assert result["status"] == "fail"
    assert all(
        frame["active_joint_limits"] == ["waist_roll_joint"]
        for frame in result["frames"]
    )
    assert all(frame["positions"][0] == pytest.approx(0.52) for frame in result["frames"])


def test_solver_rejects_non_finite_linearization(monkeypatch: pytest.MonkeyPatch) -> None:
    canonical, profile, task_map = inputs()
    problem = build_ik_problem(canonical, profile, task_map, MODEL)
    problem["frames"][1]["targets"][0]["position_m"][0] += 0.1

    import mujoco

    original = mujoco.mj_jacBody

    def non_finite_jacobian(model, data, jacp, jacr, body_id):
        original(model, data, jacp, jacr, body_id)
        jacp[0, 0] = math.nan

    monkeypatch.setattr(mujoco, "mj_jacBody", non_finite_jacobian)

    with pytest.raises(OhmcError, match="non-finite linearization data"):
        solve_ik_problem(problem, MODEL)


def test_solved_ik_compiles_to_replayable_motion_ir() -> None:
    canonical, profile, task_map = inputs()
    problem = build_ik_problem(canonical, profile, task_map, MODEL)
    result = solve_ik_problem(problem, MODEL)

    motion = ik_result_to_motion_ir(problem, result, canonical, profile)
    motion = derive_motion_kinematics(motion)

    assert validate_motion_ir(motion, MOTION_SCHEMA) == []
    assert motion["trajectory"]["joints"] == ["waist_roll_joint"]
    assert all(
        sample["solver_status"] == "solved"
        for sample in motion["trajectory"]["samples"]
    )
    report = replay_mujoco(motion, MODEL)
    assert report["status"] == "pass"
    assert report["joints_mapped"] == 1


def test_retarget_ik_cli_builds_atomic_bundle_and_refuses_overwrite(
    tmp_path: Path,
) -> None:
    canonical, _, _ = inputs()
    canonical_path = tmp_path / "canonical.json"
    canonical_path.write_text(json.dumps(canonical))
    output = tmp_path / "ik-bundle"
    args = [
        "retarget-ik",
        str(canonical_path),
        "--robot",
        str(ROOT / "profiles" / "unitree_g1_29dof.yaml"),
        "--task-map",
        str(ROOT / "examples" / "ik_task_map_fixture.yaml"),
        "--model",
        str(MODEL),
        "--output",
        str(output),
    ]

    assert main(args) == 0
    assert json.loads((output / "ik-result.json").read_text())["status"] == "pass"
    assert (output / "motion.json").is_file()
    assert main(args) == 2


@pytest.mark.parametrize(
    ("name", "variable_count"),
    [
        ("full_body_unitree_g1_v2.yaml", 29),
        ("full_body_agibot_x2_v2.yaml", 30),
    ],
)
def test_multilimb_vendor_task_maps_are_schema_valid_and_cover_full_landmarks(
    name: str, variable_count: int
) -> None:
    task_map = load_yaml_object(ROOT / "profiles" / "ik" / name)

    assert validate_ik_task_map(task_map, TASK_MAP_SCHEMA) == []
    assert len(task_map["variables"]) == variable_count
    assert len(task_map["tasks"]) == 16
    assert {task["source_joint"] for task in task_map["tasks"]} == {
        "Hips",
        "Spine",
        "Chest",
        "Head",
        "LeftShoulder",
        "RightShoulder",
        "LeftHip",
        "LeftKnee",
        "LeftAnkle",
        "RightHip",
        "RightKnee",
        "RightAnkle",
        "LeftElbow",
        "LeftWrist",
        "RightElbow",
        "RightWrist",
    }


@pytest.mark.parametrize(
    "name",
    ["full_body_unitree_g1_v1.yaml", "full_body_agibot_x2_v1.yaml"],
)
def test_legacy_nine_task_maps_remain_versioned_and_valid(name: str) -> None:
    task_map = load_yaml_object(ROOT / "profiles" / "ik" / name)

    assert validate_ik_task_map(task_map, TASK_MAP_SCHEMA) == []
    assert task_map["id"].endswith("_v1")
    assert len(task_map["tasks"]) == 9
