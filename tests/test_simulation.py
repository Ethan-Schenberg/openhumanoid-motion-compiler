import hashlib
import json
from pathlib import Path

from ohmc.cli import main


ROOT = Path(__file__).resolve().parents[1]


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_fixture_simulation(output: Path, cache_dir: Path) -> int:
    return main(
        [
            "simulate",
            str(ROOT / "examples" / "simple_motion.bvh"),
            "--target",
            "unitree-g1-contract-fixture",
            "--source-license",
            "CC0-1.0",
            "--source-convention",
            "right_handed_x_right_y_up_z_backward",
            "--source-length-unit",
            "m",
            "--output",
            str(output),
            "--cache-dir",
            str(cache_dir),
        ]
    )


def test_one_command_simulation_builds_auditable_bundle(tmp_path: Path) -> None:
    output = tmp_path / "simulation"

    assert run_fixture_simulation(output, tmp_path / "cache") == 0

    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["schema"] == "ohmc.simulation_bundle/v0.1"
    assert manifest["target"] == "unitree-g1-contract-fixture"
    assert manifest["fidelity"] == "synthetic_contract_fixture"
    assert manifest["result"] == {
        "hardware_commands_sent": False,
        "motion_quality": "warning",
        "motion_validation": "warning",
        "replay": "pass",
    }
    assert manifest["capabilities"]["headless_kinematic_replay"] is True
    assert manifest["capabilities"]["canonical_source_kinematics"] is True
    assert manifest["capabilities"]["morphology_scaling"] is True
    assert manifest["capabilities"]["canonical_timeline_resampling"] is True
    assert manifest["capabilities"]["trajectory_derivatives"] is True
    assert manifest["capabilities"]["mapping_completeness_report"] is True
    assert manifest["capabilities"]["constrained_whole_body_ik"] is False
    assert manifest["capabilities"]["dynamic_controller_simulation"] is False
    assert manifest["capabilities"]["hardware_transport"] is False

    fixture = json.loads((output / "interface-fixture.json").read_text())
    assert fixture["adapter"] == "unitree-g1-lowcmd"
    assert fixture["executable"] is False
    report = json.loads((output / "replay-report.json").read_text())
    assert report["status"] == "pass"
    assert report["hardware_commands_sent"] is False
    canonical = json.loads((output / "canonical-motion.json").read_text())
    assert canonical["validation"]["status"] == "pass"
    assert len(canonical["samples"]) == 3
    assert canonical["passes"][-1]["name"] == "canonical_morphology_timeline_normalization"
    assert (output / "canonical.source.json").is_file()
    quality = json.loads((output / "quality-report.json").read_text())
    assert quality["status"] == "warning"
    assert quality["mapping"]["mapped_joint_count"] == 2
    assert quality["mapping"]["controllable_joint_count"] == 29

    for artifact in manifest["artifacts"].values():
        path = output / artifact["path"]
        assert path.is_file()
        assert file_sha256(path) == artifact["sha256"]


def test_simulation_is_reproducible_and_refuses_overwrite(tmp_path: Path) -> None:
    first = tmp_path / "first"
    second = tmp_path / "second"

    assert run_fixture_simulation(first, tmp_path / "cache") == 0
    assert run_fixture_simulation(second, tmp_path / "cache") == 0
    assert (first / "manifest.json").read_bytes() == (second / "manifest.json").read_bytes()
    assert run_fixture_simulation(first, tmp_path / "cache") == 2
