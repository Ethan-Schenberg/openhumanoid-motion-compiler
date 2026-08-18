from copy import deepcopy
import json
from pathlib import Path

import pytest

from ohmc.bvh import bvh_to_motion_ir, load_bvh
from ohmc.cli import main
from ohmc.profiles import load_yaml_object, map_motion_ir
from ohmc.quality import (
    derive_motion_kinematics,
    trajectory_quality_report,
    validate_quality_report,
)


ROOT = Path(__file__).resolve().parents[1]
QUALITY_SCHEMA = json.loads(
    (ROOT / "schemas" / "trajectory-quality-v0.1.schema.json").read_text()
)


def quadratic_motion() -> dict:
    document = json.loads((ROOT / "examples" / "minimal_motion.json").read_text())
    document["trajectory"]["joints"] = ["quadratic", "linear"]
    document["trajectory"]["samples"] = [
        {"time": 0.0, "position_targets": [0.0, 1.0]},
        {"time": 1.0, "position_targets": [1.0, 3.0]},
        {"time": 3.0, "position_targets": [9.0, 7.0]},
    ]
    return document


def complete_profile() -> dict:
    return {
        "id": "quality_fixture",
        "control": {"joint_order": ["quadratic", "linear"]},
        "joint_limits": {
            "quadratic": {
                "lower": -1.0,
                "upper": 10.0,
                "velocity": 7.0,
                "acceleration": 3.0,
            },
            "linear": {
                "lower": 0.0,
                "upper": 8.0,
                "velocity": 3.0,
                "acceleration": 1.0,
            },
        },
    }


def mapped_vendor_motion(profile_name: str) -> tuple[dict, dict]:
    source = ROOT / "examples" / "simple_motion.bvh"
    motion = derive_motion_kinematics(
        bvh_to_motion_ir(
            load_bvh(source), source.read_bytes(), source.name, "CC0-1.0"
        )
    )
    profile = load_yaml_object(ROOT / "profiles" / profile_name)
    mapping = load_yaml_object(
        ROOT / "profiles" / "mappings" / "simple_bvh_semantics_v1.yaml"
    )
    return map_motion_ir(motion, profile, mapping), profile


def test_nonuniform_lagrange_derivatives_are_exact_for_quadratics() -> None:
    document = derive_motion_kinematics(quadratic_motion())
    samples = document["trajectory"]["samples"]

    assert [sample["velocity_targets"][0] for sample in samples] == pytest.approx(
        [0.0, 2.0, 6.0]
    )
    assert [sample["acceleration_targets"][0] for sample in samples] == pytest.approx(
        [2.0, 2.0, 2.0]
    )
    assert [sample["velocity_targets"][1] for sample in samples] == pytest.approx(
        [2.0, 2.0, 2.0]
    )
    assert [sample["acceleration_targets"][1] for sample in samples] == pytest.approx(
        [0.0, 0.0, 0.0], abs=1e-12
    )
    assert document["passes"][-1]["name"] == "derive_trajectory_kinematics"


def test_complete_trajectory_quality_passes_configured_limits() -> None:
    report = trajectory_quality_report(
        derive_motion_kinematics(quadratic_motion()), complete_profile()
    )

    assert validate_quality_report(report, QUALITY_SCHEMA) == []
    assert report["status"] == "pass"
    assert report["mapping"]["complete"] is True
    assert report["mapping"]["coverage_ratio"] == 1.0
    assert report["violations"] == []
    assert report["joint_metrics"][0]["maximum_absolute_velocity"] == pytest.approx(6.0)
    assert report["joint_metrics"][0]["maximum_absolute_acceleration"] == pytest.approx(
        2.0
    )


def test_quality_report_exposes_incomplete_vendor_mapping_and_missing_limits() -> None:
    motion, profile = mapped_vendor_motion("unitree_g1_29dof.yaml")

    report = trajectory_quality_report(motion, profile)

    assert validate_quality_report(report, QUALITY_SCHEMA) == []
    assert report["status"] == "warning"
    assert report["mapping"]["mapped_joint_count"] == 2
    assert report["mapping"]["controllable_joint_count"] == 29
    assert len(report["mapping"]["missing_joints"]) == 27
    assert report["dynamic_limit_coverage"] == {
        "velocity_configured_joint_count": 0,
        "acceleration_configured_joint_count": 0,
        "mapped_joint_count": 2,
    }


def test_quality_report_fails_velocity_violation() -> None:
    profile = complete_profile()
    profile = deepcopy(profile)
    profile["joint_limits"]["quadratic"]["velocity"] = 1.0

    report = trajectory_quality_report(
        derive_motion_kinematics(quadratic_motion()), profile
    )

    assert report["status"] == "fail"
    assert report["joint_metrics"][0]["velocity_status"] == "fail"
    assert any(item["kind"] == "velocity" for item in report["violations"])


def test_derive_and_quality_cli_support_strict_mapping_gate(tmp_path: Path) -> None:
    source = ROOT / "examples" / "simple_motion.bvh"
    imported = tmp_path / "imported.json"
    derived = tmp_path / "derived.json"
    mapped = tmp_path / "mapped.json"
    report = tmp_path / "quality.json"
    profile = ROOT / "profiles" / "unitree_g1_29dof.yaml"
    mapping = ROOT / "profiles" / "mappings" / "simple_bvh_semantics_v1.yaml"

    assert main(
        [
            "import-bvh",
            str(source),
            "--source-license",
            "CC0-1.0",
            "--output",
            str(imported),
        ]
    ) == 0
    assert main(
        ["derive-kinematics", str(imported), "--output", str(derived)]
    ) == 0
    assert main(
        [
            "map-joints",
            str(derived),
            "--robot",
            str(profile),
            "--mapping",
            str(mapping),
            "--output",
            str(mapped),
        ]
    ) == 0
    args = [
        "quality-report",
        str(mapped),
        "--robot",
        str(profile),
        "--output",
        str(report),
    ]
    assert main(args) == 0
    assert json.loads(report.read_text())["mapping"]["complete"] is False

    strict_report = tmp_path / "strict-quality.json"
    assert main(
        [
            *args[:-1],
            str(strict_report),
            "--require-complete-mapping",
        ]
    ) == 1
