import json
from pathlib import Path

import pytest

from ohmc.bvh import load_bvh
from ohmc.canonical import CANONICAL_CONVENTION, bvh_to_canonical_motion
from ohmc.cli import main
from ohmc.landmarks import landmark_coverage_report, validate_landmark_coverage
from ohmc.profiles import load_yaml_object


ROOT = Path(__file__).resolve().parents[1]
SCHEMA = json.loads(
    (ROOT / "schemas" / "landmark-coverage-v0.1.schema.json").read_text()
)


def canonical(path: Path) -> dict:
    return bvh_to_canonical_motion(
        load_bvh(path),
        source_bytes=path.read_bytes(),
        source_name=path.name,
        source_license="CC0-1.0",
        source_convention=CANONICAL_CONVENTION,
        source_length_unit="m",
    )


def test_full_body_fixture_covers_all_required_landmarks() -> None:
    document = canonical(ROOT / "examples" / "full_body_motion.bvh")

    report = landmark_coverage_report(document)

    assert validate_landmark_coverage(report, SCHEMA) == []
    assert report["status"] == "pass"
    assert report["source"]["complete"] is True
    assert report["source"]["present_count"] == 16
    assert report["source"]["coverage_ratio"] == 1.0
    assert [joint["name"] for joint in document["skeleton"]["joints"]] == [
        "Hips",
        "Spine",
        "Chest",
        "Head",
        "LeftShoulder",
        "LeftElbow",
        "LeftWrist",
        "RightShoulder",
        "RightElbow",
        "RightWrist",
        "LeftHip",
        "LeftKnee",
        "LeftAnkle",
        "RightHip",
        "RightKnee",
        "RightAnkle",
    ]
    first = document["samples"][0]["world_positions_m"]
    assert first[12] == pytest.approx([0.0, 0.09, 0.1], abs=1e-12)
    assert first[15] == pytest.approx([0.0, -0.09, 0.1], abs=1e-12)


def test_small_fixture_and_partial_task_map_report_exact_gaps() -> None:
    document = canonical(ROOT / "examples" / "simple_motion.bvh")
    task_map = load_yaml_object(ROOT / "examples" / "ik_task_map_fixture.yaml")

    report = landmark_coverage_report(document, task_map)

    assert validate_landmark_coverage(report, SCHEMA) == []
    assert report["status"] == "warning"
    assert report["source"]["present"] == ["Hips", "LeftKnee"]
    assert report["task_coverage"]["present"] == ["LeftKnee"]
    assert len(report["warnings"]) == 2


def test_landmark_report_cli_preserves_report_when_strict_gate_fails(
    tmp_path: Path,
) -> None:
    document = canonical(ROOT / "examples" / "simple_motion.bvh")
    canonical_path = tmp_path / "canonical.json"
    output = tmp_path / "coverage.json"
    canonical_path.write_text(json.dumps(document))

    result = main(
        [
            "landmark-report",
            str(canonical_path),
            "--task-map",
            str(ROOT / "examples" / "ik_task_map_fixture.yaml"),
            "--output",
            str(output),
            "--require-full-source",
            "--require-full-tasks",
        ]
    )

    assert result == 1
    report = json.loads(output.read_text())
    assert report["status"] == "warning"
    assert report["source"]["complete"] is False
    assert report["task_coverage"]["complete"] is False
