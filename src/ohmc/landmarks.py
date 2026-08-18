"""Canonical full-body landmark coverage reporting."""

from __future__ import annotations

import math
from typing import Any

from jsonschema import Draft202012Validator

from .canonical import object_sha256


FULL_BODY_LANDMARKS = (
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
)


def _coverage(present_names: set[str]) -> dict[str, Any]:
    required = list(FULL_BODY_LANDMARKS)
    present = [name for name in required if name in present_names]
    missing = [name for name in required if name not in present_names]
    return {
        "required": required,
        "present": present,
        "missing": missing,
        "required_count": len(required),
        "present_count": len(present),
        "coverage_ratio": len(present) / len(required),
        "complete": not missing,
    }


def landmark_coverage_report(
    canonical_motion: dict[str, Any], task_map: dict[str, Any] | None = None
) -> dict[str, Any]:
    source_names = {
        joint["name"] for joint in canonical_motion["skeleton"]["joints"]
    }
    source = _coverage(source_names)
    warnings = []
    if source["missing"]:
        warnings.append(
            "canonical source is missing required full-body landmarks: "
            + ", ".join(source["missing"])
        )
    task_coverage = None
    if task_map is not None:
        task_sources = {task["source_joint"] for task in task_map["tasks"]}
        task_coverage = _coverage(task_sources)
        task_coverage["task_map"] = task_map["id"]
        if task_coverage["missing"]:
            warnings.append(
                "IK task map does not cover required full-body landmarks: "
                + ", ".join(task_coverage["missing"])
            )
    payload = {
        "frames": canonical_motion["frames"],
        "skeleton": canonical_motion["skeleton"],
        "samples": canonical_motion["samples"],
    }
    return {
        "schema": "ohmc.landmark_coverage/v0.1",
        "canonical_motion_sha256": object_sha256(payload),
        "standard": "ohmc.full_body_landmarks/v0.1",
        "source": source,
        "task_coverage": task_coverage,
        "status": "warning" if warnings else "pass",
        "warnings": warnings,
        "hardware_commands_sent": False,
    }


def validate_landmark_coverage(
    document: dict[str, Any], schema: dict[str, Any]
) -> list[str]:
    issues: list[str] = []
    for error in sorted(
        Draft202012Validator(schema).iter_errors(document),
        key=lambda item: list(item.path),
    ):
        location = ".".join(str(part) for part in error.absolute_path) or "$"
        issues.append(f"{location}: {error.message}")
    if issues:
        return issues
    for label in ("source", "task_coverage"):
        coverage = document[label]
        if coverage is None:
            continue
        required = coverage["required"]
        present = coverage["present"]
        missing = coverage["missing"]
        if set(present) | set(missing) != set(required) or set(present) & set(missing):
            issues.append(f"{label}: present and missing must partition required")
        if coverage["required_count"] != len(required):
            issues.append(f"{label}.required_count does not match required")
        if coverage["present_count"] != len(present):
            issues.append(f"{label}.present_count does not match present")
        expected_ratio = len(present) / len(required)
        if not math.isclose(
            float(coverage["coverage_ratio"]),
            expected_ratio,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            issues.append(f"{label}.coverage_ratio does not match counts")
        if bool(coverage["complete"]) != (not missing):
            issues.append(f"{label}.complete does not match missing")
    expected_status = "warning" if document["warnings"] else "pass"
    if document["status"] != expected_status:
        issues.append(f"status must be {expected_status!r} for warnings")
    return issues
