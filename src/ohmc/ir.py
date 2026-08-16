"""Motion IR loading and validation."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

from .errors import OhmcError


def load_json(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise OhmcError(f"file not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise OhmcError(f"invalid JSON in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise OhmcError(f"expected a JSON object in {path}")
    return data


def validate_motion_ir(document: dict[str, Any], schema: dict[str, Any]) -> list[str]:
    """Return stable, human-readable validation errors."""
    issues: list[str] = []
    validator = Draft202012Validator(schema)
    for error in sorted(validator.iter_errors(document), key=lambda item: list(item.path)):
        location = ".".join(str(part) for part in error.absolute_path) or "$"
        issues.append(f"{location}: {error.message}")

    trajectory = document.get("trajectory")
    if not isinstance(trajectory, dict):
        return issues

    joints = trajectory.get("joints")
    samples = trajectory.get("samples")
    if not isinstance(joints, list) or not isinstance(samples, list):
        return issues

    previous_time: float | None = None
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            continue
        timestamp = sample.get("time")
        positions = sample.get("position_targets")
        if isinstance(timestamp, (int, float)) and not isinstance(timestamp, bool):
            timestamp = float(timestamp)
            if not math.isfinite(timestamp):
                issues.append(f"trajectory.samples.{index}.time: must be finite")
            elif previous_time is not None and timestamp <= previous_time:
                issues.append(
                    f"trajectory.samples.{index}.time: must be strictly greater than "
                    f"the previous timestamp ({previous_time})"
                )
            previous_time = timestamp
        if isinstance(positions, list) and len(positions) != len(joints):
            issues.append(
                f"trajectory.samples.{index}.position_targets: expected {len(joints)} "
                f"values for {len(joints)} joints, got {len(positions)}"
            )
        if isinstance(positions, list):
            for position_index, value in enumerate(positions):
                if (
                    isinstance(value, (int, float))
                    and not isinstance(value, bool)
                    and not math.isfinite(float(value))
                ):
                    issues.append(
                        "trajectory.samples."
                        f"{index}.position_targets.{position_index}: must be finite"
                    )
    return issues

