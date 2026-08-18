"""Trajectory derivative generation and robot-profile quality gates."""

from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from typing import Any

from jsonschema import Draft202012Validator

from .errors import OhmcError


LIMIT_ABSOLUTE_TOLERANCE = 1e-9


def _stable_hash(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _first_derivative_weights(nodes: list[float], evaluation: float) -> list[float]:
    weights: list[float] = []
    for joint, node in enumerate(nodes):
        denominator = math.prod(
            node - other for index, other in enumerate(nodes) if index != joint
        )
        numerator = 0.0
        for omitted in range(len(nodes)):
            if omitted == joint:
                continue
            product = 1.0
            for index, other in enumerate(nodes):
                if index not in {joint, omitted}:
                    product *= evaluation - other
            numerator += product
        weights.append(numerator / denominator)
    return weights


def _second_derivative_weights(nodes: list[float]) -> list[float]:
    if len(nodes) != 3:
        raise OhmcError("second derivative requires exactly three timestamps")
    weights = []
    for joint, node in enumerate(nodes):
        denominator = math.prod(
            node - other for index, other in enumerate(nodes) if index != joint
        )
        weights.append(2.0 / denominator)
    return weights


def _window(sample_count: int, index: int) -> list[int]:
    if sample_count < 3:
        return list(range(sample_count))
    if index == 0:
        return [0, 1, 2]
    if index == sample_count - 1:
        return [sample_count - 3, sample_count - 2, sample_count - 1]
    return [index - 1, index, index + 1]


def derive_motion_kinematics(
    document: dict[str, Any], *, overwrite: bool = False
) -> dict[str, Any]:
    """Derive velocity and acceleration with non-uniform Lagrange stencils."""
    trajectory = document.get("trajectory")
    if not isinstance(trajectory, dict):
        raise OhmcError("Motion IR has no trajectory object")
    joints = trajectory.get("joints")
    samples = trajectory.get("samples")
    if not isinstance(joints, list) or not joints:
        raise OhmcError("Motion IR trajectory has no joints")
    if not isinstance(samples, list) or not samples:
        raise OhmcError("Motion IR trajectory has no samples")
    if not overwrite and any(
        "velocity_targets" in sample or "acceleration_targets" in sample
        for sample in samples
    ):
        raise OhmcError(
            "Motion IR already contains velocity or acceleration targets; "
            "use overwrite=True only after reviewing their provenance"
        )

    times: list[float] = []
    positions: list[list[float]] = []
    previous: float | None = None
    for sample_index, sample in enumerate(samples):
        timestamp = float(sample["time"])
        if not math.isfinite(timestamp) or (
            previous is not None and timestamp <= previous
        ):
            raise OhmcError(
                f"sample {sample_index} timestamp must be finite and strictly increasing"
            )
        vector = [float(value) for value in sample["position_targets"]]
        if len(vector) != len(joints):
            raise OhmcError(
                f"sample {sample_index} has {len(vector)} positions for "
                f"{len(joints)} joints"
            )
        if any(not math.isfinite(value) for value in vector):
            raise OhmcError(f"sample {sample_index} positions must be finite")
        times.append(timestamp)
        positions.append(vector)
        previous = timestamp

    velocities: list[list[float]] = []
    accelerations: list[list[float]] = []
    for sample_index, timestamp in enumerate(times):
        indices = _window(len(samples), sample_index)
        nodes = [times[index] for index in indices]
        if len(nodes) == 1:
            first_weights = [0.0]
            second_weights = [0.0]
        elif len(nodes) == 2:
            interval = nodes[1] - nodes[0]
            first_weights = [-1.0 / interval, 1.0 / interval]
            second_weights = [0.0, 0.0]
        else:
            first_weights = _first_derivative_weights(nodes, timestamp)
            second_weights = _second_derivative_weights(nodes)
        velocities.append(
            [
                sum(
                    first_weights[weight_index] * positions[position_index][joint]
                    for weight_index, position_index in enumerate(indices)
                )
                for joint in range(len(joints))
            ]
        )
        accelerations.append(
            [
                sum(
                    second_weights[weight_index] * positions[position_index][joint]
                    for weight_index, position_index in enumerate(indices)
                )
                for joint in range(len(joints))
            ]
        )

    output = deepcopy(document)
    for index, sample in enumerate(output["trajectory"]["samples"]):
        sample["velocity_targets"] = velocities[index]
        sample["acceleration_targets"] = accelerations[index]
    config = {
        "method": "three_point_nonuniform_lagrange",
        "endpoint_method": "one_sided_quadratic",
        "one_sample_derivative": 0.0,
        "two_sample_acceleration": 0.0,
    }
    output["passes"].append(
        {
            "name": "derive_trajectory_kinematics",
            "version": "0.1.0",
            "config_sha256": _stable_hash(config),
            "warnings": [],
        }
    )
    return output


def trajectory_quality_report(
    document: dict[str, Any], profile: dict[str, Any]
) -> dict[str, Any]:
    trajectory = document["trajectory"]
    joints = trajectory["joints"]
    samples = trajectory["samples"]
    if not joints or not samples:
        raise OhmcError("trajectory quality requires at least one joint and sample")
    profile_order = profile["control"]["joint_order"]
    profile_set = set(profile_order)
    mapped_joints = [joint for joint in profile_order if joint in joints]
    missing_joints = [joint for joint in profile_order if joint not in joints]
    unknown_joints = sorted(set(joints) - profile_set)
    mapping_complete = not missing_joints and not unknown_joints
    if not mapped_joints:
        raise OhmcError("trajectory has no joints from the selected robot profile")

    warnings: list[str] = []
    violations: list[dict[str, Any]] = []
    for joint in unknown_joints:
        violations.append(
            {
                "kind": "unknown_joint",
                "joint": joint,
                "sample_index": None,
                "value": None,
                "limit": None,
            }
        )
    if missing_joints:
        warnings.append(
            f"mapping covers {len(mapped_joints)}/{len(profile_order)} controllable "
            "joints; missing: " + ", ".join(missing_joints)
        )

    joint_metrics = []
    velocity_configured = 0
    acceleration_configured = 0
    joint_index = {joint: index for index, joint in enumerate(joints)}
    missing_velocity_limits: list[str] = []
    missing_acceleration_limits: list[str] = []
    for joint in mapped_joints:
        vector_index = joint_index[joint]
        limit = profile["joint_limits"][joint]
        lower = float(limit["lower"])
        upper = float(limit["upper"])
        positions = [float(sample["position_targets"][vector_index]) for sample in samples]
        try:
            velocities = [
                float(sample["velocity_targets"][vector_index]) for sample in samples
            ]
            accelerations = [
                float(sample["acceleration_targets"][vector_index]) for sample in samples
            ]
        except (KeyError, IndexError, TypeError) as exc:
            raise OhmcError(
                "trajectory quality requires velocity and acceleration targets; "
                "run derive_motion_kinematics first"
            ) from exc

        for sample_index, value in enumerate(positions):
            if value < lower - LIMIT_ABSOLUTE_TOLERANCE:
                violations.append(
                    {
                        "kind": "position_lower",
                        "joint": joint,
                        "sample_index": sample_index,
                        "value": value,
                        "limit": lower,
                    }
                )
            if value > upper + LIMIT_ABSOLUTE_TOLERANCE:
                violations.append(
                    {
                        "kind": "position_upper",
                        "joint": joint,
                        "sample_index": sample_index,
                        "value": value,
                        "limit": upper,
                    }
                )

        velocity_limit = limit.get("velocity")
        maximum_velocity = max(abs(value) for value in velocities)
        if velocity_limit is None:
            velocity_status = "not_configured"
            missing_velocity_limits.append(joint)
        else:
            velocity_configured += 1
            velocity_limit = float(velocity_limit)
            velocity_status = "pass"
            for sample_index, value in enumerate(velocities):
                if abs(value) > velocity_limit + LIMIT_ABSOLUTE_TOLERANCE:
                    velocity_status = "fail"
                    violations.append(
                        {
                            "kind": "velocity",
                            "joint": joint,
                            "sample_index": sample_index,
                            "value": abs(value),
                            "limit": velocity_limit,
                        }
                    )

        acceleration_limit = limit.get("acceleration")
        maximum_acceleration = max(abs(value) for value in accelerations)
        if acceleration_limit is None:
            acceleration_status = "not_configured"
            missing_acceleration_limits.append(joint)
        else:
            acceleration_configured += 1
            acceleration_limit = float(acceleration_limit)
            acceleration_status = "pass"
            for sample_index, value in enumerate(accelerations):
                if abs(value) > acceleration_limit + LIMIT_ABSOLUTE_TOLERANCE:
                    acceleration_status = "fail"
                    violations.append(
                        {
                            "kind": "acceleration",
                            "joint": joint,
                            "sample_index": sample_index,
                            "value": abs(value),
                            "limit": acceleration_limit,
                        }
                    )

        joint_metrics.append(
            {
                "joint": joint,
                "minimum_position": min(positions),
                "maximum_position": max(positions),
                "minimum_position_margin": min(
                    min(value - lower, upper - value) for value in positions
                ),
                "maximum_absolute_velocity": maximum_velocity,
                "velocity_limit": velocity_limit,
                "velocity_status": velocity_status,
                "maximum_absolute_acceleration": maximum_acceleration,
                "acceleration_limit": acceleration_limit,
                "acceleration_status": acceleration_status,
            }
        )

    if missing_velocity_limits:
        warnings.append(
            "velocity limits are not configured for mapped joints: "
            + ", ".join(missing_velocity_limits)
        )
    if missing_acceleration_limits:
        warnings.append(
            "acceleration limits are not configured for mapped joints: "
            + ", ".join(missing_acceleration_limits)
        )
    status = "fail" if violations else ("warning" if warnings else "pass")
    duration = float(samples[-1]["time"]) - float(samples[0]["time"])
    return {
        "schema": "ohmc.trajectory_quality/v0.1",
        "robot_profile": profile["id"],
        "configuration": {
            "limit_absolute_tolerance": LIMIT_ABSOLUTE_TOLERANCE,
        },
        "summary": {
            "joint_count": len(joints),
            "sample_count": len(samples),
            "duration_seconds": duration,
        },
        "mapping": {
            "complete": mapping_complete,
            "coverage_ratio": len(mapped_joints) / len(profile_order),
            "mapped_joint_count": len(mapped_joints),
            "controllable_joint_count": len(profile_order),
            "mapped_joints": mapped_joints,
            "missing_joints": missing_joints,
            "unknown_joints": unknown_joints,
        },
        "dynamic_limit_coverage": {
            "velocity_configured_joint_count": velocity_configured,
            "acceleration_configured_joint_count": acceleration_configured,
            "mapped_joint_count": len(mapped_joints),
        },
        "joint_metrics": joint_metrics,
        "violations": violations,
        "warnings": warnings,
        "status": status,
        "hardware_commands_sent": False,
    }


def validate_quality_report(
    document: dict[str, Any], schema: dict[str, Any]
) -> list[str]:
    issues: list[str] = []
    for error in sorted(
        Draft202012Validator(schema).iter_errors(document),
        key=lambda item: list(item.path),
    ):
        location = ".".join(str(part) for part in error.absolute_path) or "$"
        issues.append(f"{location}: {error.message}")
    return issues
