"""Deterministic morphology scaling and timeline normalization."""

from __future__ import annotations

from bisect import bisect_right
from copy import deepcopy
import math
from typing import Any, Sequence

from .canonical import (
    Quaternion,
    Vector3,
    forward_kinematics,
    object_sha256,
)
from .errors import OhmcError


TIME_TOLERANCE_SECONDS = 1e-12
SLERP_LINEAR_THRESHOLD = 0.9995


def _vector3(values: Sequence[float]) -> Vector3:
    return (float(values[0]), float(values[1]), float(values[2]))


def _quaternion(values: Sequence[float]) -> Quaternion:
    return (
        float(values[0]),
        float(values[1]),
        float(values[2]),
        float(values[3]),
    )


def _scale_vector(values: Sequence[float], scale: float) -> list[float]:
    return [float(value) * scale for value in values]


def _lerp(left: Sequence[float], right: Sequence[float], alpha: float) -> list[float]:
    return [
        float(left[index])
        + alpha * (float(right[index]) - float(left[index]))
        for index in range(len(left))
    ]


def quaternion_slerp(
    left: Sequence[float], right: Sequence[float], alpha: float
) -> Quaternion:
    """Interpolate normalized ``xyzw`` quaternions along the shortest arc."""
    if not 0.0 <= alpha <= 1.0:
        raise OhmcError(f"SLERP alpha must be within [0, 1], got {alpha}")
    start = _quaternion(left)
    end = _quaternion(right)
    dot = sum(start[index] * end[index] for index in range(4))
    if dot < 0.0:
        end = tuple(-value for value in end)  # type: ignore[assignment]
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > SLERP_LINEAR_THRESHOLD:
        values = tuple(
            start[index] + alpha * (end[index] - start[index])
            for index in range(4)
        )
    else:
        angle = math.acos(dot)
        sine = math.sin(angle)
        left_weight = math.sin((1.0 - alpha) * angle) / sine
        right_weight = math.sin(alpha * angle) / sine
        values = tuple(
            left_weight * start[index] + right_weight * end[index]
            for index in range(4)
        )
    norm = math.sqrt(sum(value * value for value in values))
    if norm == 0.0:  # pragma: no cover - valid unit inputs cannot reach this
        raise OhmcError("SLERP produced a zero quaternion")
    return tuple(value / norm for value in values)  # type: ignore[return-value]


def _target_times(start: float, end: float, rate_hz: float) -> list[float]:
    if math.isclose(start, end, rel_tol=0.0, abs_tol=TIME_TOLERANCE_SECONDS):
        return [start]
    duration = end - start
    whole_steps = math.floor(duration * rate_hz + TIME_TOLERANCE_SECONDS)
    times = [start + index / rate_hz for index in range(whole_steps + 1)]
    if times[-1] > end and math.isclose(
        times[-1], end, rel_tol=0.0, abs_tol=TIME_TOLERANCE_SECONDS
    ):
        times[-1] = end
    elif not math.isclose(
        times[-1], end, rel_tol=0.0, abs_tol=TIME_TOLERANCE_SECONDS
    ):
        times.append(end)
    else:
        times[-1] = end
    return times


def _bracket(times: list[float], timestamp: float) -> tuple[int, int, float]:
    if timestamp <= times[0] + TIME_TOLERANCE_SECONDS:
        return 0, 0, 0.0
    if timestamp >= times[-1] - TIME_TOLERANCE_SECONDS:
        last = len(times) - 1
        return last, last, 0.0
    right = bisect_right(times, timestamp)
    left = right - 1
    alpha = (timestamp - times[left]) / (times[right] - times[left])
    return left, right, alpha


def normalize_canonical_motion(
    document: dict[str, Any], *, morphology_scale: float, rate_hz: float
) -> dict[str, Any]:
    """Scale a canonical skeleton, resample local poses, and recompute world FK."""
    if not math.isfinite(morphology_scale) or morphology_scale <= 0.0:
        raise OhmcError("morphology scale must be finite and greater than zero")
    if not math.isfinite(rate_hz) or rate_hz <= 0.0:
        raise OhmcError("normalization rate must be finite and greater than zero")
    if not document.get("samples"):
        raise OhmcError("canonical motion must contain at least one sample")

    result = deepcopy(document)
    joints = result["skeleton"]["joints"]
    for joint in joints:
        joint["rest_offset_m"] = _scale_vector(
            joint["rest_offset_m"], morphology_scale
        )

    source_samples = document["samples"]
    input_sha256 = object_sha256(
        {
            "frames": document["frames"],
            "skeleton": document["skeleton"],
            "samples": source_samples,
        }
    )
    source_times = [float(sample["time"]) for sample in source_samples]
    target_times = _target_times(source_times[0], source_times[-1], rate_hz)
    normalized_samples: list[dict[str, Any]] = []
    for timestamp in target_times:
        left_index, right_index, alpha = _bracket(source_times, timestamp)
        left = source_samples[left_index]
        right = source_samples[right_index]
        if left_index == right_index:
            root_translation = list(left["root_translation_m"])
            local_translations = [
                list(values) for values in left["local_translations_m"]
            ]
            local_rotations = [
                _quaternion(values) for values in left["local_rotations_xyzw"]
            ]
        else:
            root_translation = _lerp(
                left["root_translation_m"], right["root_translation_m"], alpha
            )
            local_translations = [
                _lerp(left_values, right_values, alpha)
                for left_values, right_values in zip(
                    left["local_translations_m"],
                    right["local_translations_m"],
                )
            ]
            local_rotations = [
                quaternion_slerp(left_values, right_values, alpha)
                for left_values, right_values in zip(
                    left["local_rotations_xyzw"],
                    right["local_rotations_xyzw"],
                )
            ]

        scaled_root = _scale_vector(root_translation, morphology_scale)
        scaled_local = [
            _scale_vector(values, morphology_scale) for values in local_translations
        ]
        world_positions, world_rotations = forward_kinematics(
            joints,
            [_vector3(values) for values in scaled_local],
            local_rotations,
        )
        normalized_samples.append(
            {
                "time": timestamp,
                "root_translation_m": scaled_root,
                "local_translations_m": scaled_local,
                "local_rotations_xyzw": [list(values) for values in local_rotations],
                "world_positions_m": [list(values) for values in world_positions],
                "world_rotations_xyzw": [list(values) for values in world_rotations],
            }
        )

    duration = source_times[-1] - source_times[0]
    final_interval = (
        target_times[-1] - target_times[-2] if len(target_times) > 1 else 0.0
    )
    expected_interval = 1.0 / rate_hz
    warnings = []
    if len(target_times) > 1 and not math.isclose(
        final_interval,
        expected_interval,
        rel_tol=0.0,
        abs_tol=TIME_TOLERANCE_SECONDS,
    ):
        warnings.append(
            "final sample preserves source duration with a shorter-than-nominal interval"
        )
    config = {
        "morphology_scale": morphology_scale,
        "rate_hz": rate_hz,
        "translation_interpolation": "linear",
        "rotation_interpolation": "shortest_arc_slerp",
        "duration_policy": "preserve_exact_final_timestamp",
        "world_pose_policy": "recompute_forward_kinematics",
    }
    result["samples"] = normalized_samples
    output_sha256 = object_sha256(
        {
            "frames": result["frames"],
            "skeleton": result["skeleton"],
            "samples": normalized_samples,
        }
    )
    result["passes"].append(
        {
            "name": "canonical_morphology_timeline_normalization",
            "version": "0.1.0",
            "config_sha256": object_sha256(config),
            "input_sha256": input_sha256,
            "output_sha256": output_sha256,
            "metrics": {
                "morphology_scale": morphology_scale,
                "target_rate_hz": rate_hz,
                "input_sample_count": len(source_samples),
                "output_sample_count": len(normalized_samples),
                "duration_seconds": duration,
            },
            "warnings": warnings,
        }
    )
    return result
