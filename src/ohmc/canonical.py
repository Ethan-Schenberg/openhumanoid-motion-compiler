"""Canonical skeleton transforms and deterministic BVH forward kinematics."""

from __future__ import annotations

import hashlib
import json
import math
from typing import Any

from jsonschema import Draft202012Validator

from .bvh import BvhMotion
from .errors import OhmcError


CANONICAL_CONVENTION = "right_handed_x_forward_y_left_z_up"
BVH_Y_UP_CONVENTION = "right_handed_x_right_y_up_z_backward"
SUPPORTED_SOURCE_CONVENTIONS = (CANONICAL_CONVENTION, BVH_Y_UP_CONVENTION)
LENGTH_SCALES = {"m": 1.0, "cm": 0.01, "mm": 0.001}

Matrix3 = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]
Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]

IDENTITY: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


def _matrix_multiply(left: Matrix3, right: Matrix3) -> Matrix3:
    return tuple(
        tuple(
            sum(left[row][axis] * right[axis][column] for axis in range(3))
            for column in range(3)
        )
        for row in range(3)
    )  # type: ignore[return-value]


def _matrix_vector(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(
        sum(matrix[row][axis] * vector[axis] for axis in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _transpose(matrix: Matrix3) -> Matrix3:
    return tuple(
        tuple(matrix[column][row] for column in range(3)) for row in range(3)
    )  # type: ignore[return-value]


def _vector_add(left: Vector3, right: Vector3) -> Vector3:
    return tuple(left[index] + right[index] for index in range(3))  # type: ignore[return-value]


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    """Compose normalized ``xyzw`` quaternions as ``left * right``."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    product = (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )
    norm = math.sqrt(sum(value * value for value in product))
    if norm == 0.0:
        raise OhmcError("cannot normalize a zero quaternion")
    return tuple(value / norm for value in product)  # type: ignore[return-value]


def quaternion_rotate(quaternion: Quaternion, vector: Vector3) -> Vector3:
    """Rotate a vector by a normalized ``xyzw`` quaternion."""
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def forward_kinematics(
    joints: list[dict[str, Any]],
    local_translations: list[Vector3],
    local_rotations: list[Quaternion],
) -> tuple[list[Vector3], list[Quaternion]]:
    """Evaluate canonical local poses for a parent-before-child hierarchy."""
    if len(local_translations) != len(joints) or len(local_rotations) != len(joints):
        raise OhmcError("forward kinematics pose count must match skeleton joints")
    world_positions: list[Vector3] = []
    world_rotations: list[Quaternion] = []
    for joint_index, joint in enumerate(joints):
        parent_index = joint["parent_index"]
        if parent_index is None:
            world_positions.append(local_translations[joint_index])
            world_rotations.append(local_rotations[joint_index])
            continue
        world_positions.append(
            _vector_add(
                world_positions[parent_index],
                quaternion_rotate(
                    world_rotations[parent_index],
                    local_translations[joint_index],
                ),
            )
        )
        world_rotations.append(
            quaternion_multiply(
                world_rotations[parent_index], local_rotations[joint_index]
            )
        )
    return world_positions, world_rotations


def _rotation(axis: str, radians: float) -> Matrix3:
    cosine = math.cos(radians)
    sine = math.sin(radians)
    if axis == "x":
        return ((1.0, 0.0, 0.0), (0.0, cosine, -sine), (0.0, sine, cosine))
    if axis == "y":
        return ((cosine, 0.0, sine), (0.0, 1.0, 0.0), (-sine, 0.0, cosine))
    return ((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0))


def _coordinate_matrix(source_convention: str) -> Matrix3:
    if source_convention == CANONICAL_CONVENTION:
        return IDENTITY
    if source_convention == BVH_Y_UP_CONVENTION:
        # Source +X right, +Y up, +Z backward -> canonical +X forward,
        # +Y left, +Z up. This is a proper rotation with determinant +1.
        return ((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    available = ", ".join(SUPPORTED_SOURCE_CONVENTIONS)
    raise OhmcError(
        f"unsupported BVH source convention {source_convention!r}; "
        f"available conventions: {available}"
    )


def _canonical_rotation(source_rotation: Matrix3, basis: Matrix3) -> Matrix3:
    return _matrix_multiply(_matrix_multiply(basis, source_rotation), _transpose(basis))


def _matrix_to_quaternion(matrix: Matrix3) -> Quaternion:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (matrix[2][1] - matrix[1][2]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[1][0] - matrix[0][1]) / scale,
            0.25 * scale,
        )
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        quaternion = (
            0.25 * scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[2][1] - matrix[1][2]) / scale,
        )
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        quaternion = (
            (matrix[0][1] + matrix[1][0]) / scale,
            0.25 * scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
        )
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        quaternion = (
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            0.25 * scale,
            (matrix[1][0] - matrix[0][1]) / scale,
        )
    norm = math.sqrt(sum(value * value for value in quaternion))
    normalized = tuple(value / norm for value in quaternion)
    if normalized[3] < 0.0:
        normalized = tuple(-value for value in normalized)
    return normalized  # type: ignore[return-value]


def object_sha256(value: Any) -> str:
    """Hash a JSON-compatible value using OHMC's canonical JSON encoding."""
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _joint_channel_indices(motion: BvhMotion) -> dict[str, dict[str, int]]:
    result: dict[str, dict[str, int]] = {joint.name: {} for joint in motion.joints}
    for index, (joint_name, channel) in enumerate(motion.channel_bindings):
        result[joint_name][channel.lower()] = index
    return result


def bvh_to_canonical_motion(
    motion: BvhMotion,
    *,
    source_bytes: bytes,
    source_name: str,
    source_license: str,
    source_convention: str,
    source_length_unit: str,
) -> dict[str, Any]:
    """Evaluate ordered BVH transforms in canonical coordinates.

    Rotation channels are multiplied in their declared BVH order. Local
    translations combine the declared joint offset and any position channels,
    then deterministic forward kinematics emits world poses for every joint.
    """
    if source_length_unit not in LENGTH_SCALES:
        available = ", ".join(LENGTH_SCALES)
        raise OhmcError(
            f"unsupported BVH length unit {source_length_unit!r}; "
            f"available units: {available}"
        )
    basis = _coordinate_matrix(source_convention)
    length_scale = LENGTH_SCALES[source_length_unit]
    joint_indices = {joint.name: index for index, joint in enumerate(motion.joints)}
    channel_indices = _joint_channel_indices(motion)

    skeleton_joints = []
    canonical_offsets: list[Vector3] = []
    for joint in motion.joints:
        source_offset = tuple(value * length_scale for value in joint.offset)
        canonical_offset = _matrix_vector(basis, source_offset)  # type: ignore[arg-type]
        canonical_offsets.append(canonical_offset)
        skeleton_joints.append(
            {
                "name": joint.name,
                "parent_index": (
                    joint_indices[joint.parent] if joint.parent is not None else None
                ),
                "rest_offset_m": list(canonical_offset),
            }
        )

    samples = []
    for frame_index, frame in enumerate(motion.frames):
        local_rotations: list[Matrix3] = []
        local_translations: list[Vector3] = []
        root_translation: Vector3 = (0.0, 0.0, 0.0)
        for joint_index, joint in enumerate(motion.joints):
            source_rotation = IDENTITY
            source_translation = [0.0, 0.0, 0.0]
            indices = channel_indices[joint.name]
            for channel in joint.channels:
                lower = channel.lower()
                value = frame[indices[lower]]
                if lower.endswith("rotation"):
                    source_rotation = _matrix_multiply(
                        source_rotation, _rotation(lower[0], math.radians(value))
                    )
                elif lower.endswith("position"):
                    source_translation["xyz".index(lower[0])] = value * length_scale
            canonical_translation = _matrix_vector(
                basis,
                tuple(source_translation),  # type: ignore[arg-type]
            )
            if joint_index == 0:
                root_translation = canonical_translation
            local_translations.append(
                _vector_add(canonical_offsets[joint_index], canonical_translation)
            )
            local_rotations.append(_canonical_rotation(source_rotation, basis))

        world_positions: list[Vector3] = []
        world_rotations: list[Matrix3] = []
        for joint_index, joint in enumerate(motion.joints):
            if joint.parent is None:
                world_positions.append(local_translations[joint_index])
                world_rotations.append(local_rotations[joint_index])
                continue
            parent_index = joint_indices[joint.parent]
            world_positions.append(
                _vector_add(
                    world_positions[parent_index],
                    _matrix_vector(
                        world_rotations[parent_index],
                        local_translations[joint_index],
                    ),
                )
            )
            world_rotations.append(
                _matrix_multiply(
                    world_rotations[parent_index], local_rotations[joint_index]
                )
            )

        samples.append(
            {
                "time": frame_index * motion.frame_time,
                "root_translation_m": list(root_translation),
                "local_translations_m": [
                    list(translation) for translation in local_translations
                ],
                "local_rotations_xyzw": [
                    list(_matrix_to_quaternion(matrix)) for matrix in local_rotations
                ],
                "world_positions_m": [list(position) for position in world_positions],
                "world_rotations_xyzw": [
                    list(_matrix_to_quaternion(matrix)) for matrix in world_rotations
                ],
            }
        )

    config = {
        "source_convention": source_convention,
        "source_length_unit": source_length_unit,
        "output_convention": CANONICAL_CONVENTION,
        "output_length_unit": "m",
        "rotation_channel_evaluation": "declared_order_postmultiply",
        "quaternion_order": "xyzw",
    }
    frames = {
        "convention": CANONICAL_CONVENTION,
        "source_convention": source_convention,
        "length_unit": "meter",
    }
    skeleton = {"joints": skeleton_joints}
    canonical_payload = {"frames": frames, "skeleton": skeleton, "samples": samples}
    return {
        "schema": "ohmc.canonical_motion/v0.1",
        "source": {
            "kind": "bvh",
            "sha256": hashlib.sha256(source_bytes).hexdigest(),
            "license": source_license,
            "uri": source_name,
        },
        "frames": frames,
        "skeleton": skeleton,
        "samples": samples,
        "passes": [
            {
                "name": "bvh_canonical_forward_kinematics",
                "version": "0.1.0",
                "config_sha256": object_sha256(config),
                "input_sha256": hashlib.sha256(source_bytes).hexdigest(),
                "output_sha256": object_sha256(canonical_payload),
                "metrics": {
                    "joint_count": len(motion.joints),
                    "frame_count": len(motion.frames),
                    "duration_seconds": motion.duration,
                    "source_length_to_meter_scale": length_scale,
                },
                "warnings": [],
            }
        ],
        "validation": {"status": "pass", "issues": []},
    }


def validate_canonical_motion(
    document: dict[str, Any], schema: dict[str, Any]
) -> list[str]:
    issues: list[str] = []
    validator = Draft202012Validator(schema)
    for error in sorted(validator.iter_errors(document), key=lambda item: list(item.path)):
        location = ".".join(str(part) for part in error.absolute_path) or "$"
        issues.append(f"{location}: {error.message}")
    if issues:
        return issues

    joints = document["skeleton"]["joints"]
    names = [joint["name"] for joint in joints]
    if len(names) != len(set(names)):
        issues.append("skeleton.joints: names must be unique")
    for index, joint in enumerate(joints):
        parent = joint["parent_index"]
        if index == 0 and parent is not None:
            issues.append("skeleton.joints.0.parent_index: root parent must be null")
        elif index > 0 and (parent is None or parent >= index):
            issues.append(
                f"skeleton.joints.{index}.parent_index: must reference an earlier joint"
            )
        if any(
            not math.isfinite(float(value)) for value in joint["rest_offset_m"]
        ):
            issues.append(f"skeleton.joints.{index}.rest_offset_m: must be finite")

    previous_time: float | None = None
    joint_count = len(joints)
    for sample_index, sample in enumerate(document["samples"]):
        timestamp = float(sample["time"])
        if not math.isfinite(timestamp):
            issues.append(f"samples.{sample_index}.time: must be finite")
        elif previous_time is not None and timestamp <= previous_time:
            issues.append(f"samples.{sample_index}.time: must be strictly increasing")
        previous_time = timestamp
        for field in (
            "local_translations_m",
            "local_rotations_xyzw",
            "world_positions_m",
            "world_rotations_xyzw",
        ):
            if len(sample[field]) != joint_count:
                issues.append(
                    f"samples.{sample_index}.{field}: expected {joint_count} entries, "
                    f"got {len(sample[field])}"
                )
        vectors = [
            sample["root_translation_m"],
            *sample["local_translations_m"],
            *sample["world_positions_m"],
        ]
        quaternions = [
            *sample["local_rotations_xyzw"],
            *sample["world_rotations_xyzw"],
        ]
        if any(not math.isfinite(float(value)) for vector in vectors for value in vector):
            issues.append(f"samples.{sample_index}: positions must be finite")
        for quaternion_index, quaternion in enumerate(quaternions):
            if any(not math.isfinite(float(value)) for value in quaternion):
                issues.append(
                    f"samples.{sample_index}.quaternion.{quaternion_index}: must be finite"
                )
                continue
            norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
            if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-9):
                issues.append(
                    f"samples.{sample_index}.quaternion.{quaternion_index}: "
                    f"must have unit norm, got {norm}"
                )
    return issues
