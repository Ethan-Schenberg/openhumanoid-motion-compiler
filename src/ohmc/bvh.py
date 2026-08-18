"""Deterministic BVH ingestion for the offline OHMC prototype."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any

from .errors import OhmcError


_NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
_HIERARCHY_TOKEN = re.compile(r"\{|\}|[^\s{}]+")


@dataclass(frozen=True)
class BvhJoint:
    """One declared BVH joint, excluding anonymous End Sites."""

    name: str
    parent: str | None
    offset: tuple[float, float, float]
    channels: tuple[str, ...]


@dataclass(frozen=True)
class BvhMotion:
    """Parsed BVH hierarchy and channel samples."""

    joints: tuple[BvhJoint, ...]
    channel_bindings: tuple[tuple[str, str], ...]
    frame_time: float
    frames: tuple[tuple[float, ...], ...]

    @property
    def duration(self) -> float:
        return max(0, len(self.frames) - 1) * self.frame_time


class _HierarchyParser:
    def __init__(self, tokens: list[str]) -> None:
        self.tokens = tokens
        self.index = 0
        self.joints: list[BvhJoint] = []
        self.channel_bindings: list[tuple[str, str]] = []
        self.names: set[str] = set()

    def peek(self) -> str | None:
        return self.tokens[self.index] if self.index < len(self.tokens) else None

    def take(self) -> str:
        token = self.peek()
        if token is None:
            raise OhmcError("unexpected end of BVH hierarchy")
        self.index += 1
        return token

    def expect(self, expected: str) -> None:
        actual = self.take()
        if actual.upper() != expected.upper():
            raise OhmcError(
                f"expected {expected!r} in BVH hierarchy, got {actual!r}"
            )

    def take_float(self, context: str) -> float:
        token = self.take()
        try:
            value = float(token)
        except ValueError as exc:
            raise OhmcError(f"invalid {context} value: {token!r}") from exc
        if not math.isfinite(value):
            raise OhmcError(f"non-finite {context} value: {token!r}")
        return value

    def parse(self) -> tuple[tuple[BvhJoint, ...], tuple[tuple[str, str], ...]]:
        self.expect("HIERARCHY")
        self.parse_joint(parent=None, expected_kind="ROOT")
        if self.peek() is not None:
            raise OhmcError(f"unexpected BVH hierarchy token: {self.peek()!r}")
        return tuple(self.joints), tuple(self.channel_bindings)

    def parse_joint(self, parent: str | None, expected_kind: str = "JOINT") -> None:
        self.expect(expected_kind)
        name = self.take()
        if name in self.names:
            raise OhmcError(f"duplicate BVH joint name: {name}")
        self.names.add(name)
        self.expect("{")
        self.expect("OFFSET")
        offset = (
            self.take_float("OFFSET"),
            self.take_float("OFFSET"),
            self.take_float("OFFSET"),
        )
        self.expect("CHANNELS")
        count_token = self.take()
        try:
            channel_count = int(count_token)
        except ValueError as exc:
            raise OhmcError(f"invalid CHANNELS count: {count_token!r}") from exc
        if channel_count < 0:
            raise OhmcError("CHANNELS count cannot be negative")
        channels = tuple(self.take() for _ in range(channel_count))
        normalized_channels = [channel.lower() for channel in channels]
        if len(normalized_channels) != len(set(normalized_channels)):
            raise OhmcError(f"duplicate BVH channel in joint {name!r}")
        for channel in channels:
            lower = channel.lower()
            if lower not in {
                "xposition",
                "yposition",
                "zposition",
                "xrotation",
                "yrotation",
                "zrotation",
            }:
                raise OhmcError(f"unsupported BVH channel: {channel!r}")
            self.channel_bindings.append((name, channel))
        self.joints.append(BvhJoint(name, parent, offset, channels))

        while True:
            token = self.peek()
            if token is None:
                raise OhmcError(f"unterminated BVH joint block: {name}")
            upper = token.upper()
            if token == "}":
                self.take()
                return
            if upper == "JOINT":
                self.parse_joint(parent=name)
                continue
            if upper == "END":
                self.parse_end_site()
                continue
            raise OhmcError(f"unexpected token in joint {name!r}: {token!r}")

    def parse_end_site(self) -> None:
        self.expect("END")
        self.expect("SITE")
        self.expect("{")
        self.expect("OFFSET")
        for _ in range(3):
            self.take_float("End Site OFFSET")
        self.expect("}")


def parse_bvh_text(text: str) -> BvhMotion:
    """Parse a BVH document and reject ambiguous or malformed input."""
    motion_match = re.search(r"(?im)^\s*MOTION\s*$", text)
    if motion_match is None:
        raise OhmcError("BVH is missing the MOTION section")

    hierarchy_text = text[: motion_match.start()]
    motion_text = text[motion_match.end() :]
    parser = _HierarchyParser(_HIERARCHY_TOKEN.findall(hierarchy_text))
    joints, channel_bindings = parser.parse()
    if not channel_bindings:
        raise OhmcError("BVH declares no motion channels")

    frames_match = re.search(r"(?im)^\s*Frames\s*:\s*(\d+)\s*$", motion_text)
    frame_time_match = re.search(
        rf"(?im)^\s*Frame\s+Time\s*:\s*({_NUMBER})\s*$", motion_text
    )
    if frames_match is None or frame_time_match is None:
        raise OhmcError("BVH MOTION section must declare Frames and Frame Time")
    if frames_match.start() > frame_time_match.start():
        raise OhmcError("BVH Frames declaration must precede Frame Time")

    frame_count = int(frames_match.group(1))
    if frame_count < 1:
        raise OhmcError("BVH must contain at least one frame")
    frame_time = float(frame_time_match.group(1))
    if not math.isfinite(frame_time) or frame_time <= 0:
        raise OhmcError("BVH Frame Time must be finite and greater than zero")

    sample_text = motion_text[frame_time_match.end() :]
    raw_values = sample_text.split()
    expected_values = frame_count * len(channel_bindings)
    if len(raw_values) != expected_values:
        raise OhmcError(
            "BVH frame data count mismatch: "
            f"expected {expected_values} values ({frame_count} frames x "
            f"{len(channel_bindings)} channels), got {len(raw_values)}"
        )

    values: list[float] = []
    for token in raw_values:
        try:
            value = float(token)
        except ValueError as exc:
            raise OhmcError(f"invalid BVH frame value: {token!r}") from exc
        if not math.isfinite(value):
            raise OhmcError(f"non-finite BVH frame value: {token!r}")
        values.append(value)

    channel_count = len(channel_bindings)
    frames = tuple(
        tuple(values[start : start + channel_count])
        for start in range(0, len(values), channel_count)
    )
    return BvhMotion(joints, channel_bindings, frame_time, frames)


def load_bvh(path: Path) -> BvhMotion:
    try:
        return parse_bvh_text(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise OhmcError(f"file not found: {path}") from exc
    except UnicodeDecodeError as exc:
        raise OhmcError(f"BVH is not valid UTF-8: {path}") from exc


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _config_hash(config: dict[str, Any]) -> str:
    encoded = json.dumps(
        config, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return _sha256_bytes(encoded)


def bvh_to_motion_ir(
    motion: BvhMotion,
    source_bytes: bytes,
    source_name: str,
    source_license: str,
) -> dict[str, Any]:
    """Emit a prototype Motion IR containing ordered BVH rotation channels.

    This is an ingestion artifact, not a retargeted robot trajectory. Translation
    channels remain source metadata and are intentionally excluded from the joint
    vector until canonical root transforms are added to the public schema.
    """
    rotation_indices = [
        index
        for index, (_, channel) in enumerate(motion.channel_bindings)
        if channel.lower().endswith("rotation")
    ]
    if not rotation_indices:
        raise OhmcError("BVH has no rotation channels to emit")

    joint_names = [
        f"{motion.channel_bindings[index][0]}.{motion.channel_bindings[index][1][0].lower()}_rotation"
        for index in rotation_indices
    ]
    if len(set(joint_names)) != len(joint_names):
        raise OhmcError("BVH rotation channels do not produce unique joint names")

    skipped = [
        f"{joint}.{channel}"
        for joint, channel in motion.channel_bindings
        if channel.lower().endswith("position")
    ]
    translation_warning = (
        "source translation channels were not emitted: " + ", ".join(skipped)
        if skipped
        else None
    )
    config = {
        "angle_input_unit": "degree",
        "angle_output_unit": "radian",
        "emitted_channels": joint_names,
        "skipped_translation_channels": skipped,
    }
    samples = []
    for frame_index, frame in enumerate(motion.frames):
        samples.append(
            {
                "time": frame_index * motion.frame_time,
                "position_targets": [
                    math.radians(frame[index]) for index in rotation_indices
                ],
            }
        )

    warnings = [
        "BVH joint axes and hierarchy are preserved; canonical axis remapping "
        "and robot retargeting have not been applied"
    ]
    if translation_warning:
        warnings.append(translation_warning)
    return {
        "schema": "ohmc.motion_ir/v0.1",
        "source": {
            "kind": "bvh",
            "sha256": _sha256_bytes(source_bytes),
            "license": source_license,
            "uri": source_name,
        },
        "robot": {
            "profile": "bvh_rotation_channels_v1",
            "model_sha256": "0" * 64,
        },
        "frames": {
            "convention": "right_handed_x_forward_y_left_z_up",
            "world": "world",
            "base": motion.joints[0].name,
        },
        "trajectory": {
            "rate_hz": 1.0 / motion.frame_time,
            "joints": joint_names,
            "samples": samples,
        },
        "passes": [
            {
                "name": "bvh_rotation_channel_import",
                "version": "0.1.0",
                "config_sha256": _config_hash(config),
                "warnings": warnings,
            }
        ],
        "validation": {
            "status": "warning" if warnings else "pass",
            "issues": warnings,
        },
    }
