"""Offline, deliberately non-executable vendor interface fixtures."""

from __future__ import annotations

import hashlib
import json
from typing import Any

from .errors import OhmcError


UNITREE_G1_ADAPTER = "unitree-g1-lowcmd"
AGIBOT_X2_ADAPTER = "agibot-x2-joint-command-array"
SUPPORTED_ADAPTERS = (UNITREE_G1_ADAPTER, AGIBOT_X2_ADAPTER)


def _stable_hash(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _check_inputs(
    document: dict[str, Any], profile: dict[str, Any], adapter: str
) -> dict[str, int]:
    expected = {
        UNITREE_G1_ADAPTER: ("Unitree", "unitree_g1_29dof_mujoco_v1"),
        AGIBOT_X2_ADAPTER: ("AgiBot", "agibot_x2_ultra_aimdk_v1"),
    }
    if adapter not in expected:
        raise OhmcError(f"unsupported vendor adapter: {adapter}")
    vendor, profile_id = expected[adapter]
    if profile.get("vendor") != vendor or profile.get("id") != profile_id:
        raise OhmcError(f"adapter {adapter} requires robot profile {profile_id}")
    if profile["control"]["hardware_transport"] != "disabled":
        raise OhmcError("vendor fixture encoding requires hardware_transport: disabled")

    robot = document.get("robot")
    if not isinstance(robot, dict) or robot.get("profile") != profile["id"]:
        raise OhmcError("Motion IR must first be mapped with the selected robot profile")
    if robot.get("model_sha256") != profile["model_evidence"]["model_sha256"]:
        raise OhmcError("Motion IR robot model hash does not match the selected profile")

    joints = document["trajectory"]["joints"]
    if len(joints) != len(set(joints)):
        raise OhmcError("Motion IR trajectory joints must be unique")
    unknown = sorted(set(joints) - set(profile["control"]["joint_order"]))
    if unknown:
        raise OhmcError(f"Motion IR contains joints outside the profile: {unknown}")
    return {name: index for index, name in enumerate(joints)}


def _base_fixture(
    document: dict[str, Any], profile: dict[str, Any], adapter: str, interface: str
) -> dict[str, Any]:
    mapped = set(document["trajectory"]["joints"])
    required = set(profile["control"]["joint_order"])
    return {
        "schema": "ohmc.vendor_interface_fixture/v0.1",
        "adapter": adapter,
        "interface": interface,
        "vendor": profile["vendor"],
        "robot_profile": profile["id"],
        "model_sha256": profile["model_evidence"]["model_sha256"],
        "source_motion_sha256": _stable_hash(document),
        "purpose": "interface_order_conformance_only",
        "transport": "disabled",
        "executable": False,
        "complete": mapped == required,
        "warnings": [
            "offline JSON fixture only; this is not serialized DDS or ROS 2 data",
            "control modes, gains, efforts, damping, and hardware transport are unset",
            "missing joint targets remain null and are never replaced with zero",
        ],
        "frames": [],
    }


def _unitree_fixture(
    document: dict[str, Any], profile: dict[str, Any], joint_index: dict[str, int]
) -> dict[str, Any]:
    fixture = _base_fixture(
        document, profile, UNITREE_G1_ADAPTER, "unitree_hg.msg.dds_.LowCmd_"
    )
    for sample in document["trajectory"]["samples"]:
        positions = sample["position_targets"]
        motor_cmd = []
        for index, name in enumerate(profile["control"]["joint_order"]):
            present = name in joint_index
            motor_cmd.append(
                {
                    "index": index,
                    "name": name,
                    "present": present,
                    "mode": None,
                    "q": float(positions[joint_index[name]]) if present else None,
                    "dq": None,
                    "kp": None,
                    "kd": None,
                    "tau": None,
                }
            )
        fixture["frames"].append(
            {
                "time": float(sample["time"]),
                "message": {
                    "topic": "rt/lowcmd",
                    "mode_pr": None,
                    "mode_machine": None,
                    "motor_cmd": motor_cmd,
                },
            }
        )
    return fixture


def _agibot_group_order(profile: dict[str, Any]) -> list[tuple[str, str, list[str]]]:
    groups = profile["groups"]
    return [
        (
            "leg",
            "/aima/hal/joint/leg/command",
            groups["left_leg"] + groups["right_leg"],
        ),
        ("waist", "/aima/hal/joint/waist/command", groups["waist"]),
        (
            "arm",
            "/aima/hal/joint/arm/command",
            groups["left_arm"] + groups["right_arm"],
        ),
        (
            "head",
            "/aima/hal/joint/head/command",
            groups["head"] + ["head_pitch_joint"],
        ),
    ]


def _agibot_fixture(
    document: dict[str, Any], profile: dict[str, Any], joint_index: dict[str, int]
) -> dict[str, Any]:
    fixture = _base_fixture(
        document,
        profile,
        AGIBOT_X2_ADAPTER,
        "aimdk_msgs/msg/JointCommandArray",
    )
    excluded = {item["name"] for item in profile["control"]["excluded_joints"]}
    for sample in document["trajectory"]["samples"]:
        positions = sample["position_targets"]
        messages = []
        for group, topic, names in _agibot_group_order(profile):
            joints = []
            for index, name in enumerate(names):
                available = name not in excluded
                present = available and name in joint_index
                joints.append(
                    {
                        "index": index,
                        "name": name,
                        "available": available,
                        "present": present,
                        "position": (
                            float(positions[joint_index[name]]) if present else None
                        ),
                        "velocity": None,
                        "effort": None,
                        "stiffness": None,
                        "damping": None,
                    }
                )
            messages.append(
                {
                    "group": group,
                    "topic": topic,
                    "interface": "aimdk_msgs/msg/JointCommandArray",
                    "header": None,
                    "joints": joints,
                }
            )
        fixture["frames"].append(
            {"time": float(sample["time"]), "messages": messages}
        )
    return fixture


def encode_vendor_fixture(
    document: dict[str, Any], profile: dict[str, Any], adapter: str
) -> dict[str, Any]:
    """Encode an offline interface-order fixture with no transport or gains."""
    joint_index = _check_inputs(document, profile, adapter)
    if adapter == UNITREE_G1_ADAPTER:
        return _unitree_fixture(document, profile, joint_index)
    return _agibot_fixture(document, profile, joint_index)
