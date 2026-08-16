"""Offline replay backends for Motion IR artifacts."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from .errors import OhmcError


def replay_mujoco(
    document: dict[str, Any], model_path: Path
) -> dict[str, Any]:
    """Apply Motion IR joint positions to a MuJoCo model without rendering.

    This backend performs kinematic replay with ``mj_forward``. It deliberately
    does not run a controller, simulate actuators, open a viewer, or communicate
    with robot hardware.
    """
    try:
        import mujoco
    except ImportError as exc:
        raise OhmcError(
            "MuJoCo replay requires the optional dependency: "
            "python -m pip install -e '.[mujoco]'"
        ) from exc

    try:
        model = mujoco.MjModel.from_xml_path(str(model_path))
    except (ValueError, OSError) as exc:
        raise OhmcError(f"failed to load MuJoCo model {model_path}: {exc}") from exc
    data = mujoco.MjData(model)

    trajectory = document["trajectory"]
    joint_names = trajectory["joints"]
    joint_ids: list[int] = []
    qpos_addresses: list[int] = []
    for name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise OhmcError(f"Motion IR joint not found in MuJoCo model: {name}")
        joint_type = model.jnt_type[joint_id]
        if joint_type not in {
            mujoco.mjtJoint.mjJNT_HINGE,
            mujoco.mjtJoint.mjJNT_SLIDE,
        }:
            raise OhmcError(
                f"Motion IR joint must map to a scalar hinge or slide joint: {name}"
            )
        joint_ids.append(joint_id)
        qpos_addresses.append(int(model.jnt_qposadr[joint_id]))

    maximum_absolute_position = 0.0
    for sample_index, sample in enumerate(trajectory["samples"]):
        for vector_index, value in enumerate(sample["position_targets"]):
            value = float(value)
            joint_id = joint_ids[vector_index]
            name = joint_names[vector_index]
            if bool(model.jnt_limited[joint_id]):
                lower, upper = model.jnt_range[joint_id]
                if value < lower or value > upper:
                    raise OhmcError(
                        f"sample {sample_index} joint {name} violates MuJoCo range "
                        f"[{lower}, {upper}]: {value}"
                    )
            data.qpos[qpos_addresses[vector_index]] = value
            maximum_absolute_position = max(maximum_absolute_position, abs(value))
        data.time = float(sample["time"])
        mujoco.mj_forward(model, data)
        if not all(math.isfinite(float(value)) for value in data.xpos.flat):
            raise OhmcError(
                f"MuJoCo produced a non-finite body position at sample {sample_index}"
            )

    samples = trajectory["samples"]
    return {
        "backend": "mujoco",
        "mode": "headless_kinematic_mj_forward",
        "model": model_path.name,
        "frames_replayed": len(samples),
        "joints_mapped": len(joint_names),
        "duration_seconds": float(samples[-1]["time"]),
        "maximum_absolute_position": maximum_absolute_position,
        "hardware_commands_sent": False,
        "status": "pass",
    }
