"""AgiBot X2 Ultra articulation contract for the pinned Isaac Lab extension.

The gains below are conservative simulation starting values, not vendor
controller parameters. They require target-GPU simulation validation and must
not be copied into the real-robot controller.
"""

from __future__ import annotations

import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

LOCOMOTION_JOINTS = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_pitch_joint",
    "waist_roll_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
]

EXCLUDED_HEAD_JOINTS = ["head_yaw_joint", "head_pitch_joint"]


def x2_urdf_path() -> Path:
    raw_path = os.environ.get("OHMC_X2_URDF")
    if not raw_path:
        raise RuntimeError(
            "OHMC_X2_URDF is required; run `ohmc doctor` before Isaac Lab"
        )
    path = Path(raw_path).expanduser().resolve()
    if not path.is_file():
        raise RuntimeError(f"OHMC_X2_URDF is not a file: {path}")
    return path


def build_x2_cfg() -> ArticulationCfg:
    """Build the floating-base X2 articulation from the verified official URDF."""

    joint_drive = sim_utils.UrdfFileCfg.JointDriveCfg(
        drive_type="force",
        target_type="position",
        gains=sim_utils.UrdfFileCfg.JointDriveCfg.PDGainsCfg(
            stiffness=0.0,
            damping=0.0,
        ),
    )
    return ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UrdfFileCfg(
            asset_path=str(x2_urdf_path()),
            fix_base=False,
            merge_fixed_joints=False,
            joint_drive=joint_drive,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=10.0,
                enable_gyroscopic_forces=True,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=4,
                sleep_threshold=0.0,
                stabilization_threshold=0.001,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.93),
            joint_pos={
                ".*": 0.0,
                ".*_hip_pitch_joint": -0.30,
                ".*_knee_joint": 0.60,
                ".*_ankle_pitch_joint": -0.30,
                ".*_elbow_joint": -0.35,
            },
            joint_vel={".*": 0.0},
        ),
        soft_joint_pos_limit_factor=0.90,
        actuators={
            "legs": ImplicitActuatorCfg(
                joint_names_expr=[
                    ".*_hip_.*_joint",
                    ".*_knee_joint",
                    ".*_ankle_.*_joint",
                ],
                effort_limit_sim={
                    ".*_hip_.*_joint": 120.0,
                    ".*_knee_joint": 120.0,
                    ".*_ankle_pitch_joint": 36.0,
                    ".*_ankle_roll_joint": 24.0,
                },
                velocity_limit_sim=16.0,
                stiffness={
                    ".*_hip_.*_joint": 100.0,
                    ".*_knee_joint": 120.0,
                    ".*_ankle_pitch_joint": 35.0,
                    ".*_ankle_roll_joint": 25.0,
                },
                damping={
                    ".*_hip_.*_joint": 4.0,
                    ".*_knee_joint": 5.0,
                    ".*_ankle_pitch_joint": 2.0,
                    ".*_ankle_roll_joint": 1.5,
                },
                armature=0.01,
            ),
            "waist": ImplicitActuatorCfg(
                joint_names_expr=["waist_.*_joint"],
                effort_limit_sim=120.0,
                velocity_limit_sim=14.0,
                stiffness=80.0,
                damping=4.0,
                armature=0.01,
            ),
            "arms": ImplicitActuatorCfg(
                joint_names_expr=[
                    ".*_shoulder_.*_joint",
                    ".*_elbow_joint",
                    ".*_wrist_.*_joint",
                ],
                effort_limit_sim={
                    ".*_shoulder_pitch_joint": 36.0,
                    ".*_shoulder_roll_joint": 36.0,
                    ".*_shoulder_yaw_joint": 24.0,
                    ".*_elbow_joint": 24.0,
                    ".*_wrist_yaw_joint": 24.0,
                    ".*_wrist_pitch_joint": 4.8,
                    ".*_wrist_roll_joint": 4.8,
                },
                velocity_limit_sim=16.0,
                stiffness={
                    ".*_shoulder_.*_joint": 30.0,
                    ".*_elbow_joint": 25.0,
                    ".*_wrist_.*_joint": 8.0,
                },
                damping={
                    ".*_shoulder_.*_joint": 2.0,
                    ".*_elbow_joint": 1.5,
                    ".*_wrist_.*_joint": 0.5,
                },
                armature=0.005,
            ),
            "head_hold": ImplicitActuatorCfg(
                joint_names_expr=EXCLUDED_HEAD_JOINTS,
                effort_limit_sim=4.8,
                velocity_limit_sim=4.2,
                stiffness=15.0,
                damping=1.0,
            ),
        },
    )


__all__ = [
    "EXCLUDED_HEAD_JOINTS",
    "LOCOMOTION_JOINTS",
    "build_x2_cfg",
    "x2_urdf_path",
]
