"""Manager-based X2 RGB-D locomotion task for pinned Isaac Lab v3 beta."""

from __future__ import annotations

import copy
import os

import isaaclab.sim as sim_utils
import isaaclab_tasks.manager_based.locomotion.velocity.mdp as base_mdp
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import CameraCfg
from isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG
from isaaclab.utils.configclass import configclass
from isaaclab.utils.noise import UniformNoiseCfg as Unoise
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import (
    ActionsCfg,
    LocomotionVelocityRoughEnvCfg,
    MySceneCfg,
    RewardsCfg,
)

from .. import mdp
from ..assets import LOCOMOTION_JOINTS, build_x2_cfg

CURRICULUM_ORDER = (
    "00_stand",
    "01_flat",
    "02_slope",
    "03_uneven",
    "04_low_obstacle",
    "05_stairs",
)


def _terrain_for_stage(stage: str):
    terrain = copy.deepcopy(ROUGH_TERRAINS_CFG)
    keys = {
        "02_slope": ["hf_pyramid_slope", "hf_pyramid_slope_inv"],
        "03_uneven": ["random_rough"],
        "04_low_obstacle": ["boxes"],
        "05_stairs": ["pyramid_stairs", "pyramid_stairs_inv"],
    }.get(stage)
    if not keys:
        return None
    terrain.sub_terrains = {key: terrain.sub_terrains[key] for key in keys}
    proportion = 1.0 / len(terrain.sub_terrains)
    for sub_terrain in terrain.sub_terrains.values():
        sub_terrain.proportion = proportion
    if stage == "04_low_obstacle":
        terrain.sub_terrains["boxes"].grid_height_range = (0.03, 0.10)
    if stage == "05_stairs":
        for sub_terrain in terrain.sub_terrains.values():
            sub_terrain.step_height_range = (0.03, 0.15)
    return terrain


@configclass
class X2SceneCfg(MySceneCfg):
    front_camera = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/rgbd_head_front/Camera",
        offset=CameraCfg.OffsetCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(1.0, 0.0, 0.0, 0.0),
            convention="ros",
        ),
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=18.0,
            horizontal_aperture=20.955,
            clipping_range=(0.10, 4.0),
        ),
        width=64,
        height=48,
        update_period=1.0 / 15.0,
    )


@configclass
class X2ActionsCfg(ActionsCfg):
    joint_pos = base_mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=LOCOMOTION_JOINTS,
        scale=0.25,
        use_default_offset=True,
    )


@configclass
class X2ObservationsCfg:
    @configclass
    class RgbCfg(ObsGroup):
        image = ObsTerm(
            func=mdp.RandomizedCameraObservation,
            params={
                "sensor_cfg": SceneEntityCfg("front_camera"),
                "data_type": "rgb",
                "max_latency_steps": 2,
            },
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class DepthCfg(ObsGroup):
        image = ObsTerm(
            func=mdp.RandomizedCameraObservation,
            params={
                "sensor_cfg": SceneEntityCfg("front_camera"),
                "data_type": "distance_to_image_plane",
                "max_latency_steps": 2,
            },
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class PolicyCfg(ObsGroup):
        base_ang_vel = ObsTerm(
            func=base_mdp.base_ang_vel,
            noise=Unoise(n_min=-0.2, n_max=0.2),
        )
        projected_gravity = ObsTerm(
            func=base_mdp.projected_gravity,
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        velocity_commands = ObsTerm(
            func=base_mdp.generated_commands,
            params={"command_name": "base_velocity"},
        )
        joint_pos = ObsTerm(
            func=base_mdp.joint_pos_rel,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=LOCOMOTION_JOINTS)
            },
            noise=Unoise(n_min=-0.01, n_max=0.01),
        )
        joint_vel = ObsTerm(
            func=base_mdp.joint_vel_rel,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=LOCOMOTION_JOINTS)
            },
            noise=Unoise(n_min=-1.5, n_max=1.5),
        )
        previous_action = ObsTerm(func=base_mdp.last_action)
        image_age = ObsTerm(func=mdp.image_age)

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class PrivilegedCfg(ObsGroup):
        base_lin_vel = ObsTerm(func=base_mdp.base_lin_vel)
        contacts = ObsTerm(
            func=mdp.contact_forces_flat,
            params={
                "sensor_cfg": SceneEntityCfg(
                    "contact_forces",
                    body_names=["pelvis", ".*_ankle_roll_link"],
                )
            },
        )
        terrain_height = ObsTerm(
            func=base_mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    # Image terms update before the policy reads its image-age feature.
    rgb: RgbCfg = RgbCfg()
    depth: DepthCfg = DepthCfg()
    policy: PolicyCfg = PolicyCfg()
    privileged: PrivilegedCfg = PrivilegedCfg()


@configclass
class X2RewardsCfg(RewardsCfg):
    termination_penalty = RewTerm(func=base_mdp.is_terminated, weight=-200.0)
    track_lin_vel_xy_exp = RewTerm(
        func=base_mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": 0.5},
    )
    track_ang_vel_z_exp = RewTerm(
        func=base_mdp.track_ang_vel_z_world_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": 0.5},
    )
    feet_air_time = RewTerm(
        func=base_mdp.feet_air_time_positive_biped,
        weight=0.25,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg(
                "contact_forces", body_names=".*_ankle_roll_link"
            ),
            "threshold": 0.4,
        },
    )
    feet_slide = RewTerm(
        func=base_mdp.feet_slide,
        weight=-0.25,
        params={
            "sensor_cfg": SceneEntityCfg(
                "contact_forces", body_names=".*_ankle_roll_link"
            ),
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_ankle_roll_link"),
        },
    )
    joint_deviation_arms = RewTerm(
        func=base_mdp.joint_deviation_l1,
        weight=-0.15,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    ".*_shoulder_.*_joint",
                    ".*_elbow_joint",
                    ".*_wrist_.*_joint",
                ],
            )
        },
    )


@configclass
class X2RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    scene: X2SceneCfg = X2SceneCfg(num_envs=256, env_spacing=2.5)
    actions: X2ActionsCfg = X2ActionsCfg()
    observations: X2ObservationsCfg = X2ObservationsCfg()
    rewards: X2RewardsCfg = X2RewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        stage = os.environ.get("OHMC_CURRICULUM_STAGE", CURRICULUM_ORDER[-1])
        if stage not in CURRICULUM_ORDER:
            raise ValueError(f"unknown OHMC_CURRICULUM_STAGE: {stage}")

        self.scene.robot = build_x2_cfg().replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/pelvis"
        terrain = _terrain_for_stage(stage)
        if terrain is None:
            self.scene.terrain.terrain_type = "plane"
            self.scene.terrain.terrain_generator = None
            self.curriculum.terrain_levels = None
        else:
            self.scene.terrain.terrain_type = "generator"
            self.scene.terrain.terrain_generator = terrain

        self.events.add_base_mass.params["asset_cfg"].body_names = "pelvis"
        if self.events.base_com is not None:
            self.events.base_com.params["asset_cfg"].body_names = "pelvis"
        self.events.base_external_force_torque.params["asset_cfg"].body_names = "pelvis"
        self.events.reset_robot_joints.params["position_range"] = (0.9, 1.1)
        self.terminations.base_contact.params["sensor_cfg"].body_names = [
            "pelvis",
            "torso_link",
        ]

        self.commands.base_velocity.rel_standing_envs = (
            1.0 if stage == "00_stand" else 0.05
        )
        if stage == "00_stand":
            self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
            self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
            self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
        elif stage == "01_flat":
            self.commands.base_velocity.ranges.lin_vel_x = (-0.6, 0.8)
            self.commands.base_velocity.ranges.lin_vel_y = (-0.3, 0.3)
            self.commands.base_velocity.ranges.ang_vel_z = (-0.6, 0.6)
        else:
            self.commands.base_velocity.ranges.lin_vel_x = (-0.8, 1.0)
            self.commands.base_velocity.ranges.lin_vel_y = (-0.5, 0.5)
            self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)

        self.rewards.undesired_contacts = None
        self.rewards.flat_orientation_l2.weight = -1.0
        self.rewards.dof_torques_l2.weight = -1.0e-5
        self.rewards.action_rate_l2.weight = -0.005
        self.rewards.dof_acc_l2.weight = -1.25e-7


@configclass
class X2RoughEnvCfg_PLAY(X2RoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 1
        self.scene.env_spacing = 2.5
        self.episode_length_s = 60.0
        self.observations.rgb.enable_corruption = False
        self.observations.depth.enable_corruption = False
        self.observations.policy.enable_corruption = False
        self.observations.rgb.image.params["randomize"] = False
        self.observations.depth.image.params["randomize"] = False
        self.events.base_external_force_torque = None
        self.events.push_robot = None
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 2
            self.scene.terrain.terrain_generator.num_cols = 2
            self.scene.terrain.terrain_generator.curriculum = False


__all__ = ["CURRICULUM_ORDER", "X2RoughEnvCfg", "X2RoughEnvCfg_PLAY"]
