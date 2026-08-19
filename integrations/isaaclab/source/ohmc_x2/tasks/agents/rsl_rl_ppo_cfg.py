"""Asymmetric RGB-D PPO configuration for X2 rough-terrain locomotion."""

import os

from isaaclab.utils.configclass import configclass
from isaaclab_rl.rsl_rl import (
    RslRlCNNModelCfg,
    RslRlMLPModelCfg,
    RslRlOnPolicyRunnerCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class X2RgbdRoughPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    seed = 42
    device = "cuda:0"
    num_steps_per_env = 24
    max_iterations = 3000
    save_interval = 50
    experiment_name = "ohmc_x2_rgbd_rough"
    logger = "tensorboard"
    resume = False
    obs_groups = {  # noqa: RUF012 - Isaac Lab configclass field
        "actor": ["policy", "rgb", "depth"],
        "critic": ["policy", "privileged"],
    }
    actor = RslRlCNNModelCfg(
        obs_normalization=True,
        hidden_dims=[512, 256, 128],
        activation="elu",
        distribution_cfg=RslRlCNNModelCfg.GaussianDistributionCfg(init_std=0.8),
        # RSL-RL creates one CNN per image observation group. Global average
        # pooling makes each RGB/depth encoder emit exactly 32 features.
        cnn_cfg=RslRlCNNModelCfg.CNNCfg(
            output_channels=[8, 16, 32],
            kernel_size=[5, 3, 3],
            stride=[2, 2, 2],
            padding="zeros",
            activation="elu",
            global_pool="avg",
            flatten=True,
        ),
    )
    critic = RslRlMLPModelCfg(
        obs_normalization=True,
        hidden_dims=[512, 256, 128],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.01,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
        share_cnn_encoders=False,
    )

    def __post_init__(self):
        super().__post_init__()
        self.num_steps_per_env = int(
            os.environ.get("OHMC_NUM_STEPS_PER_ENV", self.num_steps_per_env)
        )
        self.save_interval = int(
            os.environ.get("OHMC_SAVE_INTERVAL", self.save_interval)
        )
