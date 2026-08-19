"""Register the OHMC X2 training and play environments."""

from __future__ import annotations

import gymnasium as gym

from . import agents

gym.register(
    id="OHMC-X2-RGBD-Rough-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rough_env_cfg:X2RoughEnvCfg",
        "rsl_rl_cfg_entry_point": (
            f"{agents.__name__}.rsl_rl_ppo_cfg:X2RgbdRoughPPORunnerCfg"
        ),
    },
)

gym.register(
    id="OHMC-X2-RGBD-Rough-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rough_env_cfg:X2RoughEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": (
            f"{agents.__name__}.rsl_rl_ppo_cfg:X2RgbdRoughPPORunnerCfg"
        ),
    },
)
