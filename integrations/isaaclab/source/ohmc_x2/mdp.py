"""X2-specific observation terms and camera-domain randomization."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from isaaclab.managers import ManagerTermBase, SceneEntityCfg
from torch.nn import functional

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.sensors import Camera, ContactSensor


class RandomizedCameraObservation(ManagerTermBase):
    """Return NCHW RGB/depth with appearance, geometry and delay randomization."""

    def __init__(self, cfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        sensor_cfg: SceneEntityCfg = cfg.params["sensor_cfg"]
        self.sensor: Camera = env.scene.sensors[sensor_cfg.name]
        self.data_type = str(cfg.params["data_type"])
        self.max_latency_steps = int(cfg.params.get("max_latency_steps", 2))
        self._history: list[torch.Tensor] = []

    def reset(self, env_ids=None) -> None:
        self._history.clear()

    def __call__(
        self,
        env: ManagerBasedRLEnv,
        sensor_cfg: SceneEntityCfg,
        data_type: str,
        max_latency_steps: int = 2,
        randomize: bool = True,
    ) -> torch.Tensor:
        raw = self.sensor.data.output[self.data_type]
        image = raw.torch if hasattr(raw, "torch") else raw
        image = image.clone()
        if self.data_type == "rgb" and randomize:
            image = self._randomize_rgb(image)
        elif self.data_type != "rgb" and randomize:
            image = self._randomize_depth(image)
        elif self.data_type == "rgb":
            image = image.float().div_(255.0).clamp_(0.0, 1.0)
        else:
            image = (
                torch.nan_to_num(image.float(), nan=0.0, posinf=0.0, neginf=0.0)
                .clamp_(0.0, 4.0)
                .div_(4.0)
            )
        image = image.permute(0, 3, 1, 2).contiguous()

        self._history.append(image)
        capacity = self.max_latency_steps + 1
        if len(self._history) > capacity:
            self._history.pop(0)
        while len(self._history) < capacity:
            self._history.insert(0, image)
        history = torch.stack(self._history, dim=0)
        delays = torch.randint(
            0,
            capacity,
            (env.num_envs,),
            device=image.device,
        )
        env_ids = torch.arange(env.num_envs, device=image.device)
        return history[-1 - delays, env_ids].clone()

    @staticmethod
    def _randomize_rgb(image: torch.Tensor) -> torch.Tensor:
        image = image.float() / 255.0
        count, height, width, _ = image.shape
        gain = torch.empty((count, 1, 1, 3), device=image.device).uniform_(0.70, 1.30)
        exposure = torch.empty((count, 1, 1, 1), device=image.device).uniform_(
            -0.12, 0.12
        )
        texture = torch.empty((count, 1, 6, 8), device=image.device).uniform_(
            0.85, 1.15
        )
        texture = functional.interpolate(
            texture,
            size=(height, width),
            mode="bilinear",
            align_corners=False,
        ).permute(0, 2, 3, 1)
        image = image * gain * texture + exposure
        image += torch.randn_like(image) * 0.015
        return image.clamp_(0.0, 1.0)

    @staticmethod
    def _randomize_depth(image: torch.Tensor) -> torch.Tensor:
        image = torch.nan_to_num(image.float(), nan=0.0, posinf=0.0, neginf=0.0)
        image = image.clamp_(0.0, 4.0) / 4.0
        count = image.shape[0]
        image += torch.randn_like(image) * (0.005 + 0.015 * image)
        image.masked_fill_(torch.rand_like(image) < 0.02, 0.0)

        theta = torch.zeros((count, 2, 3), device=image.device, dtype=image.dtype)
        theta[:, 0, 0] = 1.0
        theta[:, 1, 1] = 1.0
        theta[:, 0, 2].uniform_(-2.0 / image.shape[2], 2.0 / image.shape[2])
        theta[:, 1, 2].uniform_(-2.0 / image.shape[1], 2.0 / image.shape[1])
        nchw = image.permute(0, 3, 1, 2)
        grid = functional.affine_grid(theta, nchw.shape, align_corners=False)
        shifted = functional.grid_sample(
            nchw,
            grid,
            mode="bilinear",
            padding_mode="zeros",
            align_corners=False,
        )
        return shifted.permute(0, 2, 3, 1).clamp_(0.0, 1.0)


def image_age(
    env: ManagerBasedRLEnv,
    sensor_period_s: float = 1.0 / 15.0,
    stale_after_s: float = 0.250,
) -> torch.Tensor:
    """Return a normalized age proxy for the asynchronously updated camera."""

    elapsed = env.episode_length_buf.float() * env.step_dt
    age = torch.remainder(elapsed, sensor_period_s)
    return (age / stale_after_s).clamp_(0.0, 1.0).unsqueeze(-1)


def contact_forces_flat(
    env: ManagerBasedRLEnv,
    sensor_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Flatten privileged body contact forces for the critic only."""

    sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    forces = sensor.data.net_forces_w.torch[:, sensor_cfg.body_ids, :]
    return forces.reshape(env.num_envs, -1)


__all__ = ["RandomizedCameraObservation", "contact_forces_flat", "image_age"]
