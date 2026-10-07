# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from __future__ import annotations

from typing import TYPE_CHECKING, Literal

import torch

from isaaclab.managers import ManagerTermBase, ManagerTermBaseCfg, SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.assets import Articulation
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.sensors import ContactSensor


# =====  body  =====
def body_mass(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """The mass of the specified bodies."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    mass_tensor = asset.root_physx_view.get_masses().to(device=env.device)
    body_mass = mass_tensor[:, asset_cfg.body_ids]
    return body_mass


def body_inertia(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """The inertia of the specified bodies."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    inertia_tensor = asset.root_physx_view.get_inertias().to(device=env.device)
    body_inertia = inertia_tensor[:, asset_cfg.body_ids]
    return body_inertia.view(body_inertia.shape[0], -1)


# =====  feet  =====
def feet_contact_forces(
    env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_forces", body_names=".*_foot")
) -> torch.Tensor:
    """The contact forces of the specified bodies."""
    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    feet_contact_forces = contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, :]
    # Flatten to (num_envs, num_feet * 3) for concatenation with other observation terms
    return feet_contact_forces.view(feet_contact_forces.shape[0], -1)


def feet_contact_flag(
    env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_forces", body_names=".*_foot")
) -> torch.Tensor:
    """The contact forces of the specified bodies."""
    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    feet_contact = contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, :]
    feet_contact_flag = torch.any(feet_contact > 1.0, dim=2).float() - 0.5
    return feet_contact_flag


def feet_height(env: ManagerBasedRLEnv, feet_names: list[str]) -> torch.Tensor:
    """Height of each foot relative to the ground.

    Returns:
        Tensor of shape (num_envs, 4) containing the average height of each foot above ground.
    """
    # Stack all sensor data at once
    pos_z = torch.stack([env.scene.sensors[f"{name}_height_scanner"].data.pos_w[:, 2] for name in feet_names], dim=1)
    ray_hits_z = torch.stack(
        [env.scene.sensors[f"{name}_height_scanner"].data.ray_hits_w[..., 2] for name in feet_names], dim=1
    )

    # Compute heights: (num_envs, 4, 1) - (num_envs, 4, num_rays) = (num_envs, 4, num_rays)
    feet_heights = pos_z.unsqueeze(-1) - ray_hits_z

    # Average over rays: (num_envs, 4)
    return feet_heights.mean(dim=-1)


# =====  joint  =====
def joint_acc(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """The acceleration of the specified joints."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    joint_acc = asset.data.joint_acc[:, asset_cfg.joint_ids]
    return joint_acc


# =====  gait  =====
def gait_phase(env: ManagerBasedRLEnv, period: float) -> torch.Tensor:
    if not hasattr(env, "episode_length_buf"):
        env.episode_length_buf = torch.zeros(env.num_envs, device=env.device, dtype=torch.long)

    global_phase = (env.episode_length_buf * env.step_dt) % period / period

    phase = torch.zeros(env.num_envs, 2, device=env.device)
    phase[:, 0] = torch.sin(global_phase * torch.pi * 2.0)
    phase[:, 1] = torch.cos(global_phase * torch.pi * 2.0)
    return phase


# =====  camera  =====
class delayed_visualizable_image(ManagerTermBase):
    """Return delayed frames from a noisy camera history buffer.

    The camera must already store ``data_type`` as ``(N, history, H, W, C)``. The
    returned tensor is ``(N, num_output_frames, H, W)``, oldest frame first. The
    selected history indices are written to ``sensor._delayed_frame_indices_by_data_type``
    so a later mask term can read the same frames.
    """

    def __init__(self, cfg: ManagerTermBaseCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._num_envs = env.num_envs
        self._device = env.device
        self.sensor_cfg = cfg.params.get("sensor_cfg", SceneEntityCfg("camera"))
        self.data_type = cfg.params["data_type"]
        if "history" not in self.data_type:
            raise ValueError("`data_type` must refer to a history output.")

        self.sensor = env.scene.sensors[self.sensor_cfg.name]
        self.delayed_frame_ranges = cfg.params.get("delayed_frame_ranges", (0, 0))
        self.delayed_frame_distribution = cfg.params.get("delayed_frame_distribution", "uniform")
        self.history_skip_frames = max(cfg.params.get("history_skip_frames", 1), 1)
        self.num_output_frames = max(cfg.params.get("num_output_frames", 1), 1)
        self.sensor_history_length = self.sensor.data.output[self.data_type].shape[1]
        self._num_delayed_frames = torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
        self.frame_offset = torch.arange(
            (self.num_output_frames - 1) * self.history_skip_frames,
            -1,
            -self.history_skip_frames,
            device=env.device,
        )
        frames_needed = (self.num_output_frames - 1) * self.history_skip_frames + 1
        if frames_needed + self.delayed_frame_ranges[1] > self.sensor_history_length:
            raise ValueError(
                "Camera history is too short for the requested output frames and delay: "
                f"need {frames_needed + self.delayed_frame_ranges[1]}, have {self.sensor_history_length}."
            )

    def reset(self, env_ids: torch.Tensor | None = None) -> None:
        if self.delayed_frame_distribution != "uniform":
            raise NotImplementedError("Only uniform delayed-frame sampling is supported.")
        ids = (
            torch.arange(self._num_envs, device=self._device)
            if env_ids is None
            else torch.as_tensor(env_ids, device=self._device, dtype=torch.long)
        )
        min_delay, max_delay = self.delayed_frame_ranges
        self._num_delayed_frames[ids] = torch.randint(
            min_delay,
            max_delay + 1,
            (len(ids),),
            device=self._device,
        )

    def __call__(
        self,
        env: ManagerBasedRLEnv,
        data_type: str,
        sensor_cfg: SceneEntityCfg = SceneEntityCfg("camera"),
        history_skip_frames: int = 1,
        num_output_frames: int = 1,
        delayed_frame_ranges: tuple[int, int] = (0, 0),
        delayed_frame_distribution: Literal["uniform"] = "uniform",
    ) -> torch.Tensor:
        del env, data_type, sensor_cfg, history_skip_frames, num_output_frames
        del delayed_frame_ranges, delayed_frame_distribution

        images = self.sensor.data.output[self.data_type].squeeze(-1)
        frame_indices = (
            self.sensor_history_length
            - self.frame_offset.unsqueeze(0)
            - self._num_delayed_frames.unsqueeze(1)
            - 1
        )
        if (frame_indices < 0).any():
            raise RuntimeError("Delayed camera frame index is out of bounds.")

        indices_by_type = getattr(self.sensor, "_delayed_frame_indices_by_data_type", None)
        if indices_by_type is None:
            indices_by_type = {}
            self.sensor._delayed_frame_indices_by_data_type = indices_by_type
        indices_by_type[self.data_type] = frame_indices

        batch_indices = torch.arange(images.shape[0], device=images.device).unsqueeze(1)
        return images[batch_indices, frame_indices]
