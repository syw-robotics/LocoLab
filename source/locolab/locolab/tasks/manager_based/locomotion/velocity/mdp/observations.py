# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import math as math_utils

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


# =====  Gait  =====
def gait_phase(env: ManagerBasedRLEnv, period: float) -> torch.Tensor:
    if not hasattr(env, "episode_length_buf"):
        env.episode_length_buf = torch.zeros(env.num_envs, device=env.device, dtype=torch.long)

    global_phase = (env.episode_length_buf * env.step_dt) % period / period

    phase = torch.zeros(env.num_envs, 2, device=env.device)
    phase[:, 0] = torch.sin(global_phase * torch.pi * 2.0)
    phase[:, 1] = torch.cos(global_phase * torch.pi * 2.0)
    return phase


def amp_body_state(
    env,
    amp_body_cfg: SceneEntityCfg,
    amp_anchor_cfg: SceneEntityCfg,
) -> torch.Tensor:
    asset = env.scene[amp_body_cfg.name]

    body_pos_w = asset.data.body_pos_w[:, amp_body_cfg.body_ids, :]
    body_quat_w = asset.data.body_quat_w[:, amp_body_cfg.body_ids, :]
    body_lin_vel_w = asset.data.body_lin_vel_w[:, amp_body_cfg.body_ids, :]
    body_ang_vel_w = asset.data.body_ang_vel_w[:, amp_body_cfg.body_ids, :]

    anchor_id = amp_anchor_cfg.body_ids[0]
    num_envs, num_bodies = body_pos_w.shape[:2]

    anchor_pos_w = asset.data.body_pos_w[:, anchor_id, :].unsqueeze(1).expand(-1, num_bodies, -1)
    anchor_quat_w = asset.data.body_quat_w[:, anchor_id, :].unsqueeze(1).expand(-1, num_bodies, -1)

    body_pos_b, body_quat_b = math_utils.subtract_frame_transforms(
        anchor_pos_w.reshape(-1, 3),
        anchor_quat_w.reshape(-1, 4),
        body_pos_w.reshape(-1, 3),
        body_quat_w.reshape(-1, 4),
    )

    body_pos_b = body_pos_b.reshape(num_envs, num_bodies, 3)
    body_quat_b = body_quat_b.reshape(num_envs, num_bodies, 4)

    body_ori_b = math_utils.matrix_from_quat(body_quat_b)[..., :, :2].reshape(num_envs, num_bodies, 6)
    body_lin_vel_b = math_utils.quat_apply_inverse(body_quat_w, body_lin_vel_w)
    body_ang_vel_b = math_utils.quat_apply_inverse(body_quat_w, body_ang_vel_w)

    return torch.cat(
        [
            body_pos_b.reshape(num_envs, -1),
            body_ori_b.reshape(num_envs, -1),
            body_lin_vel_b.reshape(num_envs, -1),
            body_ang_vel_b.reshape(num_envs, -1),
        ],
        dim=-1,
    )


def amp_reference_body_state(
    env,
    motion_dir: str,
    amp_body_names: list[str],
    amp_anchor_name: str,
    motion_body_names: list[str],
) -> torch.Tensor:
    if not hasattr(env, "_amp_motion_reference"):
        from locolab.motion_reference import AmpMotionReference

        env._amp_motion_reference = AmpMotionReference(
            motion_dir=motion_dir,
            amp_body_names=amp_body_names,
            amp_anchor_name=amp_anchor_name,
            motion_body_names=motion_body_names,
            device=env.device,
        )

    return env._amp_motion_reference.get_state(env)