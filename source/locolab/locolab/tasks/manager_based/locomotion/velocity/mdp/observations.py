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


# =====  contact  =====
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
