# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from __future__ import annotations

from typing import TYPE_CHECKING, cast

import torch

from isaaclab.utils.math import quat_error_magnitude

from .commands import MotionCommand

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.sensors import ContactSensor


def _body_indexes(command: MotionCommand, body_names: tuple[str, ...] | list[str] | None) -> list[int]:
    if body_names is None:
        return list(range(len(command.cfg.body_names)))
    return [list(command.cfg.body_names).index(name) for name in body_names]


def motion_global_anchor_position_error_exp(env: ManagerBasedRLEnv, command_name: str, std: float) -> torch.Tensor:
    command = cast(MotionCommand, env.command_manager.get_term(command_name))
    error = torch.sum(torch.square(command.anchor_pos_w - command.robot_anchor_pos_w), dim=-1)
    return torch.exp(-error / std**2)


def motion_global_anchor_orientation_error_exp(env: ManagerBasedRLEnv, command_name: str, std: float) -> torch.Tensor:
    command = cast(MotionCommand, env.command_manager.get_term(command_name))
    error = quat_error_magnitude(command.anchor_quat_w, command.robot_anchor_quat_w) ** 2
    return torch.exp(-error / std**2)


def motion_relative_body_position_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    body_names: tuple[str, ...] | list[str] | None = None,
) -> torch.Tensor:
    command = cast(MotionCommand, env.command_manager.get_term(command_name))
    indexes = _body_indexes(command, body_names)
    error = torch.sum(
        torch.square(command.body_pos_relative_w[:, indexes] - command.robot_body_pos_w[:, indexes]),
        dim=-1,
    )
    return torch.exp(-error.mean(-1) / std**2)


def motion_relative_body_orientation_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    body_names: tuple[str, ...] | list[str] | None = None,
) -> torch.Tensor:
    command = cast(MotionCommand, env.command_manager.get_term(command_name))
    indexes = _body_indexes(command, body_names)
    error = quat_error_magnitude(command.body_quat_relative_w[:, indexes], command.robot_body_quat_w[:, indexes]) ** 2
    return torch.exp(-error.mean(-1) / std**2)


def motion_global_body_linear_velocity_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    body_names: tuple[str, ...] | list[str] | None = None,
) -> torch.Tensor:
    command = cast(MotionCommand, env.command_manager.get_term(command_name))
    indexes = _body_indexes(command, body_names)
    error = torch.sum(
        torch.square(command.body_lin_vel_w[:, indexes] - command.robot_body_lin_vel_w[:, indexes]),
        dim=-1,
    )
    return torch.exp(-error.mean(-1) / std**2)


def motion_global_body_angular_velocity_error_exp(
    env: ManagerBasedRLEnv,
    command_name: str,
    std: float,
    body_names: tuple[str, ...] | list[str] | None = None,
) -> torch.Tensor:
    command = cast(MotionCommand, env.command_manager.get_term(command_name))
    indexes = _body_indexes(command, body_names)
    error = torch.sum(
        torch.square(command.body_ang_vel_w[:, indexes] - command.robot_body_ang_vel_w[:, indexes]),
        dim=-1,
    )
    return torch.exp(-error.mean(-1) / std**2)


def self_collision_cost(
    env: ManagerBasedRLEnv,
    sensor_names: list[str],
    force_threshold: float = 10.0,
) -> torch.Tensor:
    """Count filtered self-contact substeps whose force exceeds ``force_threshold``."""
    cost = torch.zeros(env.num_envs, device=env.device)
    for name in sensor_names:
        sensor: ContactSensor = env.scene[name]
        history = sensor.data.normal_force_matrix_w_history
        if history is None:
            continue
        force_mag = torch.linalg.norm(history.torch, dim=-1)
        hit = (force_mag > force_threshold).flatten(start_dim=2).any(dim=-1)
        cost += hit.sum(dim=-1).float()
    return cost
