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

from isaaclab.utils.math import quat_apply_inverse

from .commands import MotionCommand
from .rewards import _body_indexes

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


_GRAVITY_W = (0.0, 0.0, -1.0)


def bad_anchor_pos_z_only(env: ManagerBasedRLEnv, command_name: str, threshold: float) -> torch.Tensor:
    command = cast(MotionCommand, env.command_manager.get_term(command_name))
    return torch.abs(command.anchor_pos_w[:, -1] - command.robot_anchor_pos_w[:, -1]) > threshold


def bad_anchor_ori(env: ManagerBasedRLEnv, command_name: str, threshold: float) -> torch.Tensor:
    command = cast(MotionCommand, env.command_manager.get_term(command_name))
    gravity_w = command.anchor_quat_w.new_tensor(_GRAVITY_W)
    motion_projected_gravity_b = quat_apply_inverse(command.anchor_quat_w, gravity_w)
    robot_projected_gravity_b = quat_apply_inverse(command.robot_anchor_quat_w, gravity_w)
    return (motion_projected_gravity_b[:, 2] - robot_projected_gravity_b[:, 2]).abs() > threshold


def bad_motion_body_pos_z_only(
    env: ManagerBasedRLEnv,
    command_name: str,
    threshold: float,
    body_names: tuple[str, ...] | list[str] | None = None,
) -> torch.Tensor:
    command = cast(MotionCommand, env.command_manager.get_term(command_name))
    indexes = _body_indexes(command, body_names)
    error = torch.abs(command.body_pos_relative_w[:, indexes, -1] - command.robot_body_pos_w[:, indexes, -1])
    return torch.any(error > threshold, dim=-1)
