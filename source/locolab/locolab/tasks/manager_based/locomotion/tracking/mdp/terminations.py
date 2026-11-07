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

from .commands import MotionCommand
from .rewards import _body_indexes

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def bad_anchor_pos_z_only(env: ManagerBasedRLEnv, command_name: str, threshold: float) -> torch.Tensor:
    command = cast(MotionCommand, env.command_manager.get_term(command_name))
    return torch.abs(command.anchor_pos_w[:, -1] - command.robot_anchor_pos_w[:, -1]) > threshold


def bad_anchor_ori(env: ManagerBasedRLEnv, command_name: str, threshold: float) -> torch.Tensor:
    command = cast(MotionCommand, env.command_manager.get_term(command_name))
    # For xyzw quaternions, projected unit gravity's z component is 2*(x*x+y*y)-1.
    motion_xy = command.anchor_quat_w[:, :2].square().sum(-1)
    robot_xy = command.robot_anchor_quat_w[:, :2].square().sum(-1)
    return (2 * (motion_xy - robot_xy)).abs() > threshold


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
