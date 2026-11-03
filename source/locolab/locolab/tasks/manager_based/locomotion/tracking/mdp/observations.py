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

from isaaclab.utils.math import matrix_from_quat, subtract_frame_transforms

from .commands import MotionCommand

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def motion_anchor_pos_b(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Reference anchor position in the robot anchor frame."""
    command = cast(MotionCommand, env.command_manager.get_term(command_name))
    pos, _ = subtract_frame_transforms(
        command.robot_anchor_pos_w,
        command.robot_anchor_quat_w,
        command.anchor_pos_w,
        command.anchor_quat_w,
    )
    return pos.view(env.num_envs, -1)


def motion_anchor_ori_b(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """First two columns of the reference anchor rotation in the robot anchor frame."""
    command = cast(MotionCommand, env.command_manager.get_term(command_name))
    _, ori = subtract_frame_transforms(
        command.robot_anchor_pos_w,
        command.robot_anchor_quat_w,
        command.anchor_pos_w,
        command.anchor_quat_w,
    )
    mat = matrix_from_quat(ori)
    return mat[..., :2].reshape(mat.shape[0], -1)


def robot_body_pos_b(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """Robot body positions in the robot anchor frame."""
    command = cast(MotionCommand, env.command_manager.get_term(command_name))
    pos_b, _ = subtract_frame_transforms(
        command.robot_anchor_pos_w[:, None, :].repeat(1, command.num_bodies, 1),
        command.robot_anchor_quat_w[:, None, :].repeat(1, command.num_bodies, 1),
        command.robot_body_pos_w,
        command.robot_body_quat_w,
    )
    return pos_b.view(env.num_envs, -1)


def robot_body_ori_b(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """First two columns of each robot body rotation in the robot anchor frame."""
    command = cast(MotionCommand, env.command_manager.get_term(command_name))
    _, ori_b = subtract_frame_transforms(
        command.robot_anchor_pos_w[:, None, :].repeat(1, command.num_bodies, 1),
        command.robot_anchor_quat_w[:, None, :].repeat(1, command.num_bodies, 1),
        command.robot_body_pos_w,
        command.robot_body_quat_w,
    )
    mat = matrix_from_quat(ori_b)
    return mat[..., :2].reshape(mat.shape[0], -1)
