# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Pose layout and yaw-aligned frame helpers shared by arm EE commands."""

from __future__ import annotations

from collections.abc import Sequence

import torch

from isaaclab.utils.math import quat_apply, quat_apply_inverse, quat_conjugate, quat_mul, subtract_frame_transforms

# Pose layout: [x, y, z, qw, qx, qy, qz].
POS_SLICE = slice(0, 3)
QUAT_SLICE = slice(3, 7)


def identity_pose_buffer(num_envs: int, device: torch.device) -> torch.Tensor:
    """Allocate ``(num_envs, 7)`` identity poses."""
    buffer = torch.zeros(num_envs, 7, device=device)
    buffer[:, 3] = 1.0
    return buffer


def resolve_env_ids(env_ids: Sequence[int], num_envs: int, device: torch.device) -> torch.Tensor:
    """Normalize CommandTerm env-id inputs to a 1D long tensor."""
    if isinstance(env_ids, slice):
        return torch.arange(num_envs, device=device)[env_ids]
    if isinstance(env_ids, torch.Tensor):
        return env_ids.to(device=device, dtype=torch.long)
    return torch.as_tensor(list(env_ids), device=device, dtype=torch.long)


def fill_yaw_anchor_center(
    anchor_center_w: torch.Tensor,
    root_pos_w: torch.Tensor,
    anchor_z_world: float,
    offset_b: torch.Tensor,
    yaw_quat_w: torch.Tensor,
    apply_offset: bool = True,
) -> None:
    """Write the yaw-aligned world anchor: follow XY, lock Z, optional yaw-frame offset."""
    anchor_center_w[:, 0] = root_pos_w[:, 0]
    anchor_center_w[:, 1] = root_pos_w[:, 1]
    anchor_center_w[:, 2] = anchor_z_world
    if apply_offset:
        offset = offset_b.expand(root_pos_w.shape[0], -1)
        anchor_center_w += quat_apply(yaw_quat_w, offset)


def yaw_cmd_b_to_world_pos(
    pos_b: torch.Tensor, center_w: torch.Tensor, yaw_quat_w: torch.Tensor
) -> torch.Tensor:
    """Map yaw-command-frame Cartesian positions to the environment world frame."""
    if pos_b.ndim == 2:
        return center_w + quat_apply(yaw_quat_w, pos_b)

    batch_size, num_points, _ = pos_b.shape
    center = center_w.unsqueeze(1).expand(batch_size, num_points, -1)
    yaw = yaw_quat_w.unsqueeze(1).expand(batch_size, num_points, 4).reshape(-1, 4)
    pos_flat = pos_b.reshape(-1, 3)
    return (center + quat_apply(yaw, pos_flat).reshape(batch_size, num_points, 3)).reshape(-1, 3)


def world_pos_to_yaw_cmd_b(
    pos_w: torch.Tensor, center_w: torch.Tensor, yaw_quat_w: torch.Tensor
) -> torch.Tensor:
    """Express world-frame positions in the yaw-command frame."""
    return quat_apply_inverse(yaw_quat_w, pos_w - center_w)


def yaw_cmd_b_to_world_quat(quat_b: torch.Tensor, yaw_quat_w: torch.Tensor) -> torch.Tensor:
    """Map yaw-command-frame orientation to the environment world frame."""
    return quat_mul(yaw_quat_w, quat_b)


def world_quat_to_yaw_cmd_b(quat_w: torch.Tensor, yaw_quat_w: torch.Tensor) -> torch.Tensor:
    """Express world-frame orientation in the yaw-command frame."""
    return quat_mul(quat_conjugate(yaw_quat_w), quat_w)


def sync_pose_command_b(
    pose_command_b: torch.Tensor,
    pose_command_w: torch.Tensor,
    root_pos_w: torch.Tensor,
    root_quat_w: torch.Tensor,
    track_orientation: bool,
) -> None:
    """Write the world-frame command into the current robot base frame for observations."""
    if not track_orientation:
        pose_command_b[:, POS_SLICE] = quat_apply_inverse(
            root_quat_w, pose_command_w[:, POS_SLICE] - root_pos_w
        )
        return
    pos_b, quat_b = subtract_frame_transforms(
        root_pos_w,
        root_quat_w,
        pose_command_w[:, POS_SLICE],
        pose_command_w[:, QUAT_SLICE],
    )
    pose_command_b[:, POS_SLICE] = pos_b
    pose_command_b[:, QUAT_SLICE] = quat_b
