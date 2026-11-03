# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Tensor utilities for motion resampling, velocities, and body alignment."""

from __future__ import annotations

import numpy as np
import torch

from isaaclab.utils.math import axis_angle_from_quat, quat_apply, quat_inv, quat_mul, yaw_quat


def finite_difference_ang_vel(quat_xyzw: torch.Tensor, dt: float) -> torch.Tensor:
    """Central-difference angular velocity of an xyzw quaternion sequence.

    ``quat_xyzw`` is ``(T, ..., 4)``. The result is ``(T, ..., 3)`` in the world frame.
    """
    if quat_xyzw.shape[0] < 3:
        raise ValueError(f"Need at least 3 frames to difference orientation, got {quat_xyzw.shape[0]}.")
    q_prev = quat_xyzw[:-2]
    q_next = quat_xyzw[2:]
    # q_next * inverse(q_prev) expresses the rotation over two steps in the world frame.
    omega = axis_angle_from_quat(quat_mul(q_next, quat_inv(q_prev))) / (2.0 * dt)
    # Reuse the nearest interior estimate at each endpoint to preserve the frame count.
    return torch.cat((omega[:1], omega, omega[-1:]), dim=0)


def finite_difference_vel(values: torch.Tensor, dt: float) -> torch.Tensor:
    """Differentiate ``(T, ...)`` values with frame spacing ``dt`` seconds.

    Use central differences inside the sequence and one-sided differences at
    the endpoints, preserving the input shape.
    """
    return torch.gradient(values, spacing=dt, dim=0)[0]


def resample_motion(
    root_pos: torch.Tensor,
    root_quat_xyzw: torch.Tensor,
    joint_pos: torch.Tensor,
    input_dt: float,
    output_dt: float,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Resample positions linearly and xyzw orientations with spherical interpolation.

    Inputs are CPU tensors of shape ``(T, 3)``, ``(T, 4)``, and ``(T, J)``.
    Frame spacings are in seconds. Outputs start at the first input frame and
    cover the clip at ``output_dt`` intervals, without extending its duration.
    """
    num_input = root_pos.shape[0]
    duration = (num_input - 1) * input_dt
    if abs(input_dt - output_dt) <= 1.0e-8:
        return root_pos, _normalize_quat(root_quat_xyzw), joint_pos

    # The tolerance retains an endpoint that would otherwise be lost to rounding.
    num_output = int(np.floor(duration / output_dt + 1.0e-6)) + 1
    times = torch.arange(num_output, dtype=torch.float32) * output_dt
    phase = times / duration
    # Map each output time to two adjacent input frames and their interpolation weight.
    index_0 = torch.floor(phase * (num_input - 1)).long()
    index_1 = torch.minimum(index_0 + 1, torch.tensor(num_input - 1))
    blend = (phase * (num_input - 1) - index_0).unsqueeze(-1)
    root_pos = _lerp(root_pos[index_0], root_pos[index_1], blend)
    joint_pos = _lerp(joint_pos[index_0], joint_pos[index_1], blend)
    root_quat = _slerp(root_quat_xyzw[index_0], root_quat_xyzw[index_1], blend.squeeze(-1))
    return root_pos, root_quat, joint_pos


def _lerp(a: torch.Tensor, b: torch.Tensor, blend: torch.Tensor) -> torch.Tensor:
    return a * (1.0 - blend) + b * blend


def _slerp(q0: torch.Tensor, q1: torch.Tensor, blend: torch.Tensor) -> torch.Tensor:
    q0 = _normalize_quat(q0)
    q1 = _normalize_quat(q1)
    dot = (q0 * q1).sum(dim=-1, keepdim=True)
    # q and -q represent the same orientation; choose the sign giving the shortest arc.
    q1 = torch.where(dot < 0.0, -q1, q1)
    dot = (q0 * q1).sum(dim=-1, keepdim=True).clamp(-1.0, 1.0)
    omega = torch.acos(dot)
    sin_omega = torch.sin(omega)
    weight = blend.unsqueeze(-1)
    # Nearly identical orientations use a linear blend to avoid division by a tiny sine.
    close = sin_omega.abs() < 1.0e-4
    coeff0 = torch.sin((1.0 - weight) * omega) / sin_omega.clamp(min=1.0e-8)
    coeff1 = torch.sin(weight * omega) / sin_omega.clamp(min=1.0e-8)
    mixed = torch.where(close, (1.0 - weight) * q0 + weight * q1, coeff0 * q0 + coeff1 * q1)
    return _normalize_quat(mixed)


def _normalize_quat(quat: torch.Tensor) -> torch.Tensor:
    return quat / torch.linalg.norm(quat, dim=-1, keepdim=True).clamp(min=1.0e-8)


# Keep tensor math in one graph; simulator reads and persistent-buffer writes stay in the caller.
# fullgraph rejects graph breaks; it does not select an optimization or CUDA Graph mode.
@torch.compile(fullgraph=True)
def relative_body_poses(
    anchor_pos: torch.Tensor,
    anchor_quat: torch.Tensor,
    robot_anchor_pos: torch.Tensor,
    robot_anchor_quat: torch.Tensor,
    body_pos: torch.Tensor,
    body_quat: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Yaw-align reference bodies to the robot anchor, preserving reference height.

    Anchor poses have shape ``(N, 3)`` / ``(N, 4)`` and reference body poses
    ``(N, B, 3)`` / ``(N, B, 4)``. All poses use world coordinates and xyzw
    quaternions. Return aligned body positions and orientations with the same shapes.
    """
    # Follow the robot horizontally while retaining the reference anchor's height.
    origin = torch.cat((robot_anchor_pos[:, :2], anchor_pos[:, 2:]), dim=-1)
    # Apply only the heading difference, keeping the reference's roll and pitch.
    rotation = yaw_quat(quat_mul(robot_anchor_quat, quat_inv(anchor_quat)))
    rotation = rotation[:, None, :].expand_as(body_quat)
    position = origin[:, None, :] + quat_apply(rotation, body_pos - anchor_pos[:, None, :])
    return position, quat_mul(rotation, body_quat)

