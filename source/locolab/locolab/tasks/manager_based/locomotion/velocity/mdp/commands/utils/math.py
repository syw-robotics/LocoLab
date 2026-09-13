# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Interpolation and workspace-expand math shared by arm EE commands."""

from __future__ import annotations

import torch

from isaaclab.utils.math import quat_apply, quat_from_euler_xyz, quat_mul, quat_unique


def cart_to_sphere(cart_coords: torch.Tensor) -> torch.Tensor:
    """Convert Cartesian coordinates to spherical ``(radius, pitch, yaw)``."""
    sphere_coords = torch.zeros_like(cart_coords)
    xy_len = torch.norm(cart_coords[..., :2], dim=-1)
    sphere_coords[..., 0] = torch.norm(cart_coords, dim=-1)
    sphere_coords[..., 1] = torch.atan2(cart_coords[..., 2], xy_len + 1e-8)
    sphere_coords[..., 2] = torch.atan2(cart_coords[..., 1], cart_coords[..., 0])
    return sphere_coords


def sphere_to_cart(sphere_coords: torch.Tensor) -> torch.Tensor:
    """Convert spherical ``(radius, pitch, yaw)`` to Cartesian coordinates."""
    radius = sphere_coords[..., 0]
    pitch = sphere_coords[..., 1]
    yaw = sphere_coords[..., 2]
    cos_pitch = torch.cos(pitch)
    cart_coords = torch.zeros_like(sphere_coords)
    cart_coords[..., 0] = radius * cos_pitch * torch.cos(yaw)
    cart_coords[..., 1] = radius * cos_pitch * torch.sin(yaw)
    cart_coords[..., 2] = radius * torch.sin(pitch)
    return cart_coords


def quat_slerp(q0: torch.Tensor, q1: torch.Tensor, blend: torch.Tensor) -> torch.Tensor:
    """Spherical linear interpolation of batched ``(w, x, y, z)`` quaternions."""
    if blend.ndim == 1:
        blend = blend.unsqueeze(-1)

    q1_adj = torch.where((torch.sum(q0 * q1, dim=-1, keepdim=True) < 0.0), -q1, q1)
    dot = torch.sum(q0 * q1_adj, dim=-1, keepdim=True).abs().clamp(-1.0, 1.0)

    q_nlerp = quat_unique((1.0 - blend) * q0 + blend * q1_adj)
    linear_mask = (dot > 0.9995).expand_as(q0)

    theta = torch.acos(dot)
    sin_theta = torch.sin(theta)
    w0 = torch.sin((1.0 - blend) * theta) / sin_theta
    w1 = torch.sin(blend * theta) / sin_theta
    q_slerp = quat_unique(w0 * q0 + w1 * q1_adj)
    return torch.where(linear_mask, q_nlerp, q_slerp)


def sample_uniform_range(
    value_range: tuple[float, float], num: int, device: torch.device
) -> torch.Tensor:
    """Sample ``num`` values uniformly from ``[low, high]``. Degenerate ranges return ``low``."""
    low, high = value_range
    if high > low:
        return torch.rand(num, device=device) * (high - low) + low
    return torch.full((num,), low, device=device, dtype=torch.float32)


def lerp_positions_sphere(
    start_pos: torch.Tensor,
    target_pos: torch.Tensor,
    blend: torch.Tensor,
    center: torch.Tensor,
) -> torch.Tensor:
    """Interpolate positions in spherical coordinates about ``center``."""
    while center.ndim < start_pos.ndim:
        center = center.unsqueeze(-2)
    start_rel = start_pos - center
    target_rel = target_pos - center
    start_s = cart_to_sphere(start_rel.reshape(-1, 3)).reshape(start_rel.shape)
    target_s = cart_to_sphere(target_rel.reshape(-1, 3)).reshape(target_rel.shape)
    curr_rel = sphere_to_cart(torch.lerp(start_s, target_s, blend).reshape(-1, 3)).reshape(start_rel.shape)
    return curr_rel + center


def lerp_positions(
    start_pos: torch.Tensor,
    target_pos: torch.Tensor,
    blend: torch.Tensor,
    *,
    center: torch.Tensor,
    use_sphere: torch.Tensor | None = None,
    only_sphere: bool = False,
    only_cartesian: bool = False,
) -> torch.Tensor:
    """Interpolate EE positions with optional per-sample spherical mode flags.

    ``only_sphere`` / ``only_cartesian`` are Python cfg flags. Mixed mode always
    evaluates both interpolants and selects with ``where`` so the hot path stays
    free of host-device synchronizations.
    """
    if only_sphere:
        return lerp_positions_sphere(start_pos, target_pos, blend, center)
    if only_cartesian:
        return torch.lerp(start_pos, target_pos, blend)

    cart_pos = torch.lerp(start_pos, target_pos, blend)
    sphere_pos = lerp_positions_sphere(start_pos, target_pos, blend, center)
    if use_sphere is None:
        return cart_pos
    mask = use_sphere
    while mask.ndim < start_pos.ndim:
        mask = mask.unsqueeze(-1)
    return torch.where(mask, sphere_pos, cart_pos)


def apply_base_assist_expand(
    pos: torch.Tensor,
    pitch: torch.Tensor,
    height: torch.Tensor,
    quat: torch.Tensor | None = None,
) -> tuple[torch.Tensor, torch.Tensor | None]:
    """Map offline bubble poses by a virtual base pitch and height residual.

    ``p ← R_y(θ) p + (0, 0, Δh)``. Command-frame y is unchanged. ``pitch`` /
    ``height`` are ``(N,)``. ``pos`` is ``(N, 3)`` or ``(N, T, 3)``.
    """
    zeros = torch.zeros_like(pitch)
    q_pitch = quat_from_euler_xyz(zeros, pitch, zeros)

    if pos.ndim == 3:
        num_points = pos.shape[1]
        q_flat = q_pitch.unsqueeze(1).expand(-1, num_points, -1).reshape(-1, 4)
        pos_out = quat_apply(q_flat, pos.reshape(-1, 3)).reshape(pos.shape)
        pos_out = pos_out.clone()
        pos_out[..., 2] = pos_out[..., 2] + height.unsqueeze(-1)
        if quat is None:
            return pos_out, None
        quat_out = quat_mul(q_flat, quat.reshape(-1, 4)).reshape(quat.shape)
        return pos_out, quat_out

    pos_out = quat_apply(q_pitch, pos)
    pos_out = pos_out.clone()
    pos_out[:, 2] = pos_out[:, 2] + height
    if quat is None:
        return pos_out, None
    return pos_out, quat_mul(q_pitch, quat)
