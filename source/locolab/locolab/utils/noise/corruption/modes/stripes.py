# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Draw thin bands at a random angle, similar to stereo scan-line failures."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from ..cfg import DepthCorruptionPatternCfg


def apply_random_stripes(data: torch.Tensor, cfg: DepthCorruptionPatternCfg, active: torch.Tensor) -> None:
    active_indices = torch.nonzero(active, as_tuple=False).squeeze(-1)
    stripe_counts = torch.randint(
        cfg.stripe_count_range[0],
        cfg.stripe_count_range[1] + 1,
        (active_indices.numel(),),
        device=data.device,
    )
    height, width = data.shape[1:3]
    rows = torch.arange(height, device=data.device, dtype=data.dtype).view(1, height, 1)
    cols = torch.arange(width, device=data.device, dtype=data.dtype).view(1, 1, width)

    for stripe_index in range(cfg.stripe_count_range[1]):
        stripe_env_ids = active_indices[stripe_counts > stripe_index]
        if stripe_env_ids.numel() == 0:
            continue
        count = stripe_env_ids.numel()
        thickness = torch.randint(
            cfg.stripe_thickness_range[0],
            cfg.stripe_thickness_range[1] + 1,
            (count,),
            device=data.device,
        )
        angle_min, angle_max = cfg.stripe_angle_range_deg
        angles = torch.empty(count, device=data.device, dtype=data.dtype).uniform_(angle_min, angle_max)
        angles = torch.deg2rad(angles)
        normal_x = angles.cos()
        normal_y = angles.sin()
        projection_min = torch.minimum(normal_x * (width - 1), torch.zeros_like(normal_x))
        projection_min += torch.minimum(normal_y * (height - 1), torch.zeros_like(normal_y))
        projection_max = torch.maximum(normal_x * (width - 1), torch.zeros_like(normal_x))
        projection_max += torch.maximum(normal_y * (height - 1), torch.zeros_like(normal_y))
        centers = projection_min + torch.rand(count, device=data.device, dtype=data.dtype) * (
            projection_max - projection_min
        )
        projections = cols * normal_x.view(-1, 1, 1) + rows * normal_y.view(-1, 1, 1)
        stripe_mask = ((projections - centers.view(-1, 1, 1)).abs() <= thickness.view(-1, 1, 1) / 2.0).unsqueeze(-1)
        value_min, value_max = cfg.stripe_value_range
        replacement = torch.empty(count, device=data.device, dtype=data.dtype).uniform_(value_min, value_max)
        data[stripe_env_ids] = torch.where(
            stripe_mask,
            replacement.view(-1, 1, 1, 1),
            data[stripe_env_ids],
        )
