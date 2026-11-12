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

from .common import masked_replace, row_col_grid

if TYPE_CHECKING:
    from ..cfg import DepthCorruptionPatternCfg


def apply_random_stripes(data: torch.Tensor, cfg: DepthCorruptionPatternCfg, active: torch.Tensor) -> None:
    """Paint up to ``stripe_count_range[1]`` bands.

    Parameters for every band are sampled together. Bands are applied one at a
    time so the mask stays one frame, not ``(count, N, H, W)``.
    """
    n, height, width, _ = data.shape
    device, dtype = data.device, data.dtype
    count_min, count_max = cfg.stripe_count_range
    thickness_min, thickness_max = cfg.stripe_thickness_range
    angle_min, angle_max = cfg.stripe_angle_range_deg
    value_min, value_max = cfg.stripe_value_range

    counts = torch.randint(count_min, count_max + 1, (n,), device=device)
    thickness = torch.randint(thickness_min, thickness_max + 1, (count_max, n), device=device)
    half_thickness = thickness.to(dtype=dtype) * 0.5
    angles = torch.empty(count_max, n, device=device, dtype=dtype).uniform_(angle_min, angle_max)
    angles = torch.deg2rad(angles)
    normal_x = angles.cos()
    normal_y = angles.sin()
    span_x = normal_x * (width - 1)
    span_y = normal_y * (height - 1)
    projection_min = span_x.clamp(max=0) + span_y.clamp(max=0)
    projection_max = span_x.clamp(min=0) + span_y.clamp(min=0)
    centers = projection_min + torch.rand_like(projection_min) * (projection_max - projection_min)
    values = torch.empty(count_max, n, device=device, dtype=dtype).uniform_(value_min, value_max)
    rows, cols = row_col_grid(height, width, device, dtype)

    for index in range(count_max):
        normal_x_i = normal_x[index].view(n, 1, 1)
        normal_y_i = normal_y[index].view(n, 1, 1)
        stripe = (cols * normal_x_i + rows * normal_y_i - centers[index].view(n, 1, 1)).abs() <= half_thickness[
            index
        ].view(n, 1, 1)
        valid = active & (counts > index)
        masked_replace(data, valid.view(n, 1, 1, 1) & stripe.unsqueeze(-1), values[index].view(n, 1, 1, 1))
