# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Draw elliptical specular blobs that punch holes in the depth image."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from .common import choice_values, masked_replace, row_col_grid

if TYPE_CHECKING:
    from ..cfg import DepthCorruptionPatternCfg


def apply_sparkles(data: torch.Tensor, cfg: DepthCorruptionPatternCfg, active: torch.Tensor) -> None:
    """Paint up to ``sparkle_count_range[1]`` blobs, one blob per pass."""
    n, height, width, _ = data.shape
    device, dtype = data.device, data.dtype
    count_min, count_max = cfg.sparkle_count_range
    counts = torch.randint(count_min, count_max + 1, (n,), device=device)
    center_y = torch.rand(count_max, n, device=device, dtype=dtype) * (height - 1)
    center_x = torch.rand(count_max, n, device=device, dtype=dtype) * (width - 1)
    radius = torch.empty(count_max, n, device=device, dtype=dtype).uniform_(*cfg.sparkle_radius_range)
    aspect_ratio = torch.empty(count_max, n, device=device, dtype=dtype).uniform_(*cfg.sparkle_aspect_ratio_range)
    radius_y = radius / aspect_ratio.sqrt()
    radius_x = radius * aspect_ratio.sqrt()
    values = choice_values(cfg.sparkle_value_choices, device, dtype)
    choice_ids = torch.randint(0, values.numel(), (count_max, n), device=device)
    rows, cols = row_col_grid(height, width, device, dtype)

    for index in range(count_max):
        sparkle = ((rows - center_y[index].view(n, 1, 1)) / radius_y[index].view(n, 1, 1)).square() + (
            (cols - center_x[index].view(n, 1, 1)) / radius_x[index].view(n, 1, 1)
        ).square() <= 1
        valid = active & (counts > index)
        masked_replace(data, valid.view(n, 1, 1, 1) & sparkle.unsqueeze(-1), values[choice_ids[index]].view(n, 1, 1, 1))
