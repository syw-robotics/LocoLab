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

if TYPE_CHECKING:
    from ..cfg import DepthCorruptionPatternCfg


def apply_sparkles(data: torch.Tensor, cfg: DepthCorruptionPatternCfg, active: torch.Tensor) -> None:
    active_indices = torch.nonzero(active, as_tuple=False).squeeze(-1)
    sparkle_counts = torch.randint(
        cfg.sparkle_count_range[0],
        cfg.sparkle_count_range[1] + 1,
        (active_indices.numel(),),
        device=data.device,
    )
    height, width = data.shape[1:3]
    rows = torch.arange(height, device=data.device, dtype=data.dtype).view(1, height, 1)
    cols = torch.arange(width, device=data.device, dtype=data.dtype).view(1, 1, width)
    values = torch.as_tensor(cfg.sparkle_value_choices, device=data.device, dtype=data.dtype)

    for sparkle_index in range(cfg.sparkle_count_range[1]):
        sparkle_env_ids = active_indices[sparkle_counts > sparkle_index]
        if sparkle_env_ids.numel() == 0:
            continue
        count = sparkle_env_ids.numel()
        center_y = torch.rand(count, device=data.device, dtype=data.dtype) * (height - 1)
        center_x = torch.rand(count, device=data.device, dtype=data.dtype) * (width - 1)
        radius = torch.empty(count, device=data.device, dtype=data.dtype).uniform_(*cfg.sparkle_radius_range)
        aspect_ratio = torch.empty(count, device=data.device, dtype=data.dtype).uniform_(
            *cfg.sparkle_aspect_ratio_range
        )
        radius_y = radius / aspect_ratio.sqrt()
        radius_x = radius * aspect_ratio.sqrt()
        sparkle_mask = (
            ((rows - center_y.view(-1, 1, 1)) / radius_y.view(-1, 1, 1)).square()
            + ((cols - center_x.view(-1, 1, 1)) / radius_x.view(-1, 1, 1)).square()
            <= 1.0
        ).unsqueeze(-1)
        choice_ids = torch.randint(0, values.numel(), (count,), device=data.device)
        data[sparkle_env_ids] = torch.where(
            sparkle_mask,
            values[choice_ids].view(-1, 1, 1, 1),
            data[sparkle_env_ids],
        )
