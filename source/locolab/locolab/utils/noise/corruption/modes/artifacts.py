# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Paint one or more rectangular stereo-hole patches."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from .common import choice_values, masked_replace, row_col_grid

if TYPE_CHECKING:
    from ..cfg import DepthCorruptionPatternCfg


def apply_strong_artifact(data: torch.Tensor, cfg: DepthCorruptionPatternCfg, active: torch.Tensor) -> None:
    """Paint up to ``strong_artifacts_max_blocks`` rectangles, one block per pass."""
    if cfg.strong_artifacts_prob <= 0.0 or cfg.strong_artifacts_max_blocks == 0:
        return

    n, height, width, _ = data.shape
    raw_expected_blocks = cfg.strong_artifacts_prob * height * width
    if raw_expected_blocks <= 0.0:
        return

    # Avoid saturating every frame at max_blocks on low-resolution images. The
    # raw per-pixel rate is still used, but its Poisson mean is capped at the
    # midpoint of the configured count range so 1..max_blocks remain observable.
    count_mean = min(raw_expected_blocks, (cfg.strong_artifacts_max_blocks + 1) / 2.0)
    block_counts = torch.poisson(torch.full((n,), count_mean, device=data.device))
    block_counts = block_counts.to(dtype=torch.long).clamp_(min=1, max=cfg.strong_artifacts_max_blocks)

    max_blocks = cfg.strong_artifacts_max_blocks
    heights = _sample_sizes(max_blocks * n, cfg.strong_artifacts_height_mean_std, height, data.device).view(
        max_blocks, n
    )
    widths = _sample_sizes(max_blocks * n, cfg.strong_artifacts_width_mean_std, width, data.device).view(max_blocks, n)
    tops = (torch.rand(max_blocks, n, device=data.device) * (height - heights + 1)).to(torch.long)
    lefts = (torch.rand(max_blocks, n, device=data.device) * (width - widths + 1)).to(torch.long)
    values = choice_values(cfg.strong_artifact_value_choices, data.device, data.dtype)
    choice_ids = torch.randint(0, values.numel(), (max_blocks, n), device=data.device)
    rows, cols = row_col_grid(height, width, data.device, torch.long)

    for index in range(max_blocks):
        block = _rectangle_mask(rows, cols, tops[index], lefts[index], heights[index], widths[index])
        valid = active & (block_counts > index)
        masked_replace(data, valid.view(n, 1, 1, 1) & block.unsqueeze(-1), values[choice_ids[index]].view(n, 1, 1, 1))

    masked_replace(data, active, data.clamp(0.0, 1.0))


def _rectangle_mask(
    rows: torch.Tensor,
    cols: torch.Tensor,
    top: torch.Tensor,
    left: torch.Tensor,
    height: torch.Tensor,
    width: torch.Tensor,
) -> torch.Tensor:
    top = top.view(-1, 1, 1)
    left = left.view(-1, 1, 1)
    return (
        (rows >= top)
        & (rows < top + height.view(-1, 1, 1))
        & (cols >= left)
        & (cols < left + width.view(-1, 1, 1))
    )


def _sample_sizes(
    num_samples: int,
    mean_std: Sequence[float],
    max_size: int,
    device: torch.device,
) -> torch.Tensor:
    mean, std = mean_std
    sizes = mean + torch.randn((num_samples,), device=device) * std
    return sizes.round().to(dtype=torch.long).clamp_(min=1, max=max_size)
