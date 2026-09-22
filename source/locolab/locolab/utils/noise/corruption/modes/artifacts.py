# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Paint one or more rectangular stereo-hole patches."""

from __future__ import annotations

from typing import TYPE_CHECKING, Sequence

import torch

if TYPE_CHECKING:
    from ..cfg import DepthCorruptionPatternCfg


def apply_strong_artifact(data: torch.Tensor, cfg: DepthCorruptionPatternCfg, active: torch.Tensor) -> None:
    active_indices = torch.nonzero(active, as_tuple=False).squeeze(-1)
    if active_indices.numel() == 0 or cfg.strong_artifacts_prob <= 0.0 or cfg.strong_artifacts_max_blocks == 0:
        return

    height, width = data.shape[1:3]
    raw_expected_blocks = cfg.strong_artifacts_prob * height * width
    if raw_expected_blocks <= 0.0:
        return

    # Avoid saturating every frame at max_blocks on low-resolution images. The
    # raw per-pixel rate is still used, but its Poisson mean is capped at the
    # midpoint of the configured count range so 1..max_blocks remain observable.
    count_mean = min(raw_expected_blocks, (cfg.strong_artifacts_max_blocks + 1) / 2.0)
    block_counts = torch.poisson(torch.full((active_indices.numel(),), count_mean, device=data.device))
    block_counts = block_counts.to(dtype=torch.long).clamp_(min=1, max=cfg.strong_artifacts_max_blocks)
    for block_index in range(cfg.strong_artifacts_max_blocks):
        block_env_ids = active_indices[block_counts > block_index]
        if block_env_ids.numel() > 0:
            _apply_rectangular_artifact_block(data, cfg, block_env_ids)
    data[active] = data[active].clamp_(0.0, 1.0)


def _apply_rectangular_artifact_block(
    data: torch.Tensor,
    cfg: DepthCorruptionPatternCfg,
    env_ids: torch.Tensor,
) -> None:
    height, width = data.shape[1:3]
    block_heights = _sample_sizes(env_ids.numel(), cfg.strong_artifacts_height_mean_std, height, data.device)
    block_widths = _sample_sizes(env_ids.numel(), cfg.strong_artifacts_width_mean_std, width, data.device)
    tops = (torch.rand(env_ids.numel(), device=data.device) * (height - block_heights + 1)).to(torch.long)
    lefts = (torch.rand(env_ids.numel(), device=data.device) * (width - block_widths + 1)).to(torch.long)
    rows = torch.arange(height, device=data.device).view(1, height, 1)
    cols = torch.arange(width, device=data.device).view(1, 1, width)
    block_mask = (
        (rows >= tops.view(-1, 1, 1))
        & (rows < (tops + block_heights).view(-1, 1, 1))
        & (cols >= lefts.view(-1, 1, 1))
        & (cols < (lefts + block_widths).view(-1, 1, 1))
    ).unsqueeze(-1)
    value_choices = torch.as_tensor(cfg.strong_artifact_value_choices, device=data.device, dtype=data.dtype)
    choice_ids = torch.randint(0, value_choices.numel(), (env_ids.numel(),), device=data.device)
    data[env_ids] = torch.where(
        block_mask,
        value_choices[choice_ids].view(-1, 1, 1, 1),
        data[env_ids],
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
