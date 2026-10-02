# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Add strong Gaussian noise and clip back to ``[0, 1]``."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from .common import masked_replace

if TYPE_CHECKING:
    from ..cfg import DepthCorruptionPatternCfg


def apply_strong_noise(data: torch.Tensor, cfg: DepthCorruptionPatternCfg, active: torch.Tensor) -> None:
    std_min, std_max = cfg.strong_noise_std_range
    std = torch.empty((data.shape[0], 1, 1, 1), device=data.device, dtype=data.dtype).uniform_(std_min, std_max)
    noisy = (data + torch.randn_like(data) * std).clamp(0.0, 1.0)
    masked_replace(data, active, noisy)
