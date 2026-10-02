# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Replace scattered pixels with 0 (pepper) or 1 (salt)."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from .common import choice_values, masked_replace

if TYPE_CHECKING:
    from ..cfg import DepthCorruptionPatternCfg


def apply_salt_pepper(data: torch.Tensor, cfg: DepthCorruptionPatternCfg, active: torch.Tensor) -> None:
    n, height, width, _ = data.shape
    device, dtype = data.device, data.dtype
    amount_min, amount_max = cfg.salt_pepper_amount_range
    amount = torch.empty((n, 1, 1), device=device, dtype=dtype).uniform_(amount_min, amount_max)
    hits = torch.rand((n, height, width), device=device, dtype=dtype) < amount
    values = choice_values(cfg.salt_pepper_value_choices, device, dtype)
    choice_ids = torch.randint(0, values.numel(), (n, height, width), device=device)
    masked_replace(data, active.reshape(-1, 1, 1, 1) & hits.unsqueeze(-1), values[choice_ids].unsqueeze(-1))
