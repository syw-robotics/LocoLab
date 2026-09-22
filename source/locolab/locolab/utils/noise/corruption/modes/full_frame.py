# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Replace the entire depth frame with a constant 0/1 value."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from ..cfg import DepthCorruptionPatternCfg


def apply_full_frame_mask(
    data: torch.Tensor,
    cfg: DepthCorruptionPatternCfg,
    active: torch.Tensor,
    value_choices: torch.Tensor | None,
) -> None:
    if value_choices is None:
        value_choices = torch.as_tensor(cfg.full_frame_value_choices, device=data.device, dtype=data.dtype)
    else:
        value_choices = value_choices.to(device=data.device, dtype=data.dtype)
    choice_ids = torch.randint(0, value_choices.numel(), (int(active.sum()),), device=data.device)
    data[active] = value_choices[choice_ids].view(-1, 1, 1, 1)
