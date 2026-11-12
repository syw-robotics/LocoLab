# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Device-side helpers shared by the corruption painters."""

from __future__ import annotations

from collections.abc import Sequence

import torch

_VALUE_CACHE: dict[tuple, torch.Tensor] = {}
_GRID_CACHE: dict[tuple, tuple[torch.Tensor, torch.Tensor]] = {}


def choice_values(values: Sequence[float], device: torch.device, dtype: torch.dtype) -> torch.Tensor:
    """Return a cached 1-D tensor of replacement values."""
    key = (tuple(values), device, dtype)
    cached = _VALUE_CACHE.get(key)
    if cached is None:
        cached = torch.tensor(key[0], device=device, dtype=dtype)
        _VALUE_CACHE[key] = cached
    return cached


def row_col_grid(
    height: int, width: int, device: torch.device, dtype: torch.dtype
) -> tuple[torch.Tensor, torch.Tensor]:
    """Return ``rows`` shaped ``(1, H, 1)`` and ``cols`` shaped ``(1, 1, W)``."""
    key = (height, width, device, dtype)
    cached = _GRID_CACHE.get(key)
    if cached is None:
        cached = (
            torch.arange(height, device=device, dtype=dtype).view(1, height, 1),
            torch.arange(width, device=device, dtype=dtype).view(1, 1, width),
        )
        _GRID_CACHE[key] = cached
    return cached


def masked_replace(data: torch.Tensor, mask: torch.Tensor, value: torch.Tensor) -> None:
    """Write ``value`` into ``data`` where ``mask`` is true.

    A row mask of shape ``(N,)`` broadcasts over ``H, W, C``. The write stays on
    device; do not reduce ``mask`` with ``any``, ``sum``, or boolean indexing.
    """
    if mask.ndim == 1:
        mask = mask.reshape(-1, 1, 1, 1)
    data.copy_(torch.where(mask, value, data))
