# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from __future__ import annotations

from collections.abc import Sequence

import torch

from .cfg import DEPTH_CORRUPTION_MODE_NAMES, DepthCorruptionPatternCfg
from .modes.artifacts import apply_strong_artifact
from .modes.full_frame import apply_full_frame_mask
from .modes.salt_pepper import apply_salt_pepper
from .modes.sparkles import apply_sparkles
from .modes.stripes import apply_random_stripes
from .modes.strong_noise import apply_strong_noise

# Index 0 is handled separately because it takes extra full-frame value choices.
_PARTIAL_MODE_HANDLERS = {
    1: apply_strong_noise,
    2: apply_strong_artifact,
    3: apply_random_stripes,
    4: apply_sparkles,
    5: apply_salt_pepper,
}


def apply_selected_depth_corruption_modes(
    data: torch.Tensor,
    cfg: DepthCorruptionPatternCfg,
    modes_by_env: torch.Tensor | Sequence[int],
) -> torch.Tensor:
    """Preview helper: corrupt every frame and return a new tensor.

    Use this from offline scripts or visualizers when you already know the mode
    for each image. The input ``data`` is not modified. Internally this clones
    the batch and calls :func:`apply_active_depth_corruption` with ``active``
    set to all True.

    Training should call :func:`apply_active_depth_corruption` instead, so idle
    environments are left untouched.
    """
    if data.ndim != 4:
        raise ValueError(f"Expected depth data shaped (N, H, W, C), got {tuple(data.shape)}.")
    modes = torch.as_tensor(modes_by_env, device=data.device, dtype=torch.long)
    if modes.ndim != 1 or modes.shape[0] != data.shape[0]:
        raise ValueError(
            f"Expected one mode per depth frame, got modes={tuple(modes.shape)}, data={tuple(data.shape)}."
        )
    if ((modes < 0) | (modes >= len(DEPTH_CORRUPTION_MODE_NAMES))).any():
        raise ValueError(
            f"Mode indices must be in [0, {len(DEPTH_CORRUPTION_MODE_NAMES) - 1}], got {modes.tolist()}."
        )

    result = data.clone()
    apply_active_depth_corruption(result, cfg, torch.ones_like(modes, dtype=torch.bool), modes)
    return result


def apply_active_depth_corruption(
    data: torch.Tensor,
    cfg: DepthCorruptionPatternCfg,
    active: torch.Tensor,
    modes_by_env: torch.Tensor,
    full_frame_value_choices: torch.Tensor | None = None,
    enabled_modes: Sequence[int] | None = None,
) -> None:
    """Training path: paint modes in place. Masks stay on device.

    ``active[i]`` selects whether env ``i`` is currently in a corruption burst.
    ``enabled_modes`` is the host-side list of mode indices that can occur.
    ``None`` runs every mode. Painters must not read those masks back to the host.

    :class:`~locolab.utils.noise.noise_model.DepthCorruptionNoiseModel` passes the
    modes whose probability is positive. For a one-shot preview of chosen modes,
    use :func:`apply_selected_depth_corruption_modes`.
    """
    if enabled_modes is None:
        enabled_modes = range(len(DEPTH_CORRUPTION_MODE_NAMES))

    for mode_index in enabled_modes:
        mask = active & (modes_by_env == mode_index)
        if mode_index == 0:
            apply_full_frame_mask(data, cfg, mask, full_frame_value_choices)
        else:
            _PARTIAL_MODE_HANDLERS[mode_index](data, cfg, mask)
