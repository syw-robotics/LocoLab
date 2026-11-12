# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Visual patterns used during depth-corruption bursts.

This package only paints already-normalized depth images ``(N, H, W, C)``.
Burst timing lives in :class:`~locolab.utils.noise.noise_model.DepthCorruptionNoiseModel`.

Call :func:`apply_active_depth_corruption` during training (in-place, ``active``
mask). Call :func:`apply_selected_depth_corruption_modes` for offline previews
(returns a cloned batch, every frame is painted). A standalone gallery script
lives at ``preview/preview_depth_corruption.py``.

Mode index must match :data:`DEPTH_CORRUPTION_MODE_NAMES`:

0. ``full_frame_mask`` — replace the whole frame with 0/1
1. ``random_depth_noise`` — strong Gaussian noise
2. ``artifact_patch`` — rectangular stereo holes
3. ``random_stripe`` — thin bands at a random angle
4. ``sparkle`` — elliptical specular blobs
5. ``salt_pepper`` — scattered pixels replaced by 0 or 1
"""

from .apply import apply_active_depth_corruption, apply_selected_depth_corruption_modes
from .cfg import DEPTH_CORRUPTION_MODE_NAMES, DepthCorruptionPatternCfg

__all__ = [
    "DEPTH_CORRUPTION_MODE_NAMES",
    "DepthCorruptionPatternCfg",
    "apply_active_depth_corruption",
    "apply_selected_depth_corruption_modes",
]
