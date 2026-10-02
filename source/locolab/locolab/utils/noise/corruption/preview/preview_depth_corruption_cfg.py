# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Display settings for the depth-corruption preview script.

Pattern parameters stay on ``DepthCorruptionPatternCfg``. This file only
describes how the gallery labels each mode and how large the synthetic
depth image is. Keys must match ``DEPTH_CORRUPTION_MODE_NAMES``.
"""

MODE_BLURBS = {
    "full_frame_mask": "Replaces every pixel with a constant 0 or 1.",
    "random_depth_noise": "Adds strong Gaussian noise, then clips back to [0, 1].",
    "artifact_patch": "Paints a few rectangular holes filled with 0 or 1.",
    "random_stripe": "Draws thin bands at a random angle, each with its own value in [0, 1].",
    "sparkle": "Draws elliptical blobs filled with 0 or 1.",
    "salt_pepper": "Replaces a random fraction of pixels with 0 (pepper) or 1 (salt).",
}

DEFAULT_HEIGHT = 48
DEFAULT_WIDTH = 64
DEFAULT_SAMPLES = 4
DEFAULT_SEED = 7
DEPTH_CMAP = "magma"
FIGURE_BG = "#f7f5ef"
