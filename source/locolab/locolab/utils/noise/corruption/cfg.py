# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from dataclasses import dataclass


DEPTH_CORRUPTION_MODE_NAMES = (
    "full_frame_mask",
    "random_depth_noise",
    "artifact_patch",
    "random_stripe",
    "sparkle",
    "salt_pepper",
)


@dataclass
class DepthCorruptionPatternCfg:
    """Parameters for the visual corruption modes.

    Values are in normalized depth units ``[0, 1]`` unless noted otherwise.
    """

    # 0. full_frame_mask
    full_frame_value_choices: tuple[float, ...] = (0.0, 1.0)

    # 1. random_depth_noise
    strong_noise_std_range: tuple[float, float] = (0.15, 0.45)

    # 2. artifact_patch
    strong_artifacts_prob: float = 0.01
    strong_artifacts_max_blocks: int = 3
    strong_artifacts_height_mean_std: tuple[float, float] = (8.0, 2.0)
    strong_artifacts_width_mean_std: tuple[float, float] = (8.0, 2.0)
    strong_artifact_value_choices: tuple[float, ...] = (0.0, 1.0)

    # 3. random_stripe
    stripe_count_range: tuple[int, int] = (2, 5)
    stripe_thickness_range: tuple[int, int] = (1, 4)
    stripe_angle_range_deg: tuple[float, float] = (0.0, 180.0)
    stripe_value_range: tuple[float, float] = (0.0, 1.0)

    # 4. sparkle
    sparkle_count_range: tuple[int, int] = (4, 12)
    sparkle_radius_range: tuple[float, float] = (1.5, 4.0)
    sparkle_aspect_ratio_range: tuple[float, float] = (0.6, 1.4)
    sparkle_value_choices: tuple[float, ...] = (0.0, 1.0)

    # 5. salt_pepper
    salt_pepper_amount_range: tuple[float, float] = (0.02, 0.10)
    salt_pepper_value_choices: tuple[float, ...] = (0.0, 1.0)
