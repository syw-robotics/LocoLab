# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Configuration for custom terrains."""

import isaaclab.terrains as terrain_gen

from loco_lab.utils.terrains import TerrainGeneratorCfg

"""Rough terrains configuration - for blind locomotion"""
ROUGH_TERRAINS_CFG = TerrainGeneratorCfg(
    curriculum=True,
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=20,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=True,
    sub_terrains={
        "stairs_30": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.25,
            step_height_range=(0.02, 0.22),
            step_width=0.30,
            platform_width=3.0,
            border_width=0.5,
            holes=False,
        ),
        "stairs_30_inv": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.25,
            step_height_range=(0.02, 0.22),
            step_width=0.30,
            platform_width=3.0,
            border_width=0.5,
            holes=False,
        ),
        "box": terrain_gen.MeshRandomGridTerrainCfg(
            proportion=0.15, grid_width=0.45, grid_height_range=(0.00, 0.15), platform_width=2.0
        ),
        "flat_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.15, noise_range=(0.00, 0.06), noise_step=0.01, border_width=0.10
        ),
        #  "wave": terrain_gen.HfWaveTerrainCfg(
        #      proportion=0.1, amplitude_range=(0.01, 0.10), num_waves=5.0, border_width=0.10
        #  ),
        "slope": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.1, slope_range=(0.0, 0.45), platform_width=2.0, border_width=0.10
        ),
        "slope_inv": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.1, slope_range=(0.0, 0.45), platform_width=2.0, border_width=0.10
        ),
    },
)
