# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Configuration for custom terrains."""

import locolab.utils.terrains.locolab_terrains as locolab_terrain_gen
from locolab.utils.terrains import TerrainGeneratorCfg

import isaaclab.terrains as terrain_gen

"""Rough terrains configuration - for blind locomotion"""
ROUGH_TERRAINS_CFG = TerrainGeneratorCfg(
    curriculum=True,
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=20,
    horizontal_scale=0.1,
    vertical_scale=0.01,
    slope_threshold=0.75,
    use_cache=True,
    sub_terrains={
        "stairs_30": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.20,
            step_height_range=(0.05, 0.22),
            step_width=0.30,
            platform_width=3.0,
            border_width=0.5,
            holes=False,
        ),
        "stairs_30_inv": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.40,
            step_height_range=(0.05, 0.22),
            step_width=0.30,
            platform_width=3.0,
            border_width=0.5,
            holes=False,
        ),
        "slope": locolab_terrain_gen.HfPyramidSlopedRoughTerrainCfg(
            proportion=0.05,
            slope_range=(0.0, 0.45),
            platform_width=2.0,
            noise_range=(-0.05, 0.05),
            noise_step=0.01,
            apply_roughness=0.75,
            roughness_type="random",
            border_width=0.10,
        ),
        "slope_inv": locolab_terrain_gen.HfInvertedPyramidSlopedRoughTerrainCfg(
            proportion=0.2,
            slope_range=(0.0, 0.45),
            platform_width=3.0,
            noise_range=(-0.05, 0.05),
            noise_step=0.01,
            apply_roughness=0.75,
            roughness_type="random",
            border_width=0.10,
        ),
        "discrete": locolab_terrain_gen.HfDiscreteObstaclesTerrainCfg(
            proportion=0.2,
            obstacle_width_range=(1.0, 2.0),
            obstacle_height_range=(0.05, 0.25),
            num_obstacles=20,
            platform_width=3.0,
            noise_range=(-0.03, 0.05),
            noise_step=0.01,
            apply_roughness=0.75,
            roughness_type="random",
            border_width=0.10,
        ),
    },
)
