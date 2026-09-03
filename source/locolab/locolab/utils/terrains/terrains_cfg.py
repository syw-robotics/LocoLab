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

"""Flat terrain with small geometric roughness."""
FLAT_ROUGH_TERRAINS_CFG = TerrainGeneratorCfg(
    curriculum=False,
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=5,
    num_cols=5,
    horizontal_scale=0.1,
    vertical_scale=0.01,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        "flat_rough": locolab_terrain_gen.HfFlatRoughTerrainCfg(
            proportion=1.0,
            noise_range=(-0.06, 0.06),
            noise_step=0.01,
            downsampled_scale=0.1,
            roughness_type="random",
            apply_roughness=0.8,
        ),
    },
)

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
    use_cache=False,
    sub_terrains={
        "stairs_30": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.20,
            step_height_range=(0.02, 0.22),
            step_width=0.30,
            platform_width=3.0,
            border_width=0.5,
            holes=False,
        ),
        "stairs_30_inv": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.40,
            step_height_range=(0.02, 0.22),
            step_width=0.30,
            platform_width=3.0,
            border_width=0.5,
            holes=False,
        ),
        #  "stairs_30": locolab_terrain_gen.MeshRandomWidthPyramidStairsTerrainCfg(
        #      proportion=0.20,
        #      step_height_range=(0.05, 0.22),
        #      step_width_range=(0.25, 0.35),
        #      step_width_step=0.02,
        #      platform_width=3.0,
        #      border_width=0.5,
        #      holes=False,
        #  ),
        #  "stairs_30_inv": locolab_terrain_gen.MeshRandomWidthPyramidStairsTerrainCfg(
        #      proportion=0.40,
        #      step_height_range=(0.05, 0.22),
        #      step_width_range=(0.25, 0.35),
        #      step_width_step=0.02,
        #      platform_width=3.0,
        #      border_width=0.5,
        #      holes=False,
        #      inverted=True,
        #  ),
        "slope": locolab_terrain_gen.HfPyramidSlopedRoughTerrainCfg(
            proportion=0.05,
            slope_range=(0.0, 0.45),
            platform_width=2.0,
            noise_range=(-0.08, 0.08),
            noise_step=0.01,
            apply_roughness=0.8,
            roughness_type="random",
            border_width=0.10,
        ),
        "slope_inv": locolab_terrain_gen.HfInvertedPyramidSlopedRoughTerrainCfg(
            proportion=0.2,
            slope_range=(0.0, 0.45),
            platform_width=3.0,
            noise_range=(-0.08, 0.08),
            noise_step=0.01,
            apply_roughness=0.8,
            roughness_type="random",
            border_width=0.10,
        ),
        "discrete": locolab_terrain_gen.HfDiscreteObstaclesTerrainCfg(
            proportion=0.2,
            obstacle_width_range=(1.0, 2.0),
            obstacle_height_range=(0.02, 0.22),
            num_obstacles=20,
            platform_width=3.0,
            noise_range=(-0.05, 0.05),
            noise_step=0.01,
            apply_roughness=0.8,
            roughness_type="random",
            border_width=0.10,
        ),
    },
)

"""Rough parkour terrains configuration - for perceptive locomotion"""
PARKOUR_TERRAINS_CFG = TerrainGeneratorCfg(
    curriculum=True,
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=20,
    horizontal_scale=0.1,
    vertical_scale=0.01,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        "stairs_30": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.10,
            step_height_range=(0.10, 0.30),
            step_width=0.30,
            platform_width=3.0,
            border_width=0.5,
            holes=False,
        ),
        "stairs_30_inv": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.15,
            step_height_range=(0.10, 0.30),
            step_width=0.30,
            platform_width=3.0,
            border_width=0.5,
            holes=False,
        ),
        "slope": locolab_terrain_gen.HfPyramidSlopedRoughTerrainCfg(
            proportion=0.05,
            slope_range=(0.0, 0.45),
            platform_width=2.0,
            noise_range=(-0.08, 0.08),
            noise_step=0.01,
            apply_roughness=0.8,
            roughness_type="random",
            border_width=0.10,
        ),
        "slope_inv": locolab_terrain_gen.HfInvertedPyramidSlopedRoughTerrainCfg(
            proportion=0.05,
            slope_range=(0.0, 0.45),
            platform_width=3.0,
            noise_range=(-0.08, 0.08),
            noise_step=0.01,
            apply_roughness=0.8,
            roughness_type="random",
            border_width=0.10,
        ),
        "discrete": locolab_terrain_gen.HfDiscreteObstaclesTerrainCfg(
            proportion=0.05,
            obstacle_width_range=(1.0, 2.0),
            obstacle_height_range=(0.10, 0.35),
            num_obstacles=20,
            platform_width=3.0,
            noise_range=(-0.05, 0.05),
            noise_step=0.01,
            apply_roughness=0.8,
            roughness_type="random",
            border_width=0.10,
        ),
        "box": locolab_terrain_gen.MeshRepeatedBoxesTerrainCfg(
            proportion=0.1,
            platform_width=3.0,
            num_objects_range=(20, 36),
            num_objects_type="random",
            box_height_range=(0.10, 0.35),
            box_length_range=(0.30, 1.0),
            box_width_range=(0.30, 1.0),
            angle_range=(0.0, 15.0),
            angle_type="random",
            angle_degrees=True,
            abs_height_noise=(-0.08, 0.08),
            rel_height_noise=(1.0, 2.0),
        ),
        "gap": locolab_terrain_gen.HfGapTerrainCfg(
            proportion=0.15,
            gap_width_range=(0.10, 0.70),
            gap_depth_range=(1.0, 2.5),
            gap_depth_type="random",
            platform_width_range=(2.0, 4.0),
            platform_height_range=(-0.10, 0.10),
        ),
        "double_gap": locolab_terrain_gen.HfDoubleGapTerrainCfg(
            proportion=0.1,
            gap_width_range=(0.10, 0.50),
            gap_depth_range=(1.0, 2.5),
            gap_depth_type="random",
            gap_in_between_width_range=(0.5, 1.5),
            platform_width_range=(2.0, 3.0),
            platform_height_range=(-0.10, 0.10),
        ),
        "stairs_high": locolab_terrain_gen.HfPyramidStairsTerrainCfg(
            proportion=0.05,
            step_height_range=(0.15, 0.50),
            step_width=1.0,
            platform_width=3.0,
            noise_range=(-0.05, 0.05),
            noise_step=0.01,
            apply_roughness=0.8,
            roughness_type="random",
            border_width=0.5,
        ),
        "stairs_high_inv": locolab_terrain_gen.HfInvertedPyramidStairsTerrainCfg(
            proportion=0.1,
            step_height_range=(0.15, 0.50),
            step_width=1.0,
            platform_width=3.0,
            noise_range=(-0.05, 0.05),
            noise_step=0.01,
            apply_roughness=0.8,
            roughness_type="random",
            border_width=0.5,
        ),
        "hurdle": locolab_terrain_gen.MeshHurdleTerrainCfg(
            proportion=0.1,
            hurdle_width_range=(0.1, 0.4),
            hurdle_height_range=(0.15, 0.45),
            platform_width_range=(3.0, 5.0),
        ),
    },
)
