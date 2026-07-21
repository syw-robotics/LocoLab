"""Representative terrain parameters used by the preview script.

These values are intentionally separate from training configs.  They should
make each terrain visually recognizable at the default preview difficulty.
"""

ABSTRACT_CFG_NAMES = {"HfRoughTerrainCfg"}
"""Terrain cfg classes exported by the package but not meant to be previewed directly."""


PREVIEW_CASES: dict[str, dict] = {
    "HfPyramidSlopedRoughTerrainCfg": {
        "slope_range": (0.0, 0.6),
        "platform_width": 3.0,
        "noise_range": (-0.02, 0.08),
        "noise_step": 0.01,
        "apply_roughness": True,
        "roughness_type": "difficulty",
        "border_width": 0.1,
    },
    "HfInvertedPyramidSlopedRoughTerrainCfg": {
        "slope_range": (0.0, 0.6),
        "platform_width": 3.0,
        "noise_range": (-0.02, 0.08),
        "noise_step": 0.01,
        "apply_roughness": True,
        "roughness_type": "difficulty",
        "border_width": 0.1,
    },
    "HfDiscreteObstaclesTerrainCfg": {
        "obstacle_width_range": (1.0, 2.0),
        "obstacle_height_range": (0.05, 0.25),
        "num_obstacles": 15,
        "platform_width": 3.0,
        "border_width": 0.1,
    },
    "HfGapTerrainCfg": {
        "gap_width_range": (0.1, 0.9),
        "gap_depth_range": (1.0, 3.0),
        "platform_width_range": (3.0, 4.0),
        "platform_height_range": (-0.1, 0.1),
        "border_width": 0.1,
    },
    "HfDoubleGapTerrainCfg": {
        "gap_width_range": (0.2, 0.6),
        "gap_depth_range": (0.5, 1.0),
        "gap_in_between_width_range": (0.8, 1.0),
        "platform_width_range": (3.0, 4.0),
        "platform_height_range": (-0.1, 0.1),
        "border_width": 0.1,
    },
    "HfStraightGapTerrainCfg": {
        "gap_width_range": (0.2, 0.8),
        "gap_depth_range": (1.0, 3.0),
        "gap_offset_range": (0.5, 0.8),
        "platform_width_range": (3.0, 4.0),
        "platform_height_range": (-0.1, 0.1),
        "border_width": 0.1,
    },
    "HfHurdleTerrainCfg": {
        "hurdle_width_range": (0.2, 0.5),
        "hurdle_height_range": (0.05, 0.25),
        "platform_width_range": (3.0, 4.0),
        "border_width": 0.1,
    },
    "MeshStraightGapTerrainCfg": {
        "gap_width_range": (0.2, 0.8),
        "gap_depth_range": (0.5, 1.0),
        "gap_y_length_range": (2.0, 6.0),
        "gap_x_offset_range": (-0.2, 0.2),
    },
    "MeshStraightStairsTerrainCfg": {
        "stair_width": 0.3,
        "stair_width_noise_range": (-0.02, 0.02),
        "stair_height_range": (0.05, 0.22),
        "stair_length_range": (2.0, 6.0),
        "num_stairs_range": (2, 7),
        "platform_width_range": (3.0, 4.0),
    },
    "MeshInvertedStraightStairsTerrainCfg": {
        "stair_width": 0.3,
        "stair_width_noise_range": (-0.02, 0.02),
        "stair_height_range": (0.05, 0.22),
        "stair_length_range": (2.0, 6.0),
        "num_stairs_range": (2, 7),
        "platform_width_range": (3.0, 4.0),
    },
}
"""Concrete preview kwargs keyed by exported terrain cfg class name."""
