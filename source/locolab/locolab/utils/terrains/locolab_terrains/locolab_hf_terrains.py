"""LocoLab custom height-field terrains."""

from __future__ import annotations

import numpy as np

from isaaclab.terrains.height_field.utils import height_field_to_mesh

from . import locolab_hf_terrains_cfg
from .utils import _maybe_apply_roughness


@height_field_to_mesh
def flat_rough_terrain(difficulty: float, cfg: locolab_hf_terrains_cfg.HfFlatRoughTerrainCfg) -> np.ndarray:
    """Generate flat terrain with LocoLab fractal Perlin roughness."""
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)
    hf_raw = np.zeros((width_pixels, length_pixels), dtype=np.int16)
    return _maybe_apply_roughness(cfg, hf_raw, difficulty)


@height_field_to_mesh
def pyramid_sloped_rough_terrain(difficulty: float, cfg: locolab_hf_terrains_cfg.HfPyramidSlopedRoughTerrainCfg) -> np.ndarray:
    """Generate a terrain with a truncated pyramid structure with optional roughness.

    The terrain is a pyramid-shaped sloped surface with a slope of :obj:`slope` that trims into a flat platform
    at the center. The slope is defined as the ratio of the height change along the x axis to the width along the
    x axis. For example, a slope of 1.0 means that the height changes by 1 unit for every 1 unit of width.

    If the :obj:`cfg.inverted` flag is set to :obj:`True`, the terrain is inverted such that
    the platform is at the bottom.

    Roughness is randomly applied to the terrain surface according to :obj:`cfg.apply_roughness`.

    Args:
        difficulty: The difficulty of the terrain. This is a value between 0 and 1.
        cfg: The configuration for the terrain.

    Returns:
        The height field of the terrain as a 2D numpy array with discretized heights.
        The shape of the array is (width, length), where width and length are the number of points
        along the x and y axis, respectively.
    """
    # resolve terrain configuration
    if cfg.inverted:
        slope = -cfg.slope_range[0] - difficulty * (cfg.slope_range[1] - cfg.slope_range[0])
    else:
        slope = cfg.slope_range[0] + difficulty * (cfg.slope_range[1] - cfg.slope_range[0])

    # switch parameters to discrete units
    # -- horizontal scale
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)
    # -- height
    # we want the height to be 1/2 of the width since the terrain is a pyramid
    height_max = int(slope * cfg.size[0] / 2 / cfg.vertical_scale)
    # -- center of the terrain
    center_x = int(width_pixels / 2)
    center_y = int(length_pixels / 2)

    # create a meshgrid of the terrain
    x = np.arange(0, width_pixels)
    y = np.arange(0, length_pixels)
    xx, yy = np.meshgrid(x, y, sparse=True)
    # offset the meshgrid to the center of the terrain
    xx = (center_x - np.abs(center_x - xx)) / center_x
    yy = (center_y - np.abs(center_y - yy)) / center_y
    # reshape the meshgrid to be 2D
    xx = xx.reshape(width_pixels, 1)
    yy = yy.reshape(1, length_pixels)
    # create a sloped surface
    hf_raw = np.zeros((width_pixels, length_pixels))
    hf_raw = height_max * xx * yy

    # create a flat platform at the center of the terrain
    platform_width = int(cfg.platform_width / cfg.horizontal_scale / 2)
    # get the height of the platform at the corner of the platform
    x_pf = width_pixels // 2 - platform_width
    y_pf = length_pixels // 2 - platform_width
    z_pf = hf_raw[x_pf, y_pf]
    hf_raw = np.clip(hf_raw, min(0, z_pf), max(0, z_pf))

    hf_raw = _maybe_apply_roughness(cfg, hf_raw, difficulty)

    # round off the heights to the nearest vertical step
    return np.rint(hf_raw).astype(np.int16)


@height_field_to_mesh
def discrete_obstacles_terrain(
    difficulty: float, cfg: locolab_hf_terrains_cfg.HfDiscreteObstaclesTerrainCfg
) -> np.ndarray:
    """Generate legged_gym-style discrete rectangular obstacles."""
    # resolve terrain configuration
    obstacle_height = cfg.obstacle_height_range[0] + difficulty * (
        cfg.obstacle_height_range[1] - cfg.obstacle_height_range[0]
    )

    # switch parameters to discrete units
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)
    obstacle_height = int(obstacle_height / cfg.vertical_scale)
    obstacle_width_min = int(cfg.obstacle_width_range[0] / cfg.horizontal_scale)
    obstacle_width_max = int(cfg.obstacle_width_range[1] / cfg.horizontal_scale)
    platform_width = int(cfg.platform_width / cfg.horizontal_scale)

    if obstacle_height <= 0:
        raise ValueError(f"Obstacle height must be positive. Got: {obstacle_height * cfg.vertical_scale}.")
    if obstacle_width_min <= 0:
        raise ValueError(f"Minimum obstacle width must be positive. Got: {cfg.obstacle_width_range[0]}.")
    if obstacle_width_max < obstacle_width_min:
        raise ValueError(
            "Maximum obstacle width must be greater than or equal to the minimum obstacle width:"
            f" {cfg.obstacle_width_range[1]} < {cfg.obstacle_width_range[0]}."
        )

    # create discrete ranges for the legged_gym-style obstacles
    obstacle_width_range = np.arange(obstacle_width_min, obstacle_width_max + 1, 4)
    obstacle_length_range = np.arange(obstacle_width_min, obstacle_width_max + 1, 4)
    if len(obstacle_width_range) == 0:
        obstacle_width_range = np.array([obstacle_width_min])
    if len(obstacle_length_range) == 0:
        obstacle_length_range = np.array([obstacle_width_min])

    height_range = np.array([-obstacle_height, -obstacle_height // 2, obstacle_height // 2, obstacle_height], dtype=np.int16)
    hf_raw = np.zeros((width_pixels, length_pixels), dtype=np.int16)

    for _ in range(cfg.num_obstacles):
        width = int(np.random.choice(obstacle_width_range))
        length = int(np.random.choice(obstacle_length_range))
        width = min(width, width_pixels)
        length = min(length, length_pixels)

        x_start_range = np.arange(0, max(width_pixels - width, 0) + 1, 4)
        y_start_range = np.arange(0, max(length_pixels - length, 0) + 1, 4)
        x_start = int(np.random.choice(x_start_range))
        y_start = int(np.random.choice(y_start_range))

        hf_raw[x_start : x_start + width, y_start : y_start + length] = np.random.choice(height_range)

    # Keep the center platform free of discrete obstacles before adding surface roughness.
    platform_width = min(platform_width, width_pixels, length_pixels)
    x1 = (width_pixels - platform_width) // 2
    x2 = (width_pixels + platform_width) // 2
    y1 = (length_pixels - platform_width) // 2
    y2 = (length_pixels + platform_width) // 2
    hf_raw[x1:x2, y1:y2] = 0

    hf_raw = _maybe_apply_roughness(cfg, hf_raw, difficulty)

    return np.rint(hf_raw).astype(np.int16)


@height_field_to_mesh
def gap_terrain(
    difficulty: float, cfg: locolab_hf_terrains_cfg.HfGapTerrainCfg
) -> np.ndarray:
    """Generate rectangular gap terrain."""
    # 0------x_11         x_1-----x_2          x_22---end
    # --------- gap_size ----------- gap_size --------
    #          ----------           ----------
    # 0------y11         y_1-----y_2          y_22---end
    # --------- gap_size ----------- gap_size --------
    #          ----------           ----------

    # resolve terrain configuration
    gap_width = (cfg.gap_width_range[1] - cfg.gap_width_range[0]) * difficulty + cfg.gap_width_range[0]
    gap_width_pixels = int(gap_width / cfg.horizontal_scale)
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)

    center_x_pixels = 0.5 * cfg.size[0] / cfg.horizontal_scale
    center_y_pixels = 0.5 * cfg.size[1] / cfg.horizontal_scale
    platform_width = (cfg.platform_width_range[1] - cfg.platform_width_range[0]) * np.random.random() + cfg.platform_width_range[0]
    half_platform_width_pixels = int(0.5 * platform_width / cfg.horizontal_scale)

    # x direction
    x1 = int(center_x_pixels - half_platform_width_pixels)
    x2 = int(center_x_pixels + half_platform_width_pixels)
    x11 = x1 - gap_width_pixels
    x22 = x2 + gap_width_pixels

    # y direction
    y1 = int(center_y_pixels - half_platform_width_pixels)
    y2 = int(center_y_pixels + half_platform_width_pixels)
    y11 = y1 - gap_width_pixels
    y22 = y2 + gap_width_pixels

    # height for x1-x2 platform: -0.10m to 0.10m
    platform_height = (
        cfg.platform_height_range[1] - cfg.platform_height_range[0]
    ) * np.random.random() + cfg.platform_height_range[0]
    platform_height_pixels = int(platform_height / cfg.vertical_scale)

    # gap depth
    if cfg.gap_depth_type == "difficulty":
        gap_depth = (cfg.gap_depth_range[1] - cfg.gap_depth_range[0]) * difficulty + cfg.gap_depth_range[0]
    elif cfg.gap_depth_type == "random":
        gap_depth = np.random.uniform(cfg.gap_depth_range[0], cfg.gap_depth_range[1])
    else:
        raise ValueError(f"cfg.gap_depth_type' must be 'difficulty' or 'random'. Current value is `{cfg.gap_depth_type}`.")
    gap_depth_pixels = int(gap_depth / cfg.vertical_scale)

    # write height field array
    hf_raw = np.zeros((width_pixels, length_pixels))
    hf_raw[:, :] = -gap_depth_pixels
    hf_raw[:x11, ::] = 0
    hf_raw[x1:x2, y1:y2] = platform_height_pixels
    hf_raw[x11:x22, :y11] = 0
    hf_raw[x11:x22, y22:] = 0
    hf_raw[x22:, :] = 0

    hf_raw = _maybe_apply_roughness(cfg, hf_raw, difficulty)

    # round off the heights to the nearest vertical step
    return np.rint(hf_raw).astype(np.int16)


@height_field_to_mesh
def double_gap_terrain(
    difficulty: float, cfg: locolab_hf_terrains_cfg.HfDoubleGapTerrainCfg
) -> np.ndarray:
    """Generate double rectangular gap terrain."""
    # 0------x_33         x_3-----x_11          x_1------x_2         x_22-----x_4          x_4---end
    # --------- gap_size ----------- gap_size -------------- gap_size ----------- gap_size --------
    #          ----------           ----------              ----------           ----------
    # 0------y_33         y_3-----y_11          y_1------y_2         y_22-----y_4          y_44---end
    # --------- gap_size ----------- gap_size -------------- gap_size ----------- gap_size --------
    #          ----------           ----------              ----------           ----------

    # resolve terrain configuration
    gap_width = (cfg.gap_width_range[1] - cfg.gap_width_range[0]) * difficulty + cfg.gap_width_range[0]
    gap_width_pixels = int(gap_width / cfg.horizontal_scale)
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)

    center_x_pixels = 0.5 * cfg.size[0] / cfg.horizontal_scale
    center_y_pixels = 0.5 * cfg.size[1] / cfg.horizontal_scale
    platform_width = (cfg.platform_width_range[1] - cfg.platform_width_range[0]) * np.random.random() + cfg.platform_width_range[0]
    half_platform_width_pixels = int(0.5 * platform_width / cfg.horizontal_scale)

    gap_in_between_width = (cfg.gap_in_between_width_range[1] - cfg.gap_in_between_width_range[0]) * np.random.random() + cfg.gap_in_between_width_range[0]
    gap_in_between_width_pixels = int(gap_in_between_width / cfg.horizontal_scale)

    # x direction
    x1 = int(center_x_pixels - half_platform_width_pixels)
    x2 = int(center_x_pixels + half_platform_width_pixels)
    x11 = x1 - gap_width_pixels
    x22 = x2 + gap_width_pixels
    x3 = x11 - gap_in_between_width_pixels
    x4 = x22 + gap_in_between_width_pixels
    x33 = x3 - gap_width_pixels
    x44 = x4 + gap_width_pixels

    # y direction
    y1 = int(center_y_pixels - half_platform_width_pixels)
    y2 = int(center_y_pixels + half_platform_width_pixels)
    y11 = y1 - gap_width_pixels
    y22 = y2 + gap_width_pixels
    y3 = y11 - gap_in_between_width_pixels
    y4 = y22 + gap_in_between_width_pixels
    y33 = y3 - gap_width_pixels
    y44 = y4 + gap_width_pixels

    # height for x1-x2 platform: -0.10m to 0.10m
    platform_height = (
        cfg.platform_height_range[1] - cfg.platform_height_range[0]
    ) * np.random.random() + cfg.platform_height_range[0]
    platform_height_pixels = int(platform_height / cfg.vertical_scale)

    # gap depth
    if cfg.gap_depth_type == "difficulty":
        gap_depth = (cfg.gap_depth_range[1] - cfg.gap_depth_range[0]) * difficulty + cfg.gap_depth_range[0]
    elif cfg.gap_depth_type == "random":
        gap_depth = np.random.uniform(cfg.gap_depth_range[0], cfg.gap_depth_range[1])
    else:
        raise ValueError(f"cfg.gap_depth_type' must be 'difficulty' or 'random'. Current value is `{cfg.gap_depth_type}`.")
    gap_depth_pixels = int(gap_depth / cfg.vertical_scale)

    # write height field array
    hf_raw = np.zeros((width_pixels, length_pixels))
    hf_raw[:, :] = -gap_depth_pixels
    hf_raw[:x33, ::] = 0
    hf_raw[x44:, ::] = 0
    hf_raw[x33:x44, :y33] = 0
    hf_raw[x33:x44, y44:] = 0

    hf_raw[x3:x11, y3:y4] = 0
    hf_raw[x22:x4, y3:y4] = 0
    hf_raw[x11:x22, y3:y11] = 0
    hf_raw[x11:x22, y22:y4] = 0

    hf_raw[x1:x2, y1:y2] = platform_height_pixels

    hf_raw = _maybe_apply_roughness(cfg, hf_raw, difficulty)

    # round off the heights to the nearest vertical step
    return np.rint(hf_raw).astype(np.int16)


@height_field_to_mesh
def straight_gap_terrain(
    difficulty: float, cfg: locolab_hf_terrains_cfg.HfStraightGapTerrainCfg
) -> np.ndarray:
    """Generate straight gap terrain, gap only in x direction."""
    # 0------x_3         x_1-----x_2          x_4---end
    # --------- gap_size ----------- gap_size --------
    #          ----------           ----------
    gap_width = (cfg.gap_width_range[1] - cfg.gap_width_range[0]) * difficulty + cfg.gap_width_range[0]
    gap_width_pixels = int(gap_width / cfg.horizontal_scale)
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)
    gap_offset = (cfg.gap_offset_range[1] - cfg.gap_offset_range[0]) * np.random.random() + cfg.gap_offset_range[0]
    gap_offset_pixels = int(gap_offset / cfg.horizontal_scale)

    center_x_pixels = 0.5 * cfg.size[0] / cfg.horizontal_scale
    center_y_pixels = 0.5 * cfg.size[1] / cfg.horizontal_scale
    x1 = int(center_x_pixels - gap_offset_pixels)
    x2 = int(center_x_pixels + gap_offset_pixels)
    x3 = x1 - gap_width_pixels
    x4 = x2 + gap_width_pixels
    if x3 < 0 or x4 > width_pixels:
        raise ValueError(
            "The straight-gap offset and width must keep both gaps inside the terrain: "
            f"x3={x3}, x4={x4}, width_pixels={width_pixels}. "
            "Reduce gap_offset_range or gap_width_range."
        )

    platform_width = (
        cfg.platform_width_range[1]
        if difficulty < cfg.easy_difficulty_threshold  #
        else (cfg.platform_width_range[1] - cfg.platform_width_range[0]) * np.random.random()
        + cfg.platform_width_range[0]
    )
    half_platform_width_pixels = int(0.5 * platform_width / cfg.horizontal_scale)
    y1 = int(center_y_pixels - half_platform_width_pixels)
    y2 = int(center_y_pixels + half_platform_width_pixels)

    # height for x1-x2 platform
    platform_height = (
        cfg.platform_height_range[1] - cfg.platform_height_range[0]
    ) * np.random.random() + cfg.platform_height_range[0]
    platform_height_pixels = int(platform_height / cfg.vertical_scale)

    # gap depth
    if cfg.gap_depth_type == "difficulty":
        gap_depth = (cfg.gap_depth_range[1] - cfg.gap_depth_range[0]) * difficulty + cfg.gap_depth_range[0]
    elif cfg.gap_depth_type == "random":
        gap_depth = np.random.uniform(cfg.gap_depth_range[0], cfg.gap_depth_range[1])
    else:
        raise ValueError(f"cfg.gap_depth_type' must be 'difficulty' or 'random'. Current value is `{cfg.gap_depth_type}`.")
    gap_depth_pixels = int(gap_depth / cfg.vertical_scale)

    hf_raw = np.zeros((width_pixels, length_pixels))
    hf_raw[:, :] = -gap_depth_pixels
    hf_raw[:x3, y1:y2] = 0
    hf_raw[x1:x2, y1:y2] = platform_height_pixels
    hf_raw[x4:, y1:y2] = 0

    hf_raw = _maybe_apply_roughness(cfg, hf_raw, difficulty)
    return np.rint(hf_raw).astype(np.int16)

@height_field_to_mesh
def hurdle_terrain(
    difficulty: float, cfg: locolab_hf_terrains_cfg.HfHurdleTerrainCfg
) -> np.ndarray:
    """Generate rectangular gap terrain."""
    # 0------x_11         x_1-----x_2          x_22---end
    #          ----------           ----------
    # --------- hurdle_size ----------- hurdle_size --------
    # 0------y11         y_1-----y_2          y_22---end
    #          ----------           ----------
    # --------- hurdle_size ----------- hurdle_size --------

    # resolve terrain configuration
    # hurdle width from wide to narrow
    hurdle_width = - (cfg.hurdle_width_range[1] - cfg.hurdle_width_range[0]) * difficulty + cfg.hurdle_width_range[1] + cfg.hurdle_width_range[0]
    hurdle_width_pixels = int(hurdle_width / cfg.horizontal_scale)
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)

    center_x_pixels = 0.5 * cfg.size[0] / cfg.horizontal_scale
    center_y_pixels = 0.5 * cfg.size[1] / cfg.horizontal_scale
    platform_width = (cfg.platform_width_range[1] - cfg.platform_width_range[0]) * np.random.random() + cfg.platform_width_range[0]
    half_platform_width_pixels = int(0.5 * platform_width / cfg.horizontal_scale)

    # x direction
    x1 = int(center_x_pixels - half_platform_width_pixels)
    x2 = int(center_x_pixels + half_platform_width_pixels)
    x11 = x1 - hurdle_width_pixels
    x22 = x2 + hurdle_width_pixels

    # y direction
    y1 = int(center_y_pixels - half_platform_width_pixels)
    y2 = int(center_y_pixels + half_platform_width_pixels)
    y11 = y1 - hurdle_width_pixels
    y22 = y2 + hurdle_width_pixels

    # hurdle height from short to tall
    hurdle_height = (cfg.hurdle_height_range[1] - cfg.hurdle_height_range[0]) * difficulty + cfg.hurdle_height_range[0]
    hurdle_height_pixels = int(hurdle_height / cfg.vertical_scale)

    # write height field array
    hf_raw = np.zeros((width_pixels, length_pixels))
    hf_raw[:, :] = 0
    hf_raw[x11:x1, y11:y22] = hurdle_height_pixels
    hf_raw[x2:x22, y11:y22] = hurdle_height_pixels
    hf_raw[x1:x2, y11:y1] = hurdle_height_pixels
    hf_raw[x1:x2, y2:y22] = hurdle_height_pixels

    hf_raw = _maybe_apply_roughness(cfg, hf_raw, difficulty)

    # round off the heights to the nearest vertical step
    return np.rint(hf_raw).astype(np.int16)
