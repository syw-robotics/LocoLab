"""LocoLab custom height-field terrains."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import scipy.interpolate as interpolate

from isaaclab.terrains.height_field.utils import height_field_to_mesh

if TYPE_CHECKING:
    from . import locolab_terrains_cfg
    from isaaclab.terrains.height_field import hf_terrains_cfg


def apply_random_uniform_noise(
    cfg: hf_terrains_cfg.HfRandomUniformTerrainCfg, hf_raw: np.ndarray, difficulty: float = 0.0
) -> np.ndarray:
    # check parameters
    # -- horizontal scale
    if cfg.downsampled_scale is None:
        cfg.downsampled_scale = cfg.horizontal_scale
    elif cfg.downsampled_scale < cfg.horizontal_scale:
        raise ValueError(
            "Downsampled scale must be larger than or equal to the horizontal scale:"
            f" {cfg.downsampled_scale} < {cfg.horizontal_scale}."
        )

    # switch parameters to discrete units
    # -- horizontal scale
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)
    # -- downsampled scale
    width_downsampled = int(cfg.size[0] / cfg.downsampled_scale)
    length_downsampled = int(cfg.size[1] / cfg.downsampled_scale)
    # -- height
    # Determine max_height based on roughness_type
    roughness_type = getattr(cfg, "roughness_type", "fixed")
    if roughness_type == "difficulty":
        max_height = (cfg.noise_range[1] - cfg.noise_range[0]) * difficulty + cfg.noise_range[
            0
        ]  # roughness intensity is difficulty-based
    elif roughness_type == "random":
        max_height = np.random.uniform(
            (cfg.noise_range[0] + cfg.noise_range[1]), cfg.noise_range[1]
        )  # randomly sample roughness intensity
    elif roughness_type == "fixed":
        max_height = cfg.noise_range[1]  # fixed roughness intensity
    else:
        raise ValueError(
            f"Invalid roughness type: {roughness_type}. Must be one of 'difficulty', 'random', or 'fixed'."
        )
    height_min = int(cfg.noise_range[0] / cfg.vertical_scale)
    height_max = int(max_height / cfg.vertical_scale)
    height_step = int(cfg.noise_step / cfg.vertical_scale)

    # create range of heights possible
    height_range = np.arange(height_min, height_max + height_step, height_step)
    # sample heights randomly from the range along a grid
    height_field_downsampled = np.random.choice(height_range, size=(width_downsampled, length_downsampled))
    # create interpolation function for the sampled heights
    x = np.linspace(0, cfg.size[0] * cfg.horizontal_scale, width_downsampled)
    y = np.linspace(0, cfg.size[1] * cfg.horizontal_scale, length_downsampled)
    func = interpolate.RectBivariateSpline(x, y, height_field_downsampled)

    # interpolate the sampled heights to obtain the height field
    x_upsampled = np.linspace(0, cfg.size[0] * cfg.horizontal_scale, width_pixels)
    y_upsampled = np.linspace(0, cfg.size[1] * cfg.horizontal_scale, length_pixels)
    z_upsampled = np.rint(func(x_upsampled, y_upsampled)).astype(np.int16)

    # Add noise to terrain
    hf_raw += z_upsampled

    return hf_raw


@height_field_to_mesh
def pyramid_sloped_rough_terrain(difficulty: float, cfg: hf_terrains_cfg.HfPyramidSlopedRoughTerrainCfg) -> np.ndarray:
    """Generate a terrain with a truncated pyramid structure with optional roughness.

    The terrain is a pyramid-shaped sloped surface with a slope of :obj:`slope` that trims into a flat platform
    at the center. The slope is defined as the ratio of the height change along the x axis to the width along the
    x axis. For example, a slope of 1.0 means that the height changes by 1 unit for every 1 unit of width.

    If the :obj:`cfg.inverted` flag is set to :obj:`True`, the terrain is inverted such that
    the platform is at the bottom.

    If :obj:`cfg.apply_roughness` is set to :obj:`True`, random noise is applied to the terrain surface.

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

    # Apply roughness if enabled
    if cfg.apply_roughness:
        hf_raw = apply_random_uniform_noise(cfg, hf_raw, difficulty)

    # round off the heights to the nearest vertical step
    return np.rint(hf_raw).astype(np.int16)


@height_field_to_mesh
def discrete_obstacles_terrain(
    difficulty: float, cfg: locolab_terrains_cfg.HfDiscreteObstaclesTerrainCfg
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

    # keep a flat spawn platform in the center.
    platform_width = min(platform_width, width_pixels, length_pixels)
    x1 = (width_pixels - platform_width) // 2
    x2 = (width_pixels + platform_width) // 2
    y1 = (length_pixels - platform_width) // 2
    y2 = (length_pixels + platform_width) // 2
    hf_raw[x1:x2, y1:y2] = 0

    if cfg.apply_roughness:
        hf_raw = apply_random_uniform_noise(cfg, hf_raw, difficulty)
        hf_raw[x1:x2, y1:y2] = 0

    return np.rint(hf_raw).astype(np.int16)
