"""Configuration classes for LocoLab heightfield terrains."""

from __future__ import annotations

from dataclasses import MISSING
from typing import Literal

from isaaclab.terrains.height_field.hf_terrains_cfg import HfTerrainBaseCfg
from isaaclab.utils import configclass

from . import locolab_hf_terrains


@configclass
class RoughnessParamsCfg:
    """Shared parameters for terrain roughness."""

    noise_range: tuple[float, float] = (-0.02, 0.02)
    """The minimum and maximum height noise in meters."""

    noise_step: float = 0.01
    """The height increment used when sampling roughness in meters."""

    downsampled_scale: float | None = 0.20
    """Distance between sampled roughness points in meters."""

    roughness_type: Literal["difficulty", "random", "fixed"] = "fixed"
    """The roughness intensity mode."""


@configclass
class HfRoughTerrainCfg(RoughnessParamsCfg, HfTerrainBaseCfg):
    """Base configuration for height-field terrains with optional roughness."""

    apply_roughness: bool = False
    """Whether to apply random uniform roughness."""


@configclass
class HfPyramidSlopedRoughTerrainCfg(HfRoughTerrainCfg):
    """Configuration for a pyramid sloped terrain with optional roughness."""

    function = locolab_hf_terrains.pyramid_sloped_rough_terrain

    slope_range: tuple[float, float] = MISSING
    """The minimum and maximum slope."""

    platform_width: float = 1.0
    """The width of the square platform at the center of the terrain."""

    inverted: bool = False
    """Whether the slope is inverted."""


@configclass
class HfInvertedPyramidSlopedRoughTerrainCfg(HfPyramidSlopedRoughTerrainCfg):
    """Configuration for an inverted pyramid sloped terrain with optional roughness."""

    inverted: bool = True


@configclass
class HfDiscreteObstaclesTerrainCfg(HfRoughTerrainCfg):
    """Configuration for legged_gym-style discrete rectangular obstacles."""

    function = locolab_hf_terrains.discrete_obstacles_terrain

    obstacle_width_range: tuple[float, float] = MISSING
    """The minimum and maximum rectangle obstacle size in meters."""

    obstacle_height_range: tuple[float, float] = MISSING
    """The minimum and maximum obstacle height in meters."""

    num_obstacles: int = MISSING
    """The number of rectangular obstacles to generate."""

    platform_width: float = 1.0
    """The width of the square flat platform at the center of the terrain."""


@configclass
class HfGapTerrainCfg(HfRoughTerrainCfg):
    """Configuration for rectangular gap terrain."""

    function = locolab_hf_terrains.gap_terrain

    gap_width_range: tuple[float, float] = MISSING
    """The minimum and maximum gap width in meters."""

    gap_depth_range: tuple[float, float] = MISSING
    """ The minimum and maximum size of the gap depth in meters."""

    gap_depth_type: Literal["difficulty", "random"] = "difficulty"

    platform_width_range: tuple[float, float] = MISSING
    """The width of the square flat platform at the center of the terrain."""

    platform_height_range: tuple[float, float] = MISSING
    """The height of the square flat platform at the center of the terrain."""


@configclass
class HfDoubleGapTerrainCfg(HfRoughTerrainCfg):
    """Configuration for double rectangular gap terrain."""

    function = locolab_hf_terrains.double_gap_terrain

    gap_width_range: tuple[float, float] = MISSING
    """The minimum and maximum gap width in meters."""

    gap_depth_range: tuple[float, float] = MISSING
    """ The minimum and maximum size of the gap depth in meters."""

    gap_depth_type: Literal["difficulty", "random"] = "difficulty"
    """ The type of gap depth dormulation. Must be one of 'diffifulty' or 'random'"""

    gap_in_between_width_range: tuple[float, float] = MISSING
    """ The flat terrain width between two gaps in meters"""

    platform_width_range: tuple[float, float] = MISSING
    """The width of the square flat platform at the center of the terrain."""

    platform_height_range: tuple[float, float] = MISSING
    """The height of the square flat platform at the center of the terrain."""


@configclass
class HfStraightGapTerrainCfg(HfRoughTerrainCfg):
    """Configuration for straight gap terrain, gap only for x direaction."""

    function = locolab_hf_terrains.straight_gap_terrain

    gap_width_range: tuple[float, float] = MISSING
    """The minimum and maximum gap width in meters."""

    gap_depth_range: tuple[float, float] = MISSING
    """ The minimum and maximum size of the gap depth in meters."""

    gap_depth_type: Literal["difficulty", "random"] = "difficulty"
    """ The type of gap depth dormulation. Must be one of 'diffifulty' or 'random'"""

    gap_offset_range: tuple[float, float] = MISSING
    """Distance from terrain center to the inner edge of each gap along x, in meters."""

    platform_width_range: tuple[float, float] = MISSING
    """The width in y direction of the center platform at the center of the terrain."""

    platform_height_range: tuple[float, float] = MISSING
    """The height of the center platform at the center of the terrain."""

    easy_difficulty_threshold: float = 0.2
    """Difficulty threshold below which platform width is maximized."""


@configclass
class HfHurdleTerrainCfg(HfRoughTerrainCfg):
    """Configuration for rectangular hurdle terrain."""

    function = locolab_hf_terrains.hurdle_terrain

    hurdle_width_range: tuple[float, float] = MISSING
    """The minimum and maximum hurdle width in meters."""

    hurdle_height_range: tuple[float, float] = MISSING
    """ The minimum and maximum size of the hurdle height in meters."""

    platform_width_range: tuple[float, float] = MISSING
    """The width of the square flat platform at the center of the terrain."""
