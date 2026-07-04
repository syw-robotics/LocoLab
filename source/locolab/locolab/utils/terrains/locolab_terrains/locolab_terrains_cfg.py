"""Configuration classes for LocoLab custom terrains."""

from __future__ import annotations

from dataclasses import MISSING
from typing import Literal

from isaaclab.terrains.height_field.hf_terrains_cfg import HfTerrainBaseCfg
from isaaclab.utils import configclass

from . import locolab_terrains


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

    function = locolab_terrains.pyramid_sloped_rough_terrain

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

    function = locolab_terrains.discrete_obstacles_terrain

    obstacle_width_range: tuple[float, float] = MISSING
    """The minimum and maximum rectangle obstacle size in meters."""

    obstacle_height_range: tuple[float, float] = MISSING
    """The minimum and maximum obstacle height in meters."""

    num_obstacles: int = MISSING
    """The number of rectangular obstacles to generate."""

    platform_width: float = 1.0
    """The width of the square flat platform at the center of the terrain."""
