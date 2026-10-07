"""Configuration classes for LocoLab heightfield terrains."""

from __future__ import annotations

from dataclasses import MISSING
from typing import Literal

from isaaclab.terrains.height_field.hf_terrains_cfg import HfTerrainBaseCfg
from isaaclab.utils import configclass

from . import locolab_hf_terrains


@configclass
class RoughnessParamsCfg:
    """Shared parameters for terrain roughness.

    Used by both height-field terrains and mesh terrains that mix in these fields.
    """

    noise_range: tuple[float, float] = (-0.02, 0.02)
    """The minimum and maximum height noise in meters."""

    noise_step: float = 0.01
    """The height increment used when sampling roughness in meters."""

    downsampled_scale: float | None = 0.20
    """Minimum distance between sampled roughness points in meters.

    The effective distance is sampled over one octave for each sub-terrain.
    """

    roughness_type: Literal["difficulty", "random", "fixed"] = "fixed"
    """The roughness intensity mode.

    Random mode samples uniformly from :attr:`roughness_strengths`.
    """

    roughness_strengths: tuple[float, ...] = (0.2, 0.4, 0.6, 0.8, 1.0)
    """Discrete roughness strengths sampled when :attr:`roughness_type` is ``"random"``."""


@configclass
class PoleParamsCfg:
    """Shared parameters for optional obstacle poles.

    Used by both height-field and mesh terrains that mix in these fields.
    Each pole is independently a cylinder or a square prism. Centers are
    sampled uniformly inside the sub-terrain.
    """

    apply_poles: float = 0.0
    """Probability of adding poles to a generated sub-terrain. Must be within [0, 1]."""

    num_poles_range: tuple[int, int] = (2, 6)
    """Inclusive range. The number of poles is sampled uniformly from this interval."""

    cylinder_radius_range: tuple[float, float] = (0.05, 0.15)
    """Cylinder radius in meters, sampled uniformly per cylindrical pole."""

    box_side_range: tuple[float, float] = (0.10, 0.20)
    """Square-prism side length in meters, sampled uniformly per box pole."""

    cylinder_probability: float = 0.5
    """Probability that a pole is a cylinder. The rest are square prisms."""

    pole_height_range: tuple[float, float] = (1.0, 2.0)
    """Pole height above the local surface, in meters. Sampled uniformly per pole."""

    min_pole_separation: float = 1.0
    """Minimum center-to-center distance between poles, in meters."""

    pole_edge_margin: float = 0.0
    """Keep pole footprints this far inside the sub-terrain border, in meters."""

    keep_center_clear: float = 1.0
    """Keep poles this far from the tile center, in meters. Use 0 to allow center poles."""


@configclass
class HfRoughTerrainCfg(RoughnessParamsCfg, PoleParamsCfg, HfTerrainBaseCfg):
    """Base configuration for height-field terrains with optional roughness and poles."""

    apply_roughness: float = 0.0
    """Probability of applying roughness to a generated sub-terrain. Must be within [0, 1]."""


@configclass
class HfFlatRoughTerrainCfg(HfRoughTerrainCfg):
    """Configuration for flat terrain with LocoLab fractal Perlin roughness."""

    function = locolab_hf_terrains.flat_rough_terrain

    apply_roughness: float = 1.0
    """Probability of applying roughness. Defaults to always enabled for this terrain type."""


@configclass
class HfPyramidSlopedRoughTerrainCfg(HfRoughTerrainCfg):
    """Configuration for a pyramid sloped terrain with optional roughness."""

    function = locolab_hf_terrains.pyramid_sloped_rough_terrain

    slope_range: tuple[float, float] = MISSING
    """The minimum and maximum slope."""

    center_platform_width: float = 1.0
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

    center_platform_width: float = 1.0
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

    center_platform_width_range: tuple[float, float] = MISSING
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

    center_platform_width_range: tuple[float, float] = MISSING
    """The width of the square flat platform at the center of the terrain."""

    platform_height_range: tuple[float, float] = MISSING
    """The height of the square flat platform at the center of the terrain."""


@configclass
class HfStraightGapTerrainCfg(HfRoughTerrainCfg):
    """Straight corridor along x with a gap sequence on each side.

    Each side packs as many gaps as fit at the difficulty-scaled gap width.
    Consecutive gaps are separated by an island. The center platform width is
    sampled along x. A side with two gaps looks like::

        landing | gap | island | gap | center

    Sampled lengths are not the collision size. Gap width, platform width, island
    x-width, y spans, and lateral offsets are truncated with ``int()`` to an integer
    number of :attr:`horizontal_scale` cells; the realized length is that cell count
    times :attr:`horizontal_scale`. Gap depth is truncated the same way onto
    :attr:`vertical_scale`.
    """

    function = locolab_hf_terrains.straight_gap_terrain

    gap_width_range: tuple[float, float] = MISSING
    """The minimum and maximum gap width in meters. Scales with difficulty.

    Height-field sampling truncates this width with ``int(width / horizontal_scale)``
    and keeps at least one cell. The realized width is that cell count times
    :attr:`horizontal_scale`.
    """

    gap_depth_range: tuple[float, float] = MISSING
    """The minimum and maximum gap depth in meters."""

    gap_depth_type: Literal["difficulty", "random"] = "difficulty"
    """How gap depth is sampled. Must be ``"difficulty"`` or ``"random"``."""

    center_platform_width_range: tuple[float, float] = MISSING
    """The center platform width along x, in meters."""

    island_width_range: tuple[float, float] = (0.5, 1.5)
    """Island size in meters. Each island samples x and y independently from this range.

    The minimum is a hard lower bound on each island's x-width. Height-field cells
    round up so the painted width does not fall short of it. Landings and the
    center platform also sample their y-width from this range.
    Each x-edge landing uses half of a sample from this range along x, so two
    neighboring gap tiles join into about one island width.
    """

    island_y_offset_range: tuple[float, float] = (0.0, 0.0)
    """Lateral offset of each island center relative to the corridor, in meters.

    Sampled independently per island. Positive is +y. Landings and the center
    platform stay on the corridor. Unused when that side has only one gap.
    """

    island_height_offset_range: tuple[float, float] = (0.0, 0.0)
    """Height of each island relative to the landings and center, in meters.

    Sampled independently per island. Unused when that side has only one gap.
    """


@configclass
class HfHurdleTerrainCfg(HfRoughTerrainCfg):
    """Configuration for rectangular hurdle terrain."""

    function = locolab_hf_terrains.hurdle_terrain

    hurdle_width_range: tuple[float, float] = MISSING
    """The minimum and maximum hurdle width in meters."""

    hurdle_height_range: tuple[float, float] = MISSING
    """ The minimum and maximum size of the hurdle height in meters."""

    center_platform_width_range: tuple[float, float] = MISSING
    """The width of the square flat platform at the center of the terrain."""


@configclass
class HfPyramidStairsTerrainCfg(HfRoughTerrainCfg):
    """Configuration for a pyramid stairs height field terrain."""

    function = locolab_hf_terrains.pyramid_stairs_terrain

    stair_height_range: tuple[float, float] = MISSING
    """The minimum and maximum stair height (in m)."""

    stair_width: float = MISSING
    """The nominal tread width (in m).

    Height-field sampling snaps this width, including :attr:`stair_width_noise_range`,
    down to an integer number of :attr:`horizontal_scale` cells. The realized width
    is that cell count times :attr:`horizontal_scale`, and it is at least one cell.
    """

    stair_width_noise_range: tuple[float, float] = (0.0, 0.0)
    """Uniform noise added to :attr:`stair_width` for each sub-terrain (in m)."""

    center_platform_width: float = 1.0
    """The width of the square platform at the center of the terrain. Defaults to 1.0.

    Leftover length after fitting equal treads stays as a flat outer border so the
    center does not shrink below this size.
    """

    inverted: bool = False
    """Whether the pyramid stairs is inverted. Defaults to False.

    If True, the terrain is inverted such that the platform is at the bottom and the stairs are upwards.
    """


@configclass
class HfInvertedPyramidStairsTerrainCfg(HfPyramidStairsTerrainCfg):
    """Configuration for an inverted pyramid stairs height field terrain.

    Note:
        This is a subclass of :class:`HfPyramidStairsTerrainCfg` with :obj:`inverted` set to True.
        We make it as a separate class to make it easier to distinguish between the two and match
        the naming convention of the other terrains.
    """

    inverted: bool = True
