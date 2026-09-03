"""Configuration classes for LocoLab mesh terrains."""

from __future__ import annotations

from dataclasses import MISSING
from typing import Literal

from isaaclab.terrains import SubTerrainBaseCfg
from isaaclab.utils import configclass

from . import locolab_mesh_terrains


@configclass
class MeshRepeatedBoxesTerrainCfg(SubTerrainBaseCfg):
    """Repeated boxes with independently randomized length and width."""

    function = locolab_mesh_terrains.mesh_random_size_repeated_boxes_terrain

    num_objects_range: tuple[int, int] = MISSING
    num_objects_type: Literal["random", "difficulty"] = "difficulty"
    box_height_range: tuple[float, float] = MISSING
    box_length_range: tuple[float, float] = MISSING
    box_width_range: tuple[float, float] = MISSING
    angle_range: tuple[float, float] = (0.0, 0.0)
    angle_type: Literal["random", "difficulty"] = "random"
    angle_degrees: bool = True
    platform_width: float = 1.0
    platform_height: float = -1.0
    abs_height_noise: tuple[float, float] = (0.0, 0.0)
    rel_height_noise: tuple[float, float] = (1.0, 1.0)


@configclass
class MeshRandomWidthPyramidStairsTerrainCfg(SubTerrainBaseCfg):
    """Configuration for pyramid stairs with a discretely sampled step width."""

    function = locolab_mesh_terrains.mesh_random_width_pyramid_stairs_terrain

    step_height_range: tuple[float, float] = MISSING
    """The minimum and maximum step height (in m). Scales with difficulty."""

    step_width_range: tuple[float, float] = MISSING
    """The minimum and maximum step width (in m)."""

    step_width_step: float = MISSING
    """The sampling increment for step width (in m)."""

    platform_width: float = 1.0
    """The width of the square platform at the center of the terrain (in m)."""

    border_width: float = 0.0
    """The width of the flat border around the terrain (in m)."""

    holes: bool = False
    """Whether to leave holes outside the pyramid stairs."""

    inverted: bool = False
    """Whether to generate inverted pyramid stairs."""


@configclass
class MeshStraightGapTerrainCfg(SubTerrainBaseCfg):
    """Configuration for a terrain with a gap in x direction."""

    function = locolab_mesh_terrains.mesh_straight_gap_terrain

    gap_width_range: tuple[float, float] = MISSING
    """The minimum and maximum width of the gap in x direction (in m). Scales with difficulty."""

    gap_depth_range: tuple[float, float] = (0.5, 1.0)
    """The minimum and maximum depth of the gap (in m). Randomly sampled."""

    gap_depth_type: Literal["difficulty", "random"] = "difficulty"
    """The type of gap depth formulation. Must be one of 'difficulty', 'random'"""

    gap_y_length_range: tuple[float, float] = (2.0, 6.0)
    """The minimum and maximum length of the gap in y direction (in m). Randomly sampled."""

    gap_x_offset_range: tuple[float, float] = (0.8, 2.0)
    """The minimum and maximum offset of gap position in x direction (in m). Randomly sampled."""

    platform_width: float = 1.0
    """The width of the platform in x direction (each side of the gap). Defaults to 1.0."""


@configclass
class MeshHurdleTerrainCfg(SubTerrainBaseCfg):
    """Rectangular hurdles generated directly as metric mesh boxes.

    This is the mesh counterpart to :class:`HfHurdleTerrainCfg`.  The HF
    version first rasterizes the layout onto horizontal and vertical grids and
    then converts that height field to a mesh, so its physical hurdle width and
    height can differ from the configured values by up to one sampling step.
    This version creates explicit ``trimesh`` boxes in meters and therefore
    preserves the configured collision dimensions exactly.  Keep the HF class
    when height-field sampling or roughness compatibility is required; use this
    class for contact-sensitive hurdle and parkour tasks.
    """

    function = locolab_mesh_terrains.mesh_hurdle_terrain

    hurdle_width_range: tuple[float, float] = MISSING
    """The minimum and maximum hurdle width in meters."""

    hurdle_height_range: tuple[float, float] = MISSING
    """The minimum and maximum hurdle height in meters."""

    platform_width_range: tuple[float, float] = MISSING
    """The minimum and maximum width of the center square platform in meters."""


@configclass
class MeshStraightStairsTerrainCfg(SubTerrainBaseCfg):
    """Configuration for stairs terrain (up then down) using mesh in x direction."""

    function = locolab_mesh_terrains.mesh_straight_stairs_terrain

    stair_width: float = MISSING
    """The fixed width of each stair in y direction (in m)."""

    stair_width_noise_range: tuple[float, float] = MISSING
    """The minimum and maximum noise for stair width (in m)."""

    stair_height_range: tuple[float, float] = MISSING
    """The minimum and maximum height of each stair (in m). Scales with difficulty."""

    stair_length_range: tuple[float, float] = MISSING
    """The minimum and maximum length of each stair in y direction (in m). Randomly sampled."""

    num_stairs_range: tuple[int, int] = MISSING
    """The minimum and maximum number of stairs for ascending and descending. Randomly sampled."""

    platform_width_range: tuple[float, float] = MISSING
    """The minimum and maximum width of the platform (in m)."""

    easy_difficulty_threshold: float = 0.2
    """Difficulty threshold below which stair length is maximized."""


@configclass
class MeshInvertedStraightStairsTerrainCfg(MeshStraightStairsTerrainCfg):
    """Configuration for stairs terrain (up then down) using mesh in x direction."""

    function = locolab_mesh_terrains.mesh_inverted_straight_stairs_terrain
