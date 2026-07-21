"""Configuration classes for LocoLab mesh terrains."""

from __future__ import annotations

from dataclasses import MISSING
from typing import Literal

from isaaclab.terrains import SubTerrainBaseCfg
from isaaclab.utils import configclass

from . import locolab_mesh_terrains


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
