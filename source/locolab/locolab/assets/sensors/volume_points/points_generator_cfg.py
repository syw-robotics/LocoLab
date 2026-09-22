# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""
Reference: https://github.com/project-instinct/instinct_rl.git
"""

from dataclasses import MISSING
from typing import Callable

from isaaclab.utils import configclass

from .points_generator import grid3d_points_generator


@configclass
class PointsGeneratorCfg:
    """Specifying how the volume points are generated."""

    func: Callable = MISSING  # type: ignore


@configclass
class Grid3dPointsGeneratorCfg(PointsGeneratorCfg):
    func: Callable = grid3d_points_generator

    x_min: float = -1.0
    """Minimum x coordinate of the grid."""
    x_max: float = 1.0
    """Maximum x coordinate of the grid."""
    x_num: int = 10
    """Number of points along the x axis."""
    y_min: float = -1.0
    """Minimum y coordinate of the grid."""
    y_max: float = 1.0
    """Maximum y coordinate of the grid."""
    y_num: int = 10
    """Number of points along the y axis."""
    z_min: float = -1.0
    """Minimum z coordinate of the grid."""
    z_max: float = 1.0
    """Maximum z coordinate of the grid."""
    z_num: int = 10
    """Number of points along the z axis."""
