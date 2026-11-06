# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Shared native/browser configuration for velocity-push debug arrows."""

from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.utils import configclass

from locolab.utils.debug_viz.markers import BLUE_ARROW_X_MARKER_CFG


@configclass
class PushDebugVisCfg:
    """Draw the applied velocity increment from the pushed articulation's root link."""

    duration_s: float = 3.0
    """Simulation seconds over which opacity decreases linearly from one to zero."""

    visualizer_cfg: VisualizationMarkersCfg = BLUE_ARROW_X_MARKER_CFG.replace(prim_path="/Visuals/PushVelocity")
    """Arrow material and scale, shared by native and browser viewers."""

    # X controls meters per m/s; Y and Z control arrow thickness.
    visualizer_cfg.markers["arrow"].scale = (0.5, 0.05, 0.05)
