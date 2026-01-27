# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.


import math

import locolab.tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab.utils import configclass


@configclass
class CommandsCfg:
    """Command specifications for the MDP."""

    #  base_velocity = mdp.UniformVelocityCommandCfg(
    #      asset_name="robot",
    #      resampling_time_range=(10.0, 10.0),
    #      velocity_threshold=0.2,
    #      heading_command=False,
    #      debug_vis=True,
    #      ranges=mdp.UniformVelocityCommandCfg.Ranges(
    #          lin_vel_x=(-0.1, 0.1),
    #          lin_vel_y=(-0.1, 0.1),
    #          ang_vel_z=(-0.5, 0.5),
    #      ),
    #      vel_visualizer_offset_z=0.7,
    #  )
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(10.0, 10.0),
        velocity_threshold=0.2,
        heading_command=True,
        rel_heading_envs=0.5,
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-0.5, 1.0),
            lin_vel_y=(-0.5, 0.5),
            ang_vel_z=(-1.5, 1.5),
            heading=(-math.pi, math.pi),
        ),
        vel_visualizer_offset_z=0.7,
    )
