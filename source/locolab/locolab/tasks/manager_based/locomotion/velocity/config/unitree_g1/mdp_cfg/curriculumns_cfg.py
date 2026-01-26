# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.utils import configclass

import locolab.tasks.manager_based.locomotion.velocity.mdp as mdp


@configclass
class CurriculumsCfg:
    """Curriculum terms for rough terrain."""

    terrain_levels = CurrTerm(func=mdp.terrain_levels)
    lin_vel_cmd_levels = CurrTerm(mdp.lin_vel_cmd_levels,params={
        "max_lin_vel_x_ranges": (-0.5, 1.0),
        "max_lin_vel_y_ranges": (-0.3, 0.3),
    })
    # ang_vel_cmd_levels = CurrTerm(mdp.ang_vel_cmd_levels,params={
    #     "max_ang_vel_z_ranges": (-0.2, 0.2),
    # })
