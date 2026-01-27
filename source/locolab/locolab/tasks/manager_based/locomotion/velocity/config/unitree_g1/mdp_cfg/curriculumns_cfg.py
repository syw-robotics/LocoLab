# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

import locolab.tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.utils import configclass


@configclass
class FlatCurriculumsCfg:
    """Curriculum terms for flat terrain. Not added in g1 flat task"""

    lin_vel_cmd_levels = CurrTerm(
        mdp.lin_vel_cmd_levels,
        params={
            "max_lin_vel_x_ranges": (-1.0, 1.5),
            "max_lin_vel_y_ranges": (-1.0, 1.0),
            "percentage_threshold": 0.8,
        },
    )
    ang_vel_cmd_levels = CurrTerm(
        mdp.ang_vel_cmd_levels,
        params={
            "max_ang_vel_z_ranges": (-2.0, 2.0),
            "percentage_threshold": 0.5,
        },
    )
