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
class FlatCurriculumsCfg:
    """Curriculum terms for flat terrain."""

    lin_vel_cmd_levels = CurrTerm(
        mdp.lin_vel_cmd_levels,
        params={
            "max_lin_vel_x_ranges": (-0.5, 1.0),
            "max_lin_vel_y_ranges": (-0.5, 0.5),
            "percentage_threshold": 0.75,
            "velocity_step": 0.1,
        },
    )
    ang_vel_cmd_levels = CurrTerm(
        mdp.ang_vel_cmd_levels,
        params={
            "max_ang_vel_z_ranges": (-1.5, 1.5),
            "percentage_threshold": 0.5,
            "velocity_step": 0.1,
        },
    )
    push_robot_levels = CurrTerm(
        mdp.push_robot_velocity_levels,
        params={
            "max_push_velocity_ranges": {"x": (-1.5, 1.5), "y": (-1.0, 1.0)},
            "percentage_threshold": 0.75,
            "velocity_step": 0.1,
        },
    )


@configclass
class RoughCurriculumsCfg:
    """Curriculum terms for rough terrain."""

    lin_vel_cmd_levels = CurrTerm(
        mdp.lin_vel_cmd_levels,
        params={
            "max_lin_vel_x_ranges": (-0.5, 1.0),
            "max_lin_vel_y_ranges": (-0.5, 0.5),
            "percentage_threshold": 0.7,
            "velocity_step": 0.1,
        },
    )
    ang_vel_cmd_levels = CurrTerm(
        mdp.ang_vel_cmd_levels,
        params={
            "max_ang_vel_z_ranges": (-1.5, 1.5),
            "percentage_threshold": 0.5,
            "velocity_step": 0.1,
        },
    )
    push_robot_levels = CurrTerm(
        mdp.push_robot_velocity_levels,
        params={
            "max_push_velocity_ranges": {"x": (-1.5, 1.5), "y": (-1.0, 1.0)},
            "percentage_threshold": 0.7,
            "velocity_step": 0.1,
        },
    )
