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
class RoughCurriculumsCfg:
    """Curriculum terms for rough terrain."""

    terrain_levels = CurrTerm(func=mdp.terrain_levels, params={"log_by_terrain_type": False})
