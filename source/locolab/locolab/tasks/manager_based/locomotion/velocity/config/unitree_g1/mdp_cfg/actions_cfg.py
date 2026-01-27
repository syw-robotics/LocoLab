# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

import locolab.tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab.utils import configclass
from locolab.assets.actuators import beyondmimic_action_scale

from . import JOINT_NAMES, PRESERVE_ORDER


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=JOINT_NAMES,
        scale=beyondmimic_action_scale,
        use_default_offset=True,
        clip={".*": (-10.0, 10.0)},
        preserve_order=PRESERVE_ORDER,
    )
