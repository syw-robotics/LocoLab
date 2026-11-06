# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from isaaclab.utils import configclass

import locolab.tasks.manager_based.locomotion.tracking.mdp as mdp
from locolab.assets.robots.unitree.G1.g1_actuators import g1_action_scale

from . import JOINT_NAMES


@configclass
class ActionsCfg:
    """Choose joint position residuals around the default pose or motion reference."""

    joint_pos = mdp.TrackingJointPositionActionCfg(
        asset_name="robot",
        joint_names=JOINT_NAMES,
        scale=g1_action_scale,
        use_default_offset=True,
        use_reference_residual=False,
        preserve_order=True,
    )
