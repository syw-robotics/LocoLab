# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

import locolab.tasks.manager_based.locomotion.velocity.mdp as mdp

from . import BASE_LINK_NAME


@configclass
class FlatTerminationsCfg:
    """Termination terms for the flat terrain."""

    # terminate when max episode length is reached
    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    # bad_orientation is a substitute to illegal_contact, in case of not training fall-recovery behavior
    bad_orientation = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": 1.0})


@configclass
class RoughTerminationsCfg:
    """Termination terms for the rough terrain."""

    # terminate when max episode length is reached
    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    # terminate when contact happens on specified links
    illegal_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=BASE_LINK_NAME),
            "threshold": 1.0,
        },
    )
