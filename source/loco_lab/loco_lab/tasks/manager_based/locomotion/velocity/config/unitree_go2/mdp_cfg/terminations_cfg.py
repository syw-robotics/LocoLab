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

import loco_lab.tasks.manager_based.locomotion.velocity.mdp as mdp

from . import BASE_LINK_NAME


@configclass
class FlatTerminationsCfg:
    """Termination terms for the flat terrain."""

    # terminate when max episode length is reached
    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    # bad_orientation is a substitute to illegal_contact, in case of not training fall-recovery behavior
    #  bad_orientation = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": 0.8})

    # terminate when contact happens on specified links
    illegal_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=BASE_LINK_NAME),
            "threshold": 1.0,
        },
    )


@configclass
class RoughTerminationsCfg:
    """Termination terms for the rough terrain."""

    # terminate when max episode length is reached
    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    # terrain_out_of_bounds is needed for rough terrain
    #  terrain_out_of_bounds = DoneTerm(
    #      func=mdp.terrain_out_of_bounds,
    #      params={"asset_cfg": SceneEntityCfg("robot"), "distance_buffer": 3.0},
    #      time_out=True,
    #  )

    # bad_orientation is a substitute to illegal_contact, in case of not training fall-recovery behavior
    #  bad_orientation = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": 1.5})  # about 80 degrees

    # terminate when contact happens on specified links
    illegal_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=BASE_LINK_NAME),
            "threshold": 1.0,
        },
    )
