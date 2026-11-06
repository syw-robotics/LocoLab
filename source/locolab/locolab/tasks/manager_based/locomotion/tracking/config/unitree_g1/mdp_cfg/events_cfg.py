# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import locolab.tasks.manager_based.locomotion.tracking.mdp as mdp

from . import FOOT_LINK_NAMES, JOINT_NAMES, TORSO_LINK_NAME, VELOCITY_RANGE


@configclass
class EventCfg:
    """Configuration for events."""

    # ===== startup =====
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_LINK_NAMES),
            "static_friction_range": (0.3, 1.2),
            "dynamic_friction_range": (0.3, 1.2),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
            "make_consistent": True,
        },
    )
    randomize_joint_encoder_bias = EventTerm(
        func=mdp.randomize_encoder_bias,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=True),
            "bias_range": (-0.01, 0.01),
        },
    )
    randomize_base_com = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=TORSO_LINK_NAME),
            "com_range": {"x": (-0.025, 0.025), "y": (-0.05, 0.05), "z": (-0.05, 0.05)},
        },
    )

    # ===== interval =====
    push_robot = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(1.0, 3.0),
        params={"velocity_range": VELOCITY_RANGE, "debug_vis_cfg": mdp.PushDebugVisCfg()},
    )
