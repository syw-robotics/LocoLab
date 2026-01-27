# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

import math

import locolab.tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from . import (
    ARM_JOINT_NAMES,
    FOOT_LINK_NAMES,
    HIP_ROLL_JOINT_NAME,
    HIP_YAW_JOINT_NAME,
    UNDESIRED_CONTACT_LINK_NAMES,
    WAIST_JOINT_NAMES,
)


@configclass
class FlatRewardsCfg:
    """Reward terms for flat terrain."""

    # ===== task-specific rewards =====
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_world_exp, weight=1.0, params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )
    alive = RewTerm(func=mdp.is_alive, weight=0.1)

    # ===== penalty rewards =====
    # -- base --
    base_linear_velocity = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    base_angular_velocity = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-5.0)
    base_height = RewTerm(func=mdp.base_height_l2, weight=-10, params={"target_height": 0.78})
    # -- joint --
    joint_acc = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    joint_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-5.0)
    joint_power_l1 = RewTerm(func=mdp.joint_power_l1, weight=-2.0e-5)
    joint_vel_l2 = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-1.0e-3,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"])},
    )
    joint_deviation_arms_l1 = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.1,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=ARM_JOINT_NAMES)},
    )
    joint_deviation_waists_l1 = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=WAIST_JOINT_NAMES)},
    )
    joint_deviation_hip_roll_l1 = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.1,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=HIP_ROLL_JOINT_NAME)},
    )
    joint_deviation_hip_yaw_l1 = RewTerm(
        func=mdp.joint_deviation_humanoid_hip_yaw_l1,
        weight=-0.01,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=HIP_YAW_JOINT_NAME),
            "ang_vel_threshold": 0.1,
            "zero_ang_vel_command_weight_scale": 5.0,
        },
    )
    # -- action --
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    # -- collision --
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1,
        params={
            "threshold": 1.0,
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=UNDESIRED_CONTACT_LINK_NAMES),
        },
    )
    # -- feet --
    feet_gait = RewTerm(
        func=mdp.feet_gait,
        weight=0.5,
        params={
            "period": 0.8,
            "offset": [0.0, 0.5],
            "threshold": 0.55,
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_LINK_NAMES),
            "velocity_threshold": 0.2,
        },
    )
    feet_flat_contact = RewTerm(
        func=mdp.feet_flat_contact,
        weight=-0.5,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_LINK_NAMES),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_LINK_NAMES),
        },
    )
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.1,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_LINK_NAMES),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_LINK_NAMES),
        },
    )
    feet_clearance = RewTerm(
        func=mdp.feet_clearance,
        weight=1.0,
        params={
            "std": 0.05,
            "tanh_mult": 2.0,
            "target_height": 0.1,
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_LINK_NAMES),
        },
    )
