# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

import math

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import locolab.tasks.manager_based.locomotion.velocity.mdp as mdp

from . import (
    JOINT_NAMES,
    ARM_JOINT_NAMES,
    FOOT_LINK_NAMES,
    GAIT_PERIOD,
    HIP_ROLL_JOINT_NAME,
    HIP_YAW_JOINT_NAME,
    TORSO_LINK_NAME,
    WAIST_JOINT_NAMES,
)



@configclass
class FlatRewardsCfg:
    """Velocity tracking, torso stabilization and contact-driven biped walking."""

    # ===== task-specific rewards =====
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_world_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )
    #  termination_penalty = RewTerm(func=mdp.is_terminated, weight=-100.0)
    alive = RewTerm(func=mdp.is_alive, weight=0.1)

    # ===== penalty rewards =====
    # -- base --
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    ang_vel_xy_l2 = RewTerm(
        func=mdp.ang_vel_xy_l2,
        weight=-0.2,
        params={
            "select_bodies": True,
            "asset_cfg": SceneEntityCfg("robot", body_names=[TORSO_LINK_NAME]),
        },
    )
    pelvis_ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    torso_flat_orientation_l2 = RewTerm(
        func=mdp.flat_orientation_l2,
        weight=-5.0,
        params={
            "select_bodies": True,
            "asset_cfg": SceneEntityCfg("robot", body_names=[TORSO_LINK_NAME]),
        },
    )
    #  flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-1.0)
    base_height = RewTerm(func=mdp.base_height_l2, weight=-10.0, params={"target_height": 0.78})

    # -- joint --
    joint_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    #  joint_vel_l2 = RewTerm(func=mdp.joint_vel_l2, weight=-1.0e-3)
    joint_torques_l2 = RewTerm(
        func=mdp.joint_torques_l2,
        weight=-1.0e-4,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES)},
    )
    joint_power_l1 = RewTerm(func=mdp.joint_power_l1, weight=-2.5e-5)
    joint_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-5.0)
    joint_deviation_arms = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.1,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=ARM_JOINT_NAMES)},
    )
    joint_deviation_waists = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=WAIST_JOINT_NAMES)},
    )
    joint_deviation_hips = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.1,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[HIP_ROLL_JOINT_NAME, HIP_YAW_JOINT_NAME])},
    )
    stand_still = RewTerm(
        func=mdp.stand_still,
        weight=-1.0,
        params={"command_name": "base_velocity"},
    )
    # -- action --
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.05)
    #  action_smoothness_l2 = RewTerm(func=mdp.action_smoothness_l2, weight=-0.01)
    # -- feet --
    #  feet_standing_contact = RewTerm(
    #      func=mdp.feet_standing_contact,
    #      weight=-0.5,
    #      params={
    #          "command_name": "base_velocity",
    #          "sensor_cfg": SceneEntityCfg("feet_contact_forces", body_names=FOOT_LINK_NAMES),
    #      },
    #  )
    feet_flight = RewTerm(
        func=mdp.feet_flight,
        weight=-1.0,
        params={"sensor_cfg": SceneEntityCfg("feet_contact_forces", body_names=FOOT_LINK_NAMES)},
    )
    feet_short_swing = RewTerm(
        func=mdp.feet_short_swing,
        weight=-0.5,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("feet_contact_forces", body_names=FOOT_LINK_NAMES),
            "min_air_time": 0.20,
        },
    )
    feet_air_time = RewTerm(
        func=mdp.feet_air_time,
        weight=1.0,
        params={
            "command_name": "base_velocity",
            "threshold": 0.40,
            "sensor_cfg": SceneEntityCfg("feet_contact_forces", body_names=FOOT_LINK_NAMES),
        },
    )
    feet_gait = RewTerm(
        func=mdp.feet_gait,
        weight=0.5,
        params={
            "period": GAIT_PERIOD,
            "offset": [0.0, 0.5],
            "threshold": 0.55,
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("feet_contact_forces", body_names=FOOT_LINK_NAMES),
        },
    )
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.2,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_LINK_NAMES),
            "sensor_cfg": SceneEntityCfg("feet_contact_forces", body_names=FOOT_LINK_NAMES),
        },
    )
    feet_clearance_flat = RewTerm(
        func=mdp.feet_clearance_flat,
        weight=1.0,
        params={
            "std": 0.05,
            "tanh_mult": 2.0,
            "target_height": 0.12,
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_LINK_NAMES),
        },
    )
    #  feet_too_near = RewTerm(
    #      func=mdp.feet_too_near_bipedal,
    #      weight=-1.0,
    #      params={
    #          "threshold": 0.15,
    #          "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_LINK_NAMES),
    #      },
    #  )

    # -- collision --
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1.0,
        params={
            "threshold": 1.0,
            "sensor_cfg": SceneEntityCfg("self_collision_contact_forces", body_names=["(?!.*ankle.*).*"]),
        },
    )


@configclass
class RoughRewardsCfg:
    pass
