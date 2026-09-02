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
    ARM_JOINT_NAMES,
    ARM_JOINT_NAMES_WO_SHOULDER_PITCH,
    ARM_SHOULDER_PITCH_JOINT_NAME,
    COORDINATION_JOINT_NAMES,
    FOOT_LINK_NAMES,
    HIP_ROLL_JOINT_NAME,
    HIP_YAW_JOINT_NAME,
    TORSO_LINK_NAME,
    PELVIS_LINK_NAME,
    UNDESIRED_CONTACT_LINK_NAMES,
    WAIST_JOINT_NAMES,
)


@configclass
class FlatRewardsCfg:
    """V1 reward terms for the pelvis-base G1 task."""

    # ===== task-specific rewards =====
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_world_exp,
        weight=0.5,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )
    alive = RewTerm(func=mdp.is_alive, weight=0.1)

    # ===== penalty rewards =====
    # -- base --
    lin_vel_z_l2 = RewTerm(
        func=mdp.lin_vel_z_l2,
        weight=-2.0,
        params={
            "select_bodies": True,
            "asset_cfg": SceneEntityCfg("robot", body_names=[TORSO_LINK_NAME])},
    )
    ang_vel_xy_l2 = RewTerm(
        func=mdp.ang_vel_xy_l2,
        weight=-0.1,
        params={
            "select_bodies": True,
            "asset_cfg": SceneEntityCfg("robot", body_names=[TORSO_LINK_NAME])},
    )
    flat_orientation_l2 = RewTerm(
        func=mdp.flat_orientation_l2,
        weight=-10.0,
        params={
            "select_bodies": True,
            "asset_cfg": SceneEntityCfg("robot", body_names=[TORSO_LINK_NAME, PELVIS_LINK_NAME]),
        },
    )
    base_height = RewTerm(func=mdp.base_height_l2, weight=-5.0, params={"target_height": 0.76})
    # -- joint --
    joint_acc = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    joint_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-10.0)
    joint_power_l1 = RewTerm(func=mdp.joint_power_l1, weight=-2.0e-5)
    joint_vel_l2 = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-1.0e-3,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"])},
    )
    joint_deviation_arms_l1 = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=ARM_JOINT_NAMES_WO_SHOULDER_PITCH)},
    )
    joint_deviation_waists_l1 = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=WAIST_JOINT_NAMES)},
    )
    joint_deviation_hips_l1 = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[HIP_ROLL_JOINT_NAME, HIP_YAW_JOINT_NAME, ARM_SHOULDER_PITCH_JOINT_NAME])},
    )
    # penalize opposing hip and contralateral shoulder pitch directions
    joint_paired_coordination = RewTerm(
        func=mdp.joint_paired_position_coordination,
        weight=-0.05,
        params={
            "target_scale": 0.8,
            "error_scale": 0.10,
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=COORDINATION_JOINT_NAMES,
                preserve_order=True,
            ),
        },
    )
    # -- action --
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.05)
    # -- collision --
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1.0,
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
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_LINK_NAMES),
        },
    )
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.2,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_LINK_NAMES),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_LINK_NAMES),
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
    feet_clearance_flat = RewTerm(
        func=mdp.feet_clearance_flat,
        weight=1.0,
        params={
            "std": 0.05,
            "tanh_mult": 2.0,
            "target_height": 0.15,
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_LINK_NAMES),
        },
    )
    #  feet_air_time = RewTerm(
    #      func=mdp.feet_air_time,
    #      weight=0.5,
    #      params={
    #          "command_name": "base_velocity",
    #          "threshold": 0.3,
    #          "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_LINK_NAMES),
    #      },
    #  )


@configclass
class RoughRewardsCfg:
    pass
