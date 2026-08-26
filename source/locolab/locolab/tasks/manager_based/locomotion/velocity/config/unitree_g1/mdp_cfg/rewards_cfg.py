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
    ANKLE_BODY_NAMES,
    ARM_JOINT_NAMES,
    CORE_BODY_NAMES,
    ELBOW_JOINT_NAMES,
    ELBOW_WRIST_JOINT_NAMES,
    FOOT_BODY_NAMES,
    FOOT_LINK_NAMES,
    HAND_BODY_NAMES,
    HIP_BODY_NAMES,
    HIP_ROLL_JOINT_NAME,
    HIP_YAW_JOINT_NAME,
    KNEE_BODY_NAMES,
    ROOT_LINK_NAME,
    TORSO_LINK_NAME,
    UNDESIRED_CONTACT_LINK_NAMES,
    WAIST_JOINT_NAMES,
    WRIST_JOINT_NAMES,
)


CORE_BODY_INERTIA_DIAG = (
    (0.0232719, 0.0180057, 0.0144003),
    (0.000360867, 0.000232085, 0.000183698),
    (4.8e-05, 2.953e-05, 2.952e-05),
    (0.221461, 0.180271, 0.120801),
)


@configclass
class FlatRewardsCfg:
    """Reward terms for flat terrain."""

    # ===== task-specific rewards =====
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=1.5,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_world_exp, weight=1.0, params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )
    alive = RewTerm(func=mdp.is_alive, weight=0.1)

    # ===== penalty rewards =====
    # -- base --
    lin_vel_z_l2 = RewTerm(
        func=mdp.lin_vel_z_body_l2,
        weight=-2.0, 
        params={"asset_cfg": SceneEntityCfg("robot", body_names=TORSO_LINK_NAME)},
    )
    ang_vel_xy_l2 = RewTerm(
        func=mdp.ang_vel_xy_body_l2,
        weight=-0.05,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=TORSO_LINK_NAME)},
    )
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-5.0)
    base_height = RewTerm(func=mdp.base_height_l2, weight=-10, params={"target_height": 0.76})
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
        weight=-0.5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=ARM_JOINT_NAMES)},
    )
    joint_deviation_waists_l1 = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-2.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=WAIST_JOINT_NAMES)},
    )
    joint_deviation_hips_l1 = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[HIP_ROLL_JOINT_NAME, HIP_YAW_JOINT_NAME])},
    )
    # -- action --
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.1)
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
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.2,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_LINK_NAMES),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_LINK_NAMES),
        },
    )
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


@configclass
class FlatAmpRewardsCfg:
    """Reward terms for G1 flat AMP training."""

    # ===== task-specific rewards =====
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=3.0,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_world_exp,
        weight=2.0,
        params={"command_name": "base_velocity", "std": math.sqrt(0.25)},
    )
    alive = RewTerm(func=mdp.is_alive, weight=0.1)

    # ===== penalty rewards =====
    lin_vel_z_l2 = RewTerm(
        func=mdp.lin_vel_z_body_l2,
        weight=-2.0,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=TORSO_LINK_NAME)},
    )
    ang_vel_xy_l2 = RewTerm(
        func=mdp.ang_vel_xy_body_l2,
        weight=-0.05,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=TORSO_LINK_NAME)},
    )
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=-5.0)

    joint_acc = RewTerm(func=mdp.joint_acc_l2, weight=-5.0e-8)
    joint_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=-10.0)
    joint_power_l1 = RewTerm(func=mdp.joint_power_l1, weight=-2.0e-5)
    joint_vel_l2 = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-1.0e-3,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*"])},
    )
    joint_deviation_arms_l1 = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=ARM_JOINT_NAMES)},
    )
    joint_deviation_waists_l1 = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-2.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=WAIST_JOINT_NAMES)},
    )
    joint_deviation_hips_l1 = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.5,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[HIP_ROLL_JOINT_NAME, HIP_YAW_JOINT_NAME])},
    )

    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-0.01)

    root_height_below_target_l2 = RewTerm(
        func=mdp.root_height_below_target_l2,
        weight=-1.5,
        params={"std": 0.18, "target_margin": 0.09},
    )
    speed_gated_support_center = RewTerm(
        func=mdp.speed_gated_support_center,
        weight=0.1,
        params={
            "command_name": "base_velocity",
            "std": 0.18,
            "forward_speed_scale": 2.0,
            "lateral_speed_scale": 0.8,
            "max_forward_offset": 0.14,
            "max_lateral_offset": 0.08,
            "body_cfg": SceneEntityCfg("robot", body_names=TORSO_LINK_NAME),
            "feet_body_cfg": SceneEntityCfg("robot", body_names=FOOT_BODY_NAMES, preserve_order=True),
        },
    )
    speed_gated_knee_alignment = RewTerm(
        func=mdp.speed_gated_knee_alignment,
        weight=0.18,
        params={
            "command_name": "base_velocity",
            "std": 0.055,
            "command_threshold": 0.25,
            "forward_speed_scale": 1.0,
            "side_speed_scale": 0.45,
            "turn_speed_scale": 0.7,
            "body_cfg": SceneEntityCfg("robot", body_names=TORSO_LINK_NAME),
            "hip_body_cfg": SceneEntityCfg("robot", body_names=HIP_BODY_NAMES, preserve_order=True),
            "knee_body_cfg": SceneEntityCfg("robot", body_names=KNEE_BODY_NAMES, preserve_order=True),
            "ankle_body_cfg": SceneEntityCfg("robot", body_names=ANKLE_BODY_NAMES, preserve_order=True),
        },
    )

    self_collisions = RewTerm(
        func=mdp.self_collisions,
        weight=-0.1,
        params={
            "force_threshold": 10.0,
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=UNDESIRED_CONTACT_LINK_NAMES),
        },
    )

    feet_air_time = RewTerm(
        func=mdp.feet_air_time,
        weight=2.5,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_LINK_NAMES),
            "threshold": 0.45,
        },
    )
    stand_still_contacts = RewTerm(
        func=mdp.stand_still_contacts,
        weight=-1.0,
        params={
            "lin_command_threshold": 0.08,
            "ang_command_threshold": 0.08,
            "body_velocity_threshold": 0.12,
            "use_body_velocity_gate": False,
            "asset_cfg": SceneEntityCfg("robot"),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_LINK_NAMES),
        },
    )
    speed_gated_foot_heading = RewTerm(
        func=mdp.speed_gated_foot_heading,
        weight=0.10,
        params={
            "command_name": "base_velocity",
            "std": 0.35,
            "command_threshold": 0.25,
            "forward_speed_scale": 1.0,
            "side_speed_scale": 0.45,
            "turn_speed_scale": 0.7,
            "use_command_gate": False,
            "left_toe_out_yaw": 0.03,
            "right_toe_out_yaw": -0.03,
            "body_cfg": SceneEntityCfg("robot", body_names=TORSO_LINK_NAME),
            "feet_body_cfg": SceneEntityCfg("robot", body_names=FOOT_BODY_NAMES, preserve_order=True),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_BODY_NAMES, preserve_order=True),
        },
    )
    speed_accel_gated_foot_contact_force = RewTerm(
        func=mdp.speed_accel_gated_foot_contact_force,
        weight=-0.6,
        params={
            "command_name": "base_velocity",
            "base_force": 300.0,
            "speed_force_scale": 220.0,
            "robot_acc_force_scale": 55.0,
            "cmd_acc_force_scale": 80.0,
            "force_normalizer": 450.0,
            "robot_acc_scale": 8.0,
            "cmd_acc_scale": 3.0,
            "body_cfg": SceneEntityCfg("robot", body_names=TORSO_LINK_NAME),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_BODY_NAMES, preserve_order=True),
        },
    )
    foot_contact_force_spike = RewTerm(
        func=mdp.foot_contact_force_spike,
        weight=-0.3,
        params={
            "command_name": "base_velocity",
            "base_peak_force": 1100.0,
            "speed_peak_force_scale": 260.0,
            "robot_acc_peak_force_scale": 80.0,
            "cmd_acc_peak_force_scale": 120.0,
            "base_delta_force": 500.0,
            "robot_acc_delta_force_scale": 40.0,
            "cmd_acc_delta_force_scale": 60.0,
            "peak_normalizer": 1000.0,
            "delta_normalizer": 800.0,
            "peak_weight": 0.5,
            "delta_weight": 1.0,
            "body_cfg": SceneEntityCfg("robot", body_names=TORSO_LINK_NAME),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_BODY_NAMES, preserve_order=True),
        },
    )

    hand_leg_sagittal_antiphase = RewTerm(
        func=mdp.speed_gated_arm_leg_sagittal_antiphase_vel,
        weight=0.1,
        params={
            "command_name": "base_velocity",
            "std": 0.15,
            "command_threshold": 0.2,
            "forward_speed_scale": 1.0,
            "side_speed_scale": 0.45,
            "turn_speed_scale": 0.7,
            "arm_lateral_vel_std": 0.25,
            "arm_vertical_vel_std": 0.5,
            "arm_lateral_vel_cost_scale": 0.6,
            "arm_vertical_vel_cost_scale": 0.15,
            "arm_body_cfg": SceneEntityCfg("robot", body_names=HAND_BODY_NAMES, preserve_order=True),
            "leg_body_cfg": SceneEntityCfg("robot", body_names=ANKLE_BODY_NAMES, preserve_order=True),
            "anchor_cfg": SceneEntityCfg("robot", body_names=TORSO_LINK_NAME),
        },
    )

    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.4,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_LINK_NAMES),
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_LINK_NAMES),
        },
    )
    hand_rel_z_l2_cost = RewTerm(
        func=mdp.body_relative_z_l2_cost,
        weight=-0.2,
        params={
            "target_z": -0.205,
            "std": 0.1,
            "body_cfg": SceneEntityCfg("robot", body_names=HAND_BODY_NAMES, preserve_order=True),
            "anchor_cfg": SceneEntityCfg("robot", body_names=ROOT_LINK_NAME),
        },
    )
    hand_side_height_l2_cost = RewTerm(
        func=mdp.body_side_height_l2_cost,
        weight=-0.12,
        params={
            "target_z": -0.22,
            "z_std": 0.1,
            "min_lateral_abs_y": 0.12,
            "y_std": 0.08,
            "max_lateral_abs_y": 0.35,
            "max_y_std": 0.08,
            "body_cfg": SceneEntityCfg("robot", body_names=HAND_BODY_NAMES, preserve_order=True),
            "anchor_cfg": SceneEntityCfg("robot", body_names=ROOT_LINK_NAME),
        },
    )
    hand_rel_acc_l2_cost = RewTerm(
        func=mdp.body_relative_lin_acc_l2_cost,
        weight=-0.05,
        params={
            "std": 10.0,
            "body_cfg": SceneEntityCfg("robot", body_names=HAND_BODY_NAMES, preserve_order=True),
            "anchor_cfg": SceneEntityCfg("robot", body_names=ROOT_LINK_NAME),
        },
    )
    elbow_wrist_joint_vel_hinge_l2_cost = RewTerm(
        func=mdp.joint_vel_hinge_l2_cost,
        weight=-0.06,
        params={
            "max_vel": 0.3,
            "std": 2.0,
            "asset_cfg": SceneEntityCfg("robot", joint_names=ELBOW_WRIST_JOINT_NAMES, preserve_order=True),
        },
    )
    wrist_joint_pos_deviation_from_default = RewTerm(
        func=mdp.joint_pos_deviation_from_default_l2_cost,
        weight=-0.1,
        params={
            "std": 0.35,
            "asset_cfg": SceneEntityCfg("robot", joint_names=WRIST_JOINT_NAMES, preserve_order=True),
        },
    )
    elbow_joint_pos_deviation_from_default = RewTerm(
        func=mdp.joint_pos_deviation_from_default_l2_cost,
        weight=-0.03,
        params={
            "std": 0.45,
            "asset_cfg": SceneEntityCfg("robot", joint_names=ELBOW_JOINT_NAMES, preserve_order=True),
        },
    )
    core_body_rot_kinetic_energy_hinge_cost = RewTerm(
        func=mdp.body_rot_kinetic_energy_hinge_cost,
        weight=-0.2,
        params={
            "free_energy": 0.02,
            "energy_scale": 0.12,
            "inertia_diag": CORE_BODY_INERTIA_DIAG,
            "axis_weights": (1.0, 1.0, 1.0),
            "asset_cfg": SceneEntityCfg("robot", body_names=CORE_BODY_NAMES, preserve_order=True),
        },
    )
