# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

import locolab.tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from . import BASE_LINK_NAME, FOOT_LINK_NAMES, JOINT_NAMES, OTHER_BODY_LINK_NAMES


@configclass
class EventCfg:
    """Configuration for events."""

    # ===== startup ===== (6 events)
    # 1.
    randomize_feet_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_LINK_NAMES),
            "static_friction_range": (0.2, 1.5),
            "dynamic_friction_range": (0.2, 1.5),
            "restitution_range": (0.0, 0.5),
            "num_buckets": 64,
            "make_consistent": (
                True
            ),  # Ensure dynamic friction is less than or equal to static friction. This obeys the physics constraint on friction values.
        },
    )
    # 2.
    randomize_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=BASE_LINK_NAME),
            "mass_distribution_params": (-1.0, 3.0),
            "operation": "add",
            "recompute_inertia": True,
        },
    )
    # 3.
    randomize_other_bodies_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=OTHER_BODY_LINK_NAMES),
            "mass_distribution_params": (0.8, 1.2),
            "operation": "scale",
            "recompute_inertia": True,
        },
    )
    # 4.
    randomize_base_com = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=BASE_LINK_NAME),
            "com_range": {"x": (-0.05, 0.05), "y": (-0.05, 0.05), "z": (-0.05, 0.05)},
        },
    )
    # 5.
    randomize_actuator_gains = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES),
            "stiffness_distribution_params": (0.8, 1.2),
            "damping_distribution_params": (0.8, 1.2),
            "operation": "scale",
            "distribution": "uniform",
        },
    )
    # 6.
    #  randomize_joint_parameters = EventTerm(
    #      func=mdp.randomize_joint_parameters,
    #      mode="startup",
    #      params={
    #          "asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES),
    #          "friction_distribution_params": (0.0, 0.1),
    #          "armature_distribution_params": (0.0, 0.01),
    #          "operation": "add",
    #          "distribution": "uniform",
    #      },
    #  )
    # 7. TODO: This term is to be implemented to simulate the joint offset deviation. (maybe not necessary)
    #  randomize_joint_default_pos = EventTerm(
    #      func=mdp.randomize_joint_default_pos,
    #      mode="startup",
    #      params={
    #          "asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES),
    #          "pos_distribution_params": (-0.02, 0.02),
    #          "operation": "add",
    #      },
    #  )

    # ===== reset ===== (2 events)
    # 1.
    reset_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.75, 1.25),
            "velocity_range": (0.0, 0.0),
        },
    )
    # 2.
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (0.0, 0.2),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (-3.14, 3.14),
            },
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )
    #  3. NOTE: Be careful with this term!
    #           Is reset, external disturbance acts during whole episode.
    #           If interval, external disturbance resamples its value and acts within 'interval_range_s'
    #  reset_base_external_force_torque = EventTerm(
    #      func=mdp.apply_external_force_torque,
    #      mode="reset",
    #      params={
    #          "asset_cfg": SceneEntityCfg("robot", body_names=BASE_LINK_NAME),
    #          "force_range": (0.0, 0.0),
    #          "torque_range": (-0.0, 0.0),
    #      },
    #  )
    #  4. NOTE: 'reset_gravity' term is implemented by IsaacLab
    # reset_gravity = EventTerm(
    #     func=mdp.reset_gravity,
    #     mode="reset",
    #     params={
    #         "gravity_distribution_params": ([0.0, 0.0, -0.02], [0.0, 0.0, 0.02]),
    #         "operation": "add",
    #         "distribution": "uniform",
    #     },
    # )

    # ===== interval ===== (1 events)
    # 1.
    push_robot_by_setting_velocity = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),
        params={"velocity_range": {"x": (-1.0, 1.0), "y": (-1.0, 1.0)}},
    )
    # 2. TODO: This term is to be implemented
    #  push_robot_by_force_torques = EventTerm(
    #      func=mdp.push_by_setting_velocity,
    #      mode="interval",
    #      interval_range_s=(8.0, 10.0),
    #      params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
    #  )
