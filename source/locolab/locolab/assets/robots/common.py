# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Shared robot asset helpers and articulation configuration base classes."""

from isaaclab_newton.sim.schemas import MujocoCollisionCfg, NewtonArticulationCfg
from isaaclab_physx.sim.schemas import PhysxArticulationCfg, PhysxRigidBodyCfg

import isaaclab.sim as sim_utils
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.sim.schemas import UsdPhysicsCollisionCfg
from isaaclab.utils import configclass


def mujoco_foot_collision_props(foot_expression: str) -> dict[str, list]:
    """Foot friction on MJWarp; other colliders stay frictionless.

    ``condim=3`` enables tangential friction for locomotion;
    ``condim=1`` is contact only and suitable for self-collisions;
    """
    return {
        "/.*": [MujocoCollisionCfg(condim=1, priority=0)],
        foot_expression: [MujocoCollisionCfg(condim=3, priority=1)],
    }


def g1_mujoco_collision_props() -> dict[str, list]:
    """mjlab G1 contacts, thinned to fit typical MJWarp budgets (e.g. ``njmax=100``,
    ``nconmax`` in the low tens from ``use_locomotion_physics``).

    Same ``condim`` / ``priority`` idea as :func:`mujoco_foot_collision_props`, but
    non-foot links use ``priority=2`` so they still win over feet when the robot is
    on the ground; feet at ``priority=1`` beat Newton's ``priority=0`` ground, so only
    foot–ground stays ``condim=3``. mjlab keeps seven foot capsules at ``condim=3`` and
    every other collider at ``condim=1``.
    Those seven cylinders lie on one plane, so both feet already spend 14 * 4 rows. The sole
    keeps the outer, center, and outer bars (``cylinder_1``, ``cylinder_3``, ``cylinder_5``).
    The four overlapping bars are turned off. Later entries override earlier ones.
    """
    return {
        "/.*": [MujocoCollisionCfg(condim=1, priority=2)],
        "/.*ankle_roll_link/cylinder.*": [MujocoCollisionCfg(condim=3, priority=1)],
        "/.*ankle_roll_link/cylinder(_[246])?$": [UsdPhysicsCollisionCfg(collision_enabled=False)],
    }


def articulation_fragments(
    self_collisions: bool,
    position_iterations: int = 4,
    velocity_iterations: int = 2,
) -> list:
    """PhysX and Newton articulation settings for one robot.

    A bare PhysX fragment is ignored when the simulation runs on Newton, so
    self-collision is written into both backends.
    """
    return [
        PhysxArticulationCfg(
            enabled_self_collisions=self_collisions,
            solver_position_iteration_count=position_iterations,
            solver_velocity_iteration_count=velocity_iterations,
        ),
        NewtonArticulationCfg(self_collision_enabled=self_collisions),
    ]


def locolab_urdf_cfg(asset_path: str, foot_expression: str, *, merge_fixed_joints: bool) -> sim_utils.UrdfFileCfg:
    """Build the URDF spawn shared by the Unitree robots."""
    return sim_utils.UrdfFileCfg(
        fix_base=False,
        asset_path=asset_path,
        activate_contact_sensors=True,
        merge_fixed_joints=merge_fixed_joints,
        rigid_props=PhysxRigidBodyCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=articulation_fragments(self_collisions=False),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
        collision_props=mujoco_foot_collision_props(foot_expression),
    )


def quadruped_urdf_cfg(source_urdf: str, foot_expression: str) -> sim_utils.UrdfFileCfg:
    """Spawn URDF for Unitree quadrupeds: merge rotors/sensors in Python, keep foot fixed."""
    from locolab.utils.urdf import urdf_for_spawn

    return locolab_urdf_cfg(urdf_for_spawn(source_urdf), foot_expression, merge_fixed_joints=False)


@configclass
class LocoLabArticulationCfg(ArticulationCfg):
    """LocoLab articulation with SDK joint order and symmetry mappings."""

    joint_sdk_names: list[str] = None

    joint_symmetry_mapping: dict[str, tuple[float, str]] | None = None
    spatial_symmetry_mapping: dict[str, str] | None = None

    soft_joint_pos_limit_factor = 0.9
