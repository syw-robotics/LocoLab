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
from isaaclab.utils import configclass



def assetx_export_usd_cfg(
    asset_path: str,
    foot_expression: str,
    *,
    merge_fixed_joints: bool,
    replace_cylinders_with_capsules: bool = True,
    keep_fixed_pattern: str | None = None,
    honor_dont_collapse: bool = True,
) -> sim_utils.UsdFileCfg:
    """Export a URDF or MJCF with assetx and return a ``UsdFileCfg``.

    ``.xml`` / ``.mjcf`` are exported as written. See ``locolab/utils/urdf/README.md``.
    """
    from locolab.utils.urdf.assetx_export import assetx_export_usd

    usd_path = assetx_export_usd(
        asset_path,
        merge_fixed_joints=merge_fixed_joints,
        replace_cylinders_with_capsules=replace_cylinders_with_capsules,
        fix_base=False,
        keep_fixed_pattern=keep_fixed_pattern,
        honor_dont_collapse=honor_dont_collapse,
    )

    return sim_utils.UsdFileCfg(
        usd_path=str(usd_path),
        activate_contact_sensors=True,
        rigid_props=PhysxRigidBodyCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=[  # disable self-collisions by default; override later
            PhysxArticulationCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=2,
            ),
            NewtonArticulationCfg(self_collision_enabled=False),
        ],
        collision_props={  # Foot friction on MJWarp; other colliders stay frictionless.
            "/.*": [MujocoCollisionCfg(condim=1, priority=0)],  # ``condim=3`` enables tangential friction
            foot_expression: [MujocoCollisionCfg(condim=3, priority=1)],  # ``condim=1`` is contact only and suitable for self-collisions;
        },
    )


@configclass
class LocoLabArticulationCfg(ArticulationCfg):
    """LocoLab articulation with SDK joint order and symmetry mappings."""

    joint_sdk_names: list[str] = None

    joint_symmetry_mapping: dict[str, tuple[float, str]] | None = None
    spatial_symmetry_mapping: dict[str, str] | None = None

    source_asset: str | None = None
    """Description Viser draws. The MJCF when that is the spawned robot, otherwise the URDF."""

    soft_joint_pos_limit_factor = 0.9
