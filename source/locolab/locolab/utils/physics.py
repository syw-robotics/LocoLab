# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Physics presets shared by the locomotion environments.

The kitless Isaac Lab 3.0 environment has no Omniverse Kit, so the default
matches the upstream velocity tasks: Newton with MJWarp. ``--physics ovphysx``
selects kitless PhysX. ``--physics isaacsim_physx`` needs a full Isaac Sim install.
"""

from __future__ import annotations

from isaaclab_newton.physics import (
    MJWarpSolverCfg,
    NewtonCfg,
    NewtonCollisionPipelineCfg,
    NewtonShapeCfg,
)
from isaaclab_ov.physics import OvPhysxCfg
from isaaclab_physx.physics import PhysxCfg

from isaaclab.physics import PhysxAutoCfg
from isaaclab.utils import configclass
from isaaclab_tasks.utils import PresetCfg


@configclass
class LocomotionPhysicsCfg(PresetCfg):
    """Selectable physics backends for the Unitree velocity tasks."""

    # physx
    isaacsim_physx: PhysxCfg = PhysxCfg(gpu_max_rigid_patch_count=10 * 2**15)
    ovphysx: OvPhysxCfg = OvPhysxCfg(gpu_max_rigid_patch_count=10 * 2**15)
    physx: PhysxAutoCfg = PhysxAutoCfg(isaacsim_physx=isaacsim_physx, ovphysx=ovphysx)

    # newton mjwarp
    newton_mjwarp: NewtonCfg = NewtonCfg(
        solver_cfg=MJWarpSolverCfg(
            njmax=60,  # Number of constraints per environment (world)
            nconmax=25,  # Number of contact points per environment (world)
            cone="pyramidal",  # The type of contact friction cone, "pyramidal" or "elliptic"
            impratio=1.0,
            integrator="implicitfast",  # "implicitfast" is recommended in mjlab
            use_mujoco_contacts=False,  # whether to use MuJoCo's internal contact solver
        ),
        num_substeps=1,
        debug_mode=False,
        default_shape_cfg=NewtonShapeCfg(margin=0.0, ke=160000.0, kd=1100.0),
    )

    # use physx by default
    default = physx


def use_locomotion_physics(
    sim,
    rigid_patch_count: int = 2**16,
    *,
    njmax: int = 60,
    nconmax: int = 25,
) -> None:
    """Install the locomotion physics preset on a simulation config.

    ``rigid_patch_count`` is the PhysX GPU contact-patch budget. Newton does
    not read it. ``njmax`` and ``nconmax`` are the MuJoCo Warp per-environment
    constraint and contact capacities. Call this from an environment
    ``__post_init__`` so Hydra can still replace the preset from ``--physics``.
    """
    physics = LocomotionPhysicsCfg()
    physics.isaacsim_physx.gpu_max_rigid_patch_count = rigid_patch_count
    if physics.physx.isaacsim_physx is not None:
        physics.physx.isaacsim_physx.gpu_max_rigid_patch_count = rigid_patch_count
    solver_cfg = physics.newton_mjwarp.solver_cfg
    solver_cfg.njmax = njmax
    solver_cfg.nconmax = nconmax
    sim.physics = physics
    # ImplicitActuatorCfg ignores this flag and keeps its drive on the solver. The flag enables
    # native Newton actuators for explicit configs such as DCMotorCfg and IdealPDActuatorCfg.
    sim.use_newton_actuators = False


def set_self_collisions(spawn, enabled: bool) -> None:
    """Set self-collision on every articulation fragment attached to a spawner."""
    props = spawn.articulation_props
    fragments = props if isinstance(props, (list, tuple)) else (props,)
    for fragment in fragments:
        # physx
        if hasattr(fragment, "enabled_self_collisions"):
            fragment.enabled_self_collisions = enabled
        # newton mjwarp
        if hasattr(fragment, "self_collision_enabled"):
            fragment.self_collision_enabled = enabled
