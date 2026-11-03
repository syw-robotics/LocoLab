# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Physics presets shared by the locomotion environments. """

from __future__ import annotations

from isaaclab_newton.physics import (
    MJWarpSolverCfg,
    NewtonCfg,
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
            njmax=256,  # Number of constraints per environment (world)
            nconmax=64,  # Number of contact points per environment (world)
            cone="pyramidal",  # The type of contact friction cone, "pyramidal" or "elliptic"
            impratio=1.0,
            integrator="implicitfast",  # "implicitfast" is recommended in mjlab
            # Be careful!! 
            # Mujoco contact uses convex hull for contact detection, which is not suitable for rough terrain tasks
            use_mujoco_contacts=False,  # whether to use MuJoCo's internal contact solver
            #  ls_iterations=24,
        ),
        num_substeps=1,
        debug_mode=False,
        default_shape_cfg=NewtonShapeCfg(margin=0.0, ke=160000.0, kd=1100.0),
    )

    # use mjwarp by default
    default = newton_mjwarp


def set_locomotion_physics(
    sim,
    rigid_patch_count: int = 2**16,
    *,
    num_substeps: int = 1,
    cone: str = "pyramidal",
    impratio: float = 1.0,
    njmax: int = 256,
    nconmax: int = 64,
    ls_iterations: int = 50,
    use_mujoco_contacts: bool = False,
) -> None:
    """Install the locomotion physics preset on a simulation config.

    ``rigid_patch_count`` sets the GPU contact-patch budget for every PhysX
    backend, including automatic selection. The remaining parameters configure
    Newton/MJWarp. ``njmax`` and ``nconmax`` are the MuJoCo Warp per-environment
    constraint and contact capacities. ``num_substeps`` splits each physics step;
    ``cone`` and ``impratio`` control the MJWarp friction constraints. Call this
    from an environment ``__post_init__`` so Hydra can still replace the preset from ``--physics``.
    """
    physics = LocomotionPhysicsCfg()
    # Explicit backend names and --physics physx must receive the same budget.
    for backend in (physics.isaacsim_physx, physics.ovphysx, physics.physx.isaacsim_physx, physics.physx.ovphysx):
        if backend is not None:
            backend.gpu_max_rigid_patch_count = rigid_patch_count

    physics.newton_mjwarp.num_substeps = num_substeps
    physics.newton_mjwarp.solver_cfg = physics.newton_mjwarp.solver_cfg.replace(
        cone=cone,
        impratio=impratio,
        njmax=njmax,
        nconmax=nconmax,
        ls_iterations=ls_iterations,
        ls_parallel=True,
        use_mujoco_contacts=use_mujoco_contacts,
    )

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
