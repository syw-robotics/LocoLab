# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Physics presets shared by the locomotion environments. """

from __future__ import annotations

from typing import TypedDict

from isaaclab_newton.physics import (
    MJWarpSolverCfg,
    NewtonCfg,
    NewtonCollisionPipelineCfg,
    NewtonShapeCfg,
    SteelSolverCfg,
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
    isaacsim_physx: PhysxCfg = PhysxCfg(gpu_max_rigid_patch_count=2**16)
    ovphysx: OvPhysxCfg = OvPhysxCfg(gpu_max_rigid_patch_count=2**16)
    physx: PhysxAutoCfg = PhysxAutoCfg(isaacsim_physx=isaacsim_physx, ovphysx=ovphysx)

    # newton mjwarp
    newton_mjwarp: NewtonCfg = NewtonCfg(
        solver_cfg=MJWarpSolverCfg(
            njmax=256,  # Number of constraints per environment (world)
            nconmax=64,  # Number of contact points per environment (world)
            cone="pyramidal",  # The type of contact friction cone, "pyramidal" or "elliptic"
            impratio=5.0,
            integrator="implicitfast",  # "implicitfast" is recommended in mjlab
            ls_iterations=50,
            ccd_iterations=35,
            ls_parallel=True,
            # Be careful!! 
            # Mujoco contact uses convex hull for contact detection, which is not suitable for rough terrain tasks
            use_mujoco_contacts=False,  # whether to use MuJoCo's internal contact solver
            #  tolerance=1e-8,
        ),
        num_substeps=1,
        collision_decimation=0,
        debug_mode=False,
        # open the speculative contact by default, do not use mujoco contact detection
        collision_cfg=NewtonCollisionPipelineCfg(speculative_contact_gap_max=0.05),
        default_shape_cfg=NewtonShapeCfg(margin=0.0, ke=160000.0, kd=1100.0),
    )

    newton_steel: NewtonCfg = NewtonCfg(
        solver_cfg=SteelSolverCfg(
            iterations=4,
            update_mass_matrix_interval=2,
            use_parallel_streams=True,
            enable_joint_limits=True,
            articulation_constraint_max=192,
            free_body_constraint_max=32,
            contact_friction_gap_max=0.02,
        ),
        num_substeps=1,
        default_shape_cfg=NewtonShapeCfg(margin=0.002, ke=160000.0, kd=1100.0),
    )

    # use mjwarp by default
    default = newton_mjwarp


class MJWarpOverrides(TypedDict, total=False):
    num_substeps: int
    collision_decimation: int
    speculative_contact_gap_max: float | None
    cone: str
    impratio: float
    njmax: int
    nconmax: int | None
    ls_iterations: int
    ccd_iterations: int
    use_mujoco_contacts: bool


class SteelOverrides(TypedDict, total=False):
    margin: float
    articulation_constraint_max: int
    update_mass_matrix_interval: int


class PhysxOverrides(TypedDict, total=False):
    rigid_patch_count: int


def set_locomotion_physics(
    sim,
    *,
    mjwarp: MJWarpOverrides | None = None,
    steel: SteelOverrides | None = None,
    physx: PhysxOverrides | None = None,
) -> None:
    """Install the locomotion physics preset with optional per-backend overrides.

    For MJWarp, ``nconmax=None`` selects automatic contact capacity. With Newton
    collision detection, ``speculative_contact_gap_max`` caps the velocity-based
    expansion of contact detection. ``None`` selects Newton's default pipeline.

    Steel's ``margin`` overrides the shape margin. PhysX's patch budget applies
    to explicit and automatic backends. Call this from ``__post_init__`` to keep
    the presets selectable via ``--physics``.
    """
    physics = LocomotionPhysicsCfg()
    if physx:
        # Explicit backend names and --physics physx must receive the same budget.
        for backend in (physics.isaacsim_physx, physics.ovphysx, physics.physx.isaacsim_physx, physics.physx.ovphysx):
            if backend is not None:
                backend.gpu_max_rigid_patch_count = physx["rigid_patch_count"]

    mjwarp_cfg = physics.newton_mjwarp
    collision_cfg = mjwarp_cfg.collision_cfg
    if mjwarp:
        mjwarp_options = dict(mjwarp)
        mjwarp_cfg.num_substeps = mjwarp_options.pop("num_substeps", mjwarp_cfg.num_substeps)
        mjwarp_cfg.collision_decimation = mjwarp_options.pop("collision_decimation", mjwarp_cfg.collision_decimation)
        collision_cfg.speculative_contact_gap_max = mjwarp_options.pop(
            "speculative_contact_gap_max", collision_cfg.speculative_contact_gap_max
        )
        mjwarp_cfg.solver_cfg = mjwarp_cfg.solver_cfg.replace(**mjwarp_options)

    # MuJoCo contact detection bypasses Newton's collision pipeline.
    if mjwarp_cfg.solver_cfg.use_mujoco_contacts:
        mjwarp_cfg.collision_cfg = None

    if steel:
        steel_cfg = physics.newton_steel
        steel_options = dict(steel)
        steel_cfg.default_shape_cfg.margin = steel_options.pop("margin", steel_cfg.default_shape_cfg.margin)
        steel_cfg.solver_cfg = steel_cfg.solver_cfg.replace(**steel_options)

    # PresetCfg copies the default, so implicit backend selection needs the tuned instance too.
    physics.default = physics.newton_mjwarp  # overwrite mjwarp as options as default physics backend
    sim.physics = physics

    # ImplicitActuatorCfg ignores this flag and keeps its drive on the solver.
    # The flag enables native Newton actuators for explicit configs such as DCMotorCfg and IdealPDActuatorCfg.
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
