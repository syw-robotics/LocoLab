# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import locolab.tasks.manager_based.locomotion.velocity.mdp as mdp
from locolab.tasks.manager_based.locomotion.velocity.mdp import symmetry
from locolab.utils.symmetry import SymmetricObservationTermCfg as SymmObsTerm

from . import FOOT_LINK_NAMES, JOINT_NAMES, PRESERVE_ORDER


#
# We apply empirical normalization in z_rl for obs auto-scaling.
#
# Symmetry is applied by default, since humanoid robot has left-right symmetry.
#

@configclass
class PropObsCfg(ObsGroup):
    """Proprioceptive observations group."""

    # observation terms (order preserved)
    base_ang_vel = SymmObsTerm(
        func=mdp.base_ang_vel,
        scale=0.2,
        noise=Unoise(n_min=-0.2, n_max=0.2),
        clip=(-10.0, 10.0),
        symmetry_transform=symmetry.axial_vector,
    )
    projected_gravity = SymmObsTerm(
        func=mdp.projected_gravity,
        noise=Unoise(n_min=-0.05, n_max=0.05),
        clip=(-1.0, 1.0),
        symmetry_transform=symmetry.polar_vector,
    )
    velocity_commands = SymmObsTerm(
        func=mdp.generated_commands,
        params={"command_name": "base_velocity"},
        clip=(-10.0, 10.0),
        symmetry_transform=symmetry.velocity_command,
    )
    joint_pos = SymmObsTerm(
        func=mdp.joint_pos_rel,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        noise=Unoise(n_min=-0.01, n_max=0.01),
        clip=(-10.0, 10.0),
        symmetry_transform=symmetry.joint_space,
    )
    joint_vel = SymmObsTerm(
        func=mdp.joint_vel_rel,
        scale=0.05,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        noise=Unoise(n_min=-1.5, n_max=1.5),
        clip=(-50.0, 50.0),
        symmetry_transform=symmetry.joint_space,
    )
    actions = SymmObsTerm(
        func=mdp.last_action,
        clip=(-10.0, 10.0),
        symmetry_transform=symmetry.last_joint_action,
    )

    def __post_init__(self):
        self.enable_corruption = True
        self.concatenate_terms = True


@configclass
class PrivObsCfg(ObsGroup):
    """Privileged observations group."""

    # observation terms (order preserved)
    # put base_lin_vel at front for convenience
    base_lin_vel = SymmObsTerm(
        func=mdp.base_lin_vel,
        clip=(-10.0, 10.0),
        symmetry_transform=symmetry.polar_vector,
    )
    base_ang_vel = SymmObsTerm(
        func=mdp.base_ang_vel,
        scale=0.2,
        clip=(-10.0, 10.0),
        symmetry_transform=symmetry.axial_vector,
    )
    projected_gravity = SymmObsTerm(
        func=mdp.projected_gravity,
        clip=(-1.0, 1.0),
        symmetry_transform=symmetry.polar_vector,
    )
    velocity_commands = SymmObsTerm(
        func=mdp.generated_commands,
        params={"command_name": "base_velocity"},
        clip=(-10.0, 10.0),
        symmetry_transform=symmetry.velocity_command,
    )
    joint_pos = SymmObsTerm(
        func=mdp.joint_pos_rel,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        clip=(-10.0, 10.0),
        symmetry_transform=symmetry.joint_space,
    )
    joint_vel = SymmObsTerm(
        func=mdp.joint_vel_rel,
        scale=0.05,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        clip=(-50.0, 50.0),
        symmetry_transform=symmetry.joint_space,
    )
    joint_effort = SymmObsTerm(
        func=mdp.joint_effort,
        scale=0.05,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        clip=(-500.0, 500.0),
        symmetry_transform=symmetry.joint_space,
    )
    joint_acc = SymmObsTerm(
        func=mdp.joint_acc,
        scale=0.05,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        clip=(-500.0, 500.0),
        symmetry_transform=symmetry.joint_space,
    )
    actions = SymmObsTerm(
        func=mdp.last_action,
        clip=(-10.0, 10.0),
        symmetry_transform=symmetry.last_joint_action,
    )
    height_scan = SymmObsTerm(
        func=mdp.height_scan,
        params={"sensor_cfg": SceneEntityCfg("height_scanner")},
        clip=(-10.0, 10.0),
        symmetry_transform=symmetry.height_scan_y,
    )
    feet_contact_flag = SymmObsTerm(
        func=mdp.feet_contact_flag,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_LINK_NAMES)},
        symmetry_transform=symmetry.body_scalar,
    )
    gait_phase = SymmObsTerm(func=mdp.gait_phase, params={"period": 0.8}, symmetry_transform=symmetry.identity)

    def __post_init__(self):
        self.enable_corruption = False
        self.concatenate_terms = True
