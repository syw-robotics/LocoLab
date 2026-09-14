# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import locolab.tasks.manager_based.locomotion.velocity.mdp as mdp
from locolab.tasks.manager_based.locomotion.velocity.mdp import symmetry
from locolab.utils.symmetry import SymmetricObservationTermCfg as SymmObsTerm

from . import JOINT_NAMES, PRESERVE_ORDER


#
# We apply empirical normalization in z_rl for obs auto-scaling.
#

@configclass
class PropObsCfg(ObsGroup):
    """Proprioceptive observations group."""

    # observation terms (order preserved)
    base_ang_vel = ObsTerm(
        func=mdp.base_ang_vel,
        noise=Unoise(n_min=-0.2, n_max=0.2),
        clip=(-10.0, 10.0),
    )
    projected_gravity = ObsTerm(
        func=mdp.projected_gravity,
        noise=Unoise(n_min=-0.05, n_max=0.05),
        clip=(-1.0, 1.0),
    )
    velocity_commands = ObsTerm(
        func=mdp.generated_commands,
        params={"command_name": "base_velocity"},
        clip=(-10.0, 10.0),
    )
    joint_pos = ObsTerm(
        func=mdp.joint_pos_rel,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        noise=Unoise(n_min=-0.01, n_max=0.01),
        clip=(-10.0, 10.0),
    )
    joint_vel = ObsTerm(
        func=mdp.joint_vel_rel,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        noise=Unoise(n_min=-1.5, n_max=1.5),
        clip=(-50.0, 50.0),
    )
    actions = ObsTerm(
        func=mdp.last_action,
        clip=(-10.0, 10.0),
    )

    def __post_init__(self):
        self.enable_corruption = True
        self.concatenate_terms = True


@configclass
class PropObsCfg_W_Symmetry(ObsGroup):
    """Proprioceptive observations group with symmetry."""

    # observation terms (order preserved)
    base_ang_vel = SymmObsTerm(
        func=mdp.base_ang_vel,
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
    base_lin_vel = ObsTerm(func=mdp.base_lin_vel, clip=(-10.0, 10.0))
    base_ang_vel = ObsTerm(func=mdp.base_ang_vel, clip=(-10.0, 10.0))
    projected_gravity = ObsTerm(func=mdp.projected_gravity, clip=(-1.0, 1.0))
    velocity_commands = ObsTerm(
        func=mdp.generated_commands,
        params={"command_name": "base_velocity"},
        clip=(-10.0, 10.0),
    )
    joint_pos = ObsTerm(
        func=mdp.joint_pos_rel,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        clip=(-10.0, 10.0),
    )
    joint_vel = ObsTerm(
        func=mdp.joint_vel_rel,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        clip=(-50.0, 50.0),
    )
    joint_effort = ObsTerm(
        func=mdp.joint_effort,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        clip=(-50.0, 50.0),
    )
    joint_acc = ObsTerm(
        func=mdp.joint_acc,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        clip=(-50.0, 50.0),
    )
    actions = ObsTerm(
        func=mdp.last_action,
        clip=(-10.0, 10.0),
    )
    height_scan = ObsTerm(
        func=mdp.height_scan,
        params={"sensor_cfg": SceneEntityCfg("height_scanner")},
        clip=(-10.0, 10.0),
    )
    feet_contact_flag = ObsTerm(
        func=mdp.feet_contact_flag,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot")},
    )
    #  feet_height = ObsTerm(
    #      func=mdp.feet_height,
    #      params={"feet_names": ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]},
    #      clip=(-10.0, 10.0),
    #  )

    def __post_init__(self):
        self.enable_corruption = False
        self.concatenate_terms = True


@configclass
class PrivObsCfg_W_Symmetry(ObsGroup):
    """Privileged observations group with symmetry."""

    # observation terms (order preserved)
    # put base_lin_vel at front for convenience
    base_lin_vel = SymmObsTerm(
        func=mdp.base_lin_vel, 
        clip=(-10.0, 10.0), 
        symmetry_transform=symmetry.polar_vector,
    )
    base_ang_vel = SymmObsTerm(
        func=mdp.base_ang_vel, 
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
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        clip=(-50.0, 50.0),
        symmetry_transform=symmetry.joint_space,
    )
    joint_effort = SymmObsTerm(
        func=mdp.joint_effort,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        clip=(-50.0, 50.0),
        symmetry_transform=symmetry.joint_space,
    )
    joint_acc = SymmObsTerm(
        func=mdp.joint_acc,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        clip=(-50.0, 50.0),
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
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot")},
        symmetry_transform=symmetry.body_scalar,
    )
    #  feet_height = SymmObsTerm(
    #      func=mdp.feet_height,
    #      params={"feet_names": ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]},
    #      clip=(-10.0, 10.0),
    #  )

    def __post_init__(self):
        self.enable_corruption = False
        self.concatenate_terms = True