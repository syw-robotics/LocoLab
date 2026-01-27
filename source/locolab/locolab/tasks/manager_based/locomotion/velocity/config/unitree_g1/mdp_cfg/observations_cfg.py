# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

import locolab.tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from . import FOOT_LINK_NAMES, JOINT_NAMES, PRESERVE_ORDER


@configclass
class PropObsCfg(ObsGroup):
    """Proprioceptive observations group."""

    # observation terms (order preserved)
    base_ang_vel = ObsTerm(
        func=mdp.base_ang_vel,
        scale=0.2,
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
        scale=0.05,
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
class PrivObsCfg(ObsGroup):
    """Privileged observations group."""

    # observation terms (order preserved)
    # put base_lin_vel at front for convenience
    base_lin_vel = ObsTerm(func=mdp.base_lin_vel, clip=(-10.0, 10.0))
    base_ang_vel = ObsTerm(func=mdp.base_ang_vel, scale=0.2, clip=(-10.0, 10.0))
    projected_gravity = ObsTerm(func=mdp.projected_gravity, clip=(-1.0, 1.0))
    velocity_commands = ObsTerm(
        func=mdp.generated_commands, params={"command_name": "base_velocity"}, clip=(-10.0, 10.0)
    )
    joint_pos = ObsTerm(
        func=mdp.joint_pos_rel,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        clip=(-10.0, 10.0),
    )
    joint_vel = ObsTerm(
        func=mdp.joint_vel_rel,
        scale=0.05,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        clip=(-50.0, 50.0),
    )
    joint_effort = ObsTerm(
        func=mdp.joint_effort,
        scale=0.05,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        clip=(-500.0, 500.0),
    )
    joint_acc = ObsTerm(
        func=mdp.joint_acc,
        scale=0.05,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=PRESERVE_ORDER)},
        clip=(-500.0, 500.0),
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
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=FOOT_LINK_NAMES)},
    )
    gait_phase = ObsTerm(func=mdp.gait_phase, params={"period": 0.8})

    def __post_init__(self):
        self.enable_corruption = False
        self.concatenate_terms = True
