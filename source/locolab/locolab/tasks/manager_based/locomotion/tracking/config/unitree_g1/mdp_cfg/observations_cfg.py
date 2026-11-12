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
from isaaclab.utils.noise import UniformNoiseCfg as Unoise

import locolab.tasks.manager_based.locomotion.tracking.mdp as mdp

from . import JOINT_NAMES


def _robot_joints() -> dict:
    return {"asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES, preserve_order=True)}


@configclass
class PolicyCfg(ObsGroup):
    """Deployable reference and onboard observations. z_rl normalizes this group."""

    command = ObsTerm(func=mdp.generated_commands, params={"command_name": "motion"})
    base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))
    projected_gravity = ObsTerm(func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
    joint_pos = ObsTerm(func=mdp.joint_pos_rel_biased, params=_robot_joints(), noise=Unoise(n_min=-0.01, n_max=0.01))
    joint_vel = ObsTerm(func=mdp.joint_vel_rel, params=_robot_joints(), noise=Unoise(n_min=-0.5, n_max=0.5))
    actions = ObsTerm(func=mdp.last_action)

    def __post_init__(self):
        self.enable_corruption = True
        self.concatenate_terms = True


@configclass
class PrivilegedCfg(ObsGroup):
    """Critic observations, including body poses in the anchor frame."""

    command = ObsTerm(func=mdp.generated_commands, params={"command_name": "motion"})
    motion_anchor_pos_b = ObsTerm(func=mdp.motion_anchor_pos_b, params={"command_name": "motion"})
    motion_anchor_ori_b = ObsTerm(func=mdp.motion_anchor_ori_b, params={"command_name": "motion"})
    body_pos = ObsTerm(func=mdp.robot_body_pos_b, params={"command_name": "motion"})
    body_ori = ObsTerm(func=mdp.robot_body_ori_b, params={"command_name": "motion"})
    base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
    base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
    projected_gravity = ObsTerm(func=mdp.projected_gravity)
    joint_pos = ObsTerm(func=mdp.joint_pos_rel, params=_robot_joints())
    joint_vel = ObsTerm(func=mdp.joint_vel_rel, params=_robot_joints())
    actions = ObsTerm(func=mdp.last_action)

    def __post_init__(self):
        self.enable_corruption = False
        self.concatenate_terms = True
