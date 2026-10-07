# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from collections.abc import Callable
from dataclasses import MISSING

from isaaclab.envs.mdp.actions import joint_actions
from isaaclab.managers.action_manager import ActionTerm, ActionTermCfg
from isaaclab.utils import configclass

from .delayed_actions import DelayedAction

##
# Joint actions.
##


@configclass
class DelayedActionCfg(ActionTermCfg):
    """Wraps an action term and delays its processed command.

    Delay is counted in physics steps, because ``apply_actions`` runs inside the
    decimation loop. Scale, clip, and the sim write stay on ``action``.
    """

    class_type: type[ActionTerm] = DelayedAction

    action: ActionTermCfg = MISSING
    """The action term to delay."""

    delay_range: tuple[int, int] = (0, 1)
    """Inclusive physics-step delay sampled per environment at reset."""

    symmetry_transform: Callable | None = None
    """Optional symmetry transform. Defaults to the wrapped action's transform."""

    def __post_init__(self):
        self.asset_name = self.action.asset_name
        if self.symmetry_transform is None:
            self.symmetry_transform = getattr(self.action, "symmetry_transform", None)


@configclass
class JointActionCfg(ActionTermCfg):
    """Configuration for the base joint action term.

    See :class:`JointAction` for more details.
    """

    joint_names: list[str] = MISSING
    """List of joint names or regex expressions that the action will be mapped to."""
    scale: float | dict[str, float] = 1.0
    """Scale factor for the action (float or dict of regex expressions). Defaults to 1.0."""
    offset: float | dict[str, float] = 0.0
    """Offset factor for the action (float or dict of regex expressions). Defaults to 0.0."""
    preserve_order: bool = False
    """Whether to preserve the order of the joint names in the action output. Defaults to False."""
    symmetry_transform: Callable | None = None
    """Optional runtime provider for the raw action's symmetry transform."""


# -------------------- Joint Position Action -------------------- #


@configclass
class JointPositionActionCfg(JointActionCfg):
    """Configuration for the joint position action term.

    See :class:`JointPositionAction` for more details.
    """

    class_type: type[ActionTerm] = joint_actions.JointPositionAction

    use_default_offset: bool = True
    """Whether to use default joint positions configured in the articulation asset as offset.
    Defaults to True.

    If True, this flag results in overwriting the values of :attr:`offset` to the default joint positions
    from the articulation asset.
    """


@configclass
class RelativeJointPositionActionCfg(JointActionCfg):
    """Configuration for the relative joint position action term.

    See :class:`RelativeJointPositionAction` for more details.
    """

    class_type: type[ActionTerm] = joint_actions.RelativeJointPositionAction

    use_zero_offset: bool = True
    """Whether to ignore the offset defined in articulation asset. Defaults to True.

    If True, this flag results in overwriting the values of :attr:`offset` to zero.
    """


# -------------------- Joint Velocity Action -------------------- #


@configclass
class JointVelocityActionCfg(JointActionCfg):
    """Configuration for the joint velocity action term.

    See :class:`JointVelocityAction` for more details.
    """

    class_type: type[ActionTerm] = joint_actions.JointVelocityAction

    use_default_offset: bool = True
    """Whether to use default joint velocities configured in the articulation asset as offset.
    Defaults to True.

    This overrides the settings from :attr:`offset` if set to True.
    """


# -------------------- Joint Effort Action -------------------- #


@configclass
class JointEffortActionCfg(JointActionCfg):
    """Configuration for the joint effort action term.

    See :class:`JointEffortAction` for more details.
    """

    class_type: type[ActionTerm] = joint_actions.JointEffortAction
