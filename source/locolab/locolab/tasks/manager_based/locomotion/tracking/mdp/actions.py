"""Joint position targets around either the default pose or the current reference."""

from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg
from isaaclab.utils import configclass

from locolab.tasks.manager_based.locomotion.velocity.mdp.actions.delayed_actions import (
    EncoderBiasJointPositionAction,
)


class TrackingJointPositionAction(EncoderBiasJointPositionAction):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        if cfg.use_reference_residual:
            self._motion = env.command_manager.get_term("motion")
            if self._joint_names != list(self._motion.cfg.joint_names):
                raise ValueError("Reference residual actions must follow the motion joint order.")

    def process_actions(self, actions):
        if not self.cfg.use_reference_residual:
            return super().process_actions(actions)
        self._raw_actions[:] = actions
        self._processed_actions = self._raw_actions * self._scale
        self._processed_actions.add_(self._motion.joint_pos)
        if self.cfg.clip is not None:
            self._processed_actions.clamp_(min=self._clip[:, :, 0], max=self._clip[:, :, 1])


@configclass
class TrackingJointPositionActionCfg(JointPositionActionCfg):
    class_type: type = TrackingJointPositionAction
    use_reference_residual: bool = False
    """Use current reference joint positions instead of the configured/default offset.

    Targets are reference + scale * action, then target clipping and encoder-bias
    compensation. The selected reference is held through physics decimation.
    """
