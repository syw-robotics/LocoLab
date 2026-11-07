"""The experimental runtime with LocoLab history and the existing RL environment interface."""

import json
import logging
import torch
import warp as wp

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import CommandManager
from isaaclab_experimental.envs import ManagerBasedRLEnvWarp

from .history import HistoryObservationManager

logger = logging.getLogger(__name__)


class _TrackingCallSwitch:
    """Refresh Python-guarded state before replaying tracking manager graphs."""

    def __init__(self, switch, env):
        self._switch = switch
        self._env = env
        self._storage = {}
        cfg = getattr(env, "cfg", None)
        requested = getattr(cfg, "manager_call_config", None) or {}
        if isinstance(requested, str):
            requested = json.loads(requested) if requested.strip() else {}
        max_modes = getattr(cfg, "manager_call_max_mode", None) or {}
        # This G1 path has graph-safe Scene writes and interval/reset events.
        # Startup randomizers run directly, outside captured stages. Preserve
        # explicit user caps while lifting the upstream blanket restrictions.
        for name in ("Scene", "EventManager"):
            mode = requested.get(name, requested.get("default", switch.DEFAULT_CONFIG["default"]))
            if mode == 2 and max_modes.get(name, 2) >= 2:
                switch._cfg[name] = 2
                switch._max_modes[name] = 2

    def __getattr__(self, name):
        return getattr(self._switch, name)

    def call_stage(self, *, stage, **kwargs):
        if stage in (
            "TerminationManager_compute", "RewardManager_compute",
            "ObservationManager_compute_no_history", "ObservationManager_compute_update_history",
        ) and self._switch.get_mode_for_manager(stage.split("_", 1)[0]) == 2:
            data = self._env.command_manager.get_term("motion").robot.data
            arrays = [data.body_link_pose_w.warp]
            if stage == "RewardManager_compute":
                arrays.extend([data.body_link_vel_w.warp, data.root_link_pose_w.warp])
                for cfg in self._env.reward_manager._term_cfgs:
                    for name in cfg.params.get("sensor_names", ()):
                        history = self._env.scene[name].data.normal_force_matrix_w_history
                        if history is not None:
                            arrays.append(history.warp)
            storage = tuple(array.ptr for array in arrays)
            if self._storage.get(stage) != storage:
                self._graph_cache.invalidate(stage)
                self._storage[stage] = storage
        return self._switch.call_stage(stage=stage, **kwargs)


class LocoLabWarpEnv(ManagerBasedRLEnvWarp, ManagerBasedRLEnv):
    # Warp methods take precedence. The stable base only supplies nominal compatibility with
    # Z-RL's isinstance check; Warp's base initializer and close do not enter the stable runtime.
    def _reset_idx(self, env_ids, *, env_mask=None):
        super()._reset_idx(env_ids, env_mask=env_mask)
        # Managers expose persistent scalar views; the logger retains them until
        # the rollout ends. One packed snapshot prevents later resets overwriting them.
        log = self.extras["log"]
        keys = [key for key, value in log.items() if isinstance(value, torch.Tensor)]
        if keys:
            values = torch.stack([log[key].detach().reshape(()) for key in keys])
            log.update(zip(keys, values.unbind()))

    def load_managers(self):
        from .motion_command import WarpMotionCommand

        motion_cfg = getattr(self.cfg.commands, "motion", None)
        if motion_cfg is not None and motion_cfg.class_type is WarpMotionCommand and any(
            self._manager_call_switch.get_mode_for_manager(name) == 2
            for name in ("ObservationManager", "RewardManager", "TerminationManager")
        ):
            self._manager_call_switch = _TrackingCallSwitch(self._manager_call_switch, self)
        self.command_manager = CommandManager(self.cfg.commands, self)
        for attr, name, cfg in (
            ("recorder_manager", "RecorderManager", self.cfg.recorders),
            ("action_manager", "ActionManager", self.cfg.actions),
        ):
            setattr(self, attr, self._manager_call_switch.resolve_manager_class(name)(cfg, self))
        self.observation_manager = HistoryObservationManager(self.cfg.observations, self)
        for attr, name, cfg in (
            ("termination_manager", "TerminationManager", self.cfg.terminations),
            ("reward_manager", "RewardManager", self.cfg.rewards),
            ("curriculum_manager", "CurriculumManager", self.cfg.curriculum),
        ):
            setattr(self, attr, self._manager_call_switch.resolve_manager_class(name)(cfg, self))
        # Torch command/push curricula read per-term episode totals at reset.
        # These views share Warp storage and require no per-step copy.
        self.reward_manager._episode_sums = dict(zip(
            self.reward_manager.active_terms, wp.to_torch(self.reward_manager._episode_sums_wp).unbind(0)
        ))
        for name in ("event", "command", "action", "observation", "termination", "reward", "curriculum"):
            logger.info("%s", getattr(self, name + "_manager"))
        self._configure_gym_env_spaces()
        if "startup" in self.event_manager.available_modes:
            self.event_manager.apply(mode="startup")
