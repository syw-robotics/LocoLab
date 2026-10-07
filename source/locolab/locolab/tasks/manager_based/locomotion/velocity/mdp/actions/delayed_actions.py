from __future__ import annotations

import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

from isaaclab.managers.action_manager import ActionTerm

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from .actions_cfg import DelayedActionCfg


class DelayedAction(ActionTerm):
    """Delays any action term's processed command by a per-environment physics-step lag.

    The wrapped term still owns scaling, clipping, and the sim write. This class only
    replaces ``processed_actions`` for the duration of ``apply_actions``.
    """

    cfg: DelayedActionCfg

    def __init__(self, cfg: DelayedActionCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        if cfg.delay_range[0] < 0 or cfg.delay_range[0] > cfg.delay_range[1]:
            raise ValueError(
                f"[DelayedAction]: delay_range must satisfy 0 <= min <= max. Received {cfg.delay_range}."
            )
        if cfg.delay_range[1] <= 0:
            raise ValueError(f"[DelayedAction]: delay_range max must be > 0. Received {cfg.delay_range}.")

        self._term = cfg.action.class_type(cfg.action, env)
        self.max_delay = cfg.delay_range[1]
        # index 0 is the newest command: (num_envs, max_delay + 1, action_dim)
        self.action_history = torch.zeros(
            (self.num_envs, self.max_delay + 1, self.action_dim),
            device=self.device,
        )
        self.delay_steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)

    def __getattr__(self, name: str):
        # Symmetry and other callers inspect joint metadata on the action term.
        term = self.__dict__.get("_term")
        if term is not None and hasattr(term, name):
            return getattr(term, name)
        raise AttributeError(f"{type(self).__name__!r} object has no attribute {name!r}")

    @property
    def action_dim(self) -> int:
        return self._term.action_dim

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._term.raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._term.processed_actions

    def set_debug_vis(self, debug_vis: bool) -> bool:
        term = getattr(self, "_term", None)
        if term is None:
            return False
        return term.set_debug_vis(debug_vis)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._term.reset(env_ids)
        if env_ids is None:
            env_ids = slice(None)

        # randint samples [low, high), while delay_range is inclusive.
        self.delay_steps[env_ids] = torch.randint(
            self.cfg.delay_range[0],
            self.cfg.delay_range[1] + 1,
            (self.delay_steps[env_ids].shape[0],),
            device=self.device,
        )
        self.action_history[env_ids] = self._term.processed_actions[env_ids].unsqueeze(1)

    def process_actions(self, actions: torch.Tensor):
        self._term.process_actions(actions)

    def apply_actions(self):
        self.action_history = torch.roll(self.action_history, shifts=1, dims=1)
        self.action_history[:, 0, :] = self._term.processed_actions
        delayed_actions = self.action_history[torch.arange(self.num_envs, device=self.device), self.delay_steps]

        saved_actions = self._term._processed_actions
        self._term._processed_actions = delayed_actions
        try:
            self._term.apply_actions()
        finally:
            self._term._processed_actions = saved_actions
