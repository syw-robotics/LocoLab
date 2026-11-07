"""Term-major observation histories with device-side reset and per-frame corruption."""

from types import SimpleNamespace

import warp as wp

from isaaclab.managers import ObservationTermCfg
from isaaclab_experimental.managers import ManagerTermBase, ObservationManager


@wp.kernel(enable_backward=False)
def _reset_history(mask: wp.array(dtype=bool), fresh: wp.array(dtype=bool)):
    i = wp.tid()
    if not mask or mask[i]:
        fresh[i] = True


@wp.kernel(enable_backward=False)
def _push_history(
    frame: wp.array2d(dtype=float),
    history: wp.array3d(dtype=float),
    fresh: wp.array(dtype=bool),
    rng: wp.array(dtype=wp.uint32),
    noise_low: float,
    noise_high: float,
    clip_low: float,
    clip_high: float,
    scale: float,
    update: bool,
    out: wp.array2d(dtype=float),
):
    i = wp.tid()
    state = rng[i]
    for j in range(frame.shape[1]):
        value = frame[i, j]
        if noise_low != 0.0 or noise_high != 0.0:
            value += wp.randf(state, noise_low, noise_high)
        value = wp.clamp(value, clip_low, clip_high) * scale
        if fresh[i]:
            for k in range(history.shape[1]):
                history[i, k, j] = value
        elif update:
            for k in range(history.shape[1] - 1):
                history[i, k, j] = history[i, k + 1, j]
            history[i, history.shape[1] - 1, j] = value
        for k in range(history.shape[1]):
            out[i, k * frame.shape[1] + j] = history[i, k, j]
    fresh[i] = False
    rng[i] = state


class ObservationHistory(ManagerTermBase):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.frame_dim = cfg._warp_frame_dim
        if self.frame_dim == "joint":
            self.frame_dim = cfg.params["asset_cfg"].joint_ids_wp.shape[0]
        elif self.frame_dim == "action":
            self.frame_dim = env.action_manager.total_action_dim
        # Z-RL reads this resolved frame size for time slicing and symmetry.
        cfg._warp_frame_dim = self.frame_dim
        self.history_length = cfg._warp_history_length
        self._source = cfg._warp_source
        self._params = {name: cfg.params[name] for name in cfg._warp_param_names}
        self._frame = wp.zeros((env.num_envs, self.frame_dim), dtype=float, device=env.device)
        self._history = wp.zeros((env.num_envs, self.history_length, self.frame_dim), dtype=float, device=env.device)
        self._fresh = wp.ones(env.num_envs, dtype=bool, device=env.device)
        self._descriptor = SimpleNamespace(out_dim=self.frame_dim * self.history_length)

    def reset(self, env_mask=None):
        wp.launch(_reset_history, dim=self.num_envs, inputs=[env_mask, self._fresh], device=self.device)

    def __call__(self, env, out, asset_cfg=None, command_name=None):
        self._source(env, self._frame, **self._params)
        wp.launch(
            _push_history,
            dim=self.num_envs,
            inputs=[
                self._frame,
                self._history,
                self._fresh,
                env.rng_state_wp,
                *self.cfg._warp_noise,
                *self.cfg._warp_clip,
                self.cfg._warp_scale,
                env.observation_manager._update_history,
                out,
            ],
            device=self.device,
        )


class HistoryObservationManager(ObservationManager):
    def compute_group(self, group_name, update_history=True):
        self._update_history = update_history
        return super().compute_group(group_name, update_history=update_history)


def configure_history(cfg):
    """Preserve velocity histories while disabling the unsupported upstream history path."""
    dimensions = {
        "base_ang_vel": 3,
        "projected_gravity": 3,
        "velocity_commands": 3,
        "task_command": 5,
        "joint_pos": "joint",
        "joint_vel": "joint",
        "actions": "action",
    }
    for group in vars(cfg.observations).values():
        for name, term in vars(group).items():
            if not isinstance(term, ObservationTermCfg):
                continue
            window = group.history_length if group.history_length is not None else term.history_length
            if window <= 0:
                continue
            if not group.flatten_history_dim or term.tensor_processors:
                raise ValueError("LocoLab Warp history requires flattened vector terms without tensor processors.")
            term._warp_history_length = window
            term._warp_frame_dim = dimensions[name]
            term._warp_source = term.func
            term._warp_param_names = tuple(term.params)
            term._warp_noise = (0.0, 0.0)
            if group.enable_corruption and term.noise is not None:
                from isaaclab_experimental.utils.noise import UniformNoiseCfg

                if not isinstance(term.noise, UniformNoiseCfg) or term.noise.operation != "add":
                    raise ValueError("LocoLab Warp history supports additive uniform observation noise.")
                term._warp_noise = (term.noise.n_min, term.noise.n_max)
            term._warp_clip = term.clip or (-float("inf"), float("inf"))
            term._warp_scale = float(term.scale) if term.scale is not None else 1.0
            term.func = ObservationHistory
            term.history_length = 0
            term.noise = term.clip = term.scale = None
        group.history_length = 0
