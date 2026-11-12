"""Capture-safe physics-step action delay and encoder calibration."""

import warp as wp

from isaaclab_experimental.envs.mdp.actions.joint_actions import JointPositionAction
from isaaclab_experimental.managers.action_manager import ActionTerm


@wp.kernel(enable_backward=False)
def _subtract_bias(
    command: wp.array2d(dtype=float),
    bias: wp.array2d(dtype=float),
    ids: wp.array(dtype=int),
    out: wp.array2d(dtype=float),
):
    i, j = wp.tid()
    joint = j
    if ids:
        joint = ids[j]
    out[i, j] = command[i, j] - bias[i, joint]


class EncoderBiasJointPositionAction(JointPositionAction):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._bias = wp.zeros((self.num_envs, self._asset.num_joints), dtype=float, device=self.device)
        self._target = wp.zeros_like(self._processed_actions)
        self._asset.encoder_bias_wp = self._bias
        self._asset.encoder_bias = wp.to_torch(self._bias)

    def apply_actions(self):
        wp.launch(
            _subtract_bias,
            dim=(self.num_envs, self.action_dim),
            inputs=[self._processed_actions, self._bias, self._joint_ids_wp, self._target],
            device=self.device,
        )
        self._asset.set_joint_position_target_index(target=self._target, joint_ids=self._joint_ids_wp)


@wp.kernel(enable_backward=False)
def _process_tracking_action(
    actions: wp.array2d(dtype=float), action_offset: int, scale: wp.array(dtype=float),
    offset: wp.array(dtype=float), clip: wp.array2d(dtype=float), ids: wp.array(dtype=int),
    bias: wp.array2d(dtype=float), raw: wp.array2d(dtype=float), processed: wp.array2d(dtype=float),
    target: wp.array2d(dtype=float),
):
    i, j = wp.tid()
    value = actions[i, action_offset + j]
    raw[i, j] = value
    value = wp.clamp(value * scale[j] + offset[j], clip[j, 0], clip[j, 1])
    processed[i, j] = value
    joint = j
    if ids:
        joint = ids[j]
    target[i, j] = value - bias[i, joint]


class TrackingJointPositionAction(EncoderBiasJointPositionAction):
    """Fuse encoder calibration with processing once per control step.

    Tracking holds this target through physics decimation. Delayed quadruped
    actions retain the base implementation, which calibrates each delayed target.
    """

    def process_actions(self, actions, action_offset=0):
        wp.launch(_process_tracking_action, dim=(self.num_envs, self.action_dim), inputs=[
            actions, action_offset, self._scale, self._offset, self._clip, self._joint_ids_wp,
            self._bias, self._raw_actions, self._processed_actions, self._target,
        ], device=self.device)

    def apply_actions(self):
        self._asset.set_joint_position_target_index(target=self._target, joint_ids=self._joint_ids_wp)


@wp.kernel(enable_backward=False)
def _delay_reset(
    mask: wp.array(dtype=bool),
    rng: wp.array(dtype=wp.uint32),
    low: int,
    high: int,
    command: wp.array2d(dtype=float),
    history: wp.array3d(dtype=float),
    delay: wp.array(dtype=int),
):
    i = wp.tid()
    if not mask or mask[i]:
        state = rng[i]
        delay[i] = wp.randi(state, low, high + 1)
        rng[i] = state
        for k in range(history.shape[1]):
            for j in range(history.shape[2]):
                history[i, k, j] = command[i, j]


@wp.kernel(enable_backward=False)
def _delay_apply(
    command: wp.array2d(dtype=float),
    history: wp.array3d(dtype=float),
    delay: wp.array(dtype=int),
    out: wp.array2d(dtype=float),
):
    i, j = wp.tid()
    for k in range(history.shape[1] - 1, 0, -1):
        history[i, k, j] = history[i, k - 1, j]
    history[i, 0, j] = command[i, j]
    out[i, j] = history[i, delay[i], j]


class DelayedAction(ActionTerm):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        low, high = cfg.delay_range
        if not 0 <= low <= high or high <= 0:
            raise ValueError(f"Invalid delay_range: {cfg.delay_range}")
        self._term = cfg.action.class_type(cfg.action, env)
        # Z-RL's symmetry compiler reads these once as Torch metadata; processing stays Warp-native.
        self._scale = wp.to_torch(self._term._scale)
        self._offset = wp.to_torch(self._term._offset)
        self.action_history = wp.zeros((self.num_envs, high + 1, self.action_dim), dtype=float, device=self.device)
        self.delay_steps = wp.zeros(self.num_envs, dtype=int, device=self.device)
        self._delayed = wp.zeros_like(self._term.processed_actions)

    def __getattr__(self, name):
        term = self.__dict__.get("_term")
        if term is not None:
            return getattr(term, name)
        raise AttributeError(name)

    @property
    def action_dim(self):
        return self._term.action_dim

    @property
    def raw_actions(self):
        return self._term.raw_actions

    @property
    def processed_actions(self):
        return self._term.processed_actions

    def set_debug_vis(self, debug_vis):
        term = self.__dict__.get("_term")
        return term.set_debug_vis(debug_vis) if term is not None else False

    def reset(self, env_mask=None):
        self._term.reset(env_mask)
        wp.launch(
            _delay_reset,
            dim=self.num_envs,
            inputs=[
                env_mask,
                self._env.rng_state_wp,
                *self.cfg.delay_range,
                self._term.processed_actions,
                self.action_history,
                self.delay_steps,
            ],
            device=self.device,
        )

    def process_actions(self, actions, action_offset=0):
        self._term.process_actions(actions, action_offset)

    def apply_actions(self):
        wp.launch(
            _delay_apply,
            dim=(self.num_envs, self.action_dim),
            inputs=[self._term.processed_actions, self.action_history, self.delay_steps, self._delayed],
            device=self.device,
        )
        command = self._term._processed_actions
        self._term._processed_actions = self._delayed
        try:
            self._term.apply_actions()
        finally:
            self._term._processed_actions = command
