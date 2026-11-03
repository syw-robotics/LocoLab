"""G1 tracking terms reading shared clip buffers directly from Warp."""

from types import SimpleNamespace

import warp as wp

from isaaclab.utils.warp.launch_cache import _WarpLaunchCache
from isaaclab_experimental.managers import ManagerTermBase


@wp.func
def _rotation_error(reference: wp.quat, actual: wp.quat):
    delta = reference * wp.quat_inverse(actual)
    return 2.0 * wp.atan2(wp.length(wp.vec3(delta[0], delta[1], delta[2])), wp.abs(delta[3]))


class _MotionTerm(ManagerTermBase):
    # LocoLabWarpEnv refreshes guarded state before manager graph replay.

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        command = env.command_manager.get_term(cfg.params["command_name"])
        self._asset = command.robot
        self._frames = wp.from_torch(command.time_steps, dtype=wp.int64)
        self._origins = wp.from_torch(env.scene.env_origins, dtype=wp.vec3)
        self._body_ids = wp.array(command._body_ids, dtype=int, device=env.device)
        self._anchor = command.robot_anchor_body_index
        self._reference_anchor = command.motion_anchor_body_index
        self._reference_pos = wp.from_torch(command.motion.body_pos_w, dtype=wp.vec3)
        self._reference_ori = wp.from_torch(command.motion.body_quat_w, dtype=wp.quat)
        self._relative_pos = wp.from_torch(command.body_pos_relative_w, dtype=wp.vec3)
        self._relative_ori = wp.from_torch(command.body_quat_relative_w, dtype=wp.quat)
        names = cfg.params.get("body_names")
        indexes = list(range(command.num_bodies)) if names is None else [command.cfg.body_names.index(n) for n in names]
        self._indexes = wp.array(indexes, dtype=int, device=env.device)
        self._command = command
        self._pose = self._asset.data.body_link_pose_w.warp
        self._launch_cache = _WarpLaunchCache(env.device)
        self._launch_signature = None

    def _launch(self, kernel, dim, inputs, signature):
        # Record eager launches only while live inputs/output storage and scalar
        # parameters match. Raw-term checks and config changes can replace them.
        if signature != self._launch_signature:
            self._launch_cache.clear()
            self._launch_signature = signature
        self._launch_cache.launch("compute", wp.launch, kernel, dim=dim, inputs=inputs)


@wp.kernel(enable_backward=False)
def _command_frame(
    frames: wp.array(dtype=wp.int64), position: wp.array2d(dtype=float), velocity: wp.array2d(dtype=float),
    out: wp.array2d(dtype=float),
):
    i, j = wp.tid()
    if j < position.shape[1]:
        out[i, j] = position[frames[i], j]
    else:
        out[i, j] = velocity[frames[i], j - position.shape[1]]


class generated_commands(ManagerTermBase):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        command = env.command_manager.get_term(cfg.params["command_name"])
        self._frames = wp.from_torch(command.time_steps, dtype=wp.int64)
        self._position = wp.from_torch(command.motion.joint_pos)
        self._velocity = wp.from_torch(command.motion.joint_vel)
        self._descriptor = SimpleNamespace(out_dim=2 * self._position.shape[1])

    def __call__(self, env, out, command_name):
        wp.launch(_command_frame, dim=out.shape,
                  inputs=[self._frames, self._position, self._velocity, out], device=self.device)


@wp.kernel(enable_backward=False)
def _observation(
    pose: wp.array2d(dtype=wp.transform), reference_pos: wp.array2d(dtype=wp.vec3),
    reference_ori: wp.array2d(dtype=wp.quat), frames: wp.array(dtype=wp.int64),
    origins: wp.array(dtype=wp.vec3), body_ids: wp.array(dtype=int), anchor: int, reference_anchor: int,
    kind: int, out: wp.array2d(dtype=float),
):
    i, body_index = wp.tid()
    anchor_pose = pose[i, anchor]
    anchor_pos = wp.transform_get_translation(anchor_pose)
    anchor_ori = wp.transform_get_rotation(anchor_pose)
    position = wp.vec3(0.0)
    orientation = wp.quat_identity()
    if kind < 2:
        position = reference_pos[frames[i], reference_anchor] + origins[i]
        orientation = reference_ori[frames[i], reference_anchor]
    else:
        body_pose = pose[i, body_ids[body_index]]
        position = wp.transform_get_translation(body_pose)
        orientation = wp.transform_get_rotation(body_pose)
    if kind == 0 or kind == 2:
        relative = wp.quat_rotate_inv(anchor_ori, position - anchor_pos)
        for axis in range(3):
            out[i, body_index * 3 + axis] = relative[axis]
    else:
        matrix = wp.quat_to_matrix(wp.normalize(wp.quat_inverse(anchor_ori) * orientation))
        for axis in range(6):
            out[i, body_index * 6 + axis] = matrix[axis // 2, axis % 2]


class _MotionObservation(_MotionTerm):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        dimension = (3, 6, 3 * self._command.num_bodies, 6 * self._command.num_bodies)[self._kind]
        self._descriptor = SimpleNamespace(out_dim=dimension)

    def __call__(self, env, out, command_name):
        if not wp.get_device(self.device).is_capturing:
            self._pose = self._asset.data.body_link_pose_w.warp
        pose = self._pose
        self._launch(_observation, dim=(self.num_envs, 1 if self._kind < 2 else self._command.num_bodies), inputs=[
            pose, self._reference_pos, self._reference_ori,
            self._frames, self._origins, self._body_ids, self._anchor, self._reference_anchor, self._kind, out,
        ], signature=(pose.ptr, out.ptr, out.shape, out.strides))


class motion_anchor_pos_b(_MotionObservation):
    _kind = 0


class motion_anchor_ori_b(_MotionObservation):
    _kind = 1


class robot_body_pos_b(_MotionObservation):
    _kind = 2


class robot_body_ori_b(_MotionObservation):
    _kind = 3


@wp.kernel(enable_backward=False)
def _tracking_reward(
    pose: wp.array2d(dtype=wp.transform), velocity: wp.array2d(dtype=wp.spatial_vector),
    reference_pos: wp.array2d(dtype=wp.vec3), reference_ori: wp.array2d(dtype=wp.quat),
    reference_velocity: wp.array2d(dtype=wp.vec3), relative_pos: wp.array2d(dtype=wp.vec3),
    relative_ori: wp.array2d(dtype=wp.quat), frames: wp.array(dtype=wp.int64),
    origins: wp.array(dtype=wp.vec3), body_ids: wp.array(dtype=int), indexes: wp.array(dtype=int),
    anchor: int, reference_anchor: int, kind: int, inv_variance: float, out: wp.array(dtype=float),
):
    i = wp.tid()
    error = float(0.0)
    if kind == 0:
        delta = reference_pos[frames[i], reference_anchor] + origins[i] - wp.transform_get_translation(pose[i, anchor])
        error = wp.dot(delta, delta)
    elif kind == 1:
        angle = _rotation_error(reference_ori[frames[i], reference_anchor], wp.transform_get_rotation(pose[i, anchor]))
        error = angle * angle
    else:
        for k in range(indexes.shape[0]):
            j = indexes[k]
            body = body_ids[j]
            if kind == 2:
                delta = relative_pos[i, j] - wp.transform_get_translation(pose[i, body])
                error += wp.dot(delta, delta)
            elif kind == 3:
                angle = _rotation_error(relative_ori[i, j], wp.transform_get_rotation(pose[i, body]))
                error += angle * angle
            else:
                v = velocity[i, body]
                actual = wp.vec3(v[0], v[1], v[2])
                if kind == 5:
                    actual = wp.vec3(v[3], v[4], v[5])
                delta = reference_velocity[frames[i], j] - actual
                error += wp.dot(delta, delta)
        error /= float(indexes.shape[0])
    out[i] = wp.exp(-error * inv_variance)


class _MotionReward(_MotionTerm):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._reference_velocity = None
        self._velocity = self._asset.data.body_link_vel_w.warp if self._kind >= 4 else None
        if self._kind == 4:
            self._reference_velocity = wp.from_torch(self._command.motion.body_lin_vel_w, dtype=wp.vec3)
        elif self._kind == 5:
            self._reference_velocity = wp.from_torch(self._command.motion.body_ang_vel_w, dtype=wp.vec3)

    def __call__(self, env, out, command_name, std, body_names=None):
        data = self._asset.data
        if not wp.get_device(self.device).is_capturing:
            self._pose = data.body_link_pose_w.warp
            if self._kind >= 4:
                self._velocity = data.body_link_vel_w.warp
        pose, velocity = self._pose, self._velocity
        self._launch(_tracking_reward, dim=self.num_envs, inputs=[
            pose, velocity, self._reference_pos, self._reference_ori,
            self._reference_velocity, self._relative_pos, self._relative_ori, self._frames, self._origins,
            self._body_ids, self._indexes, self._anchor, self._reference_anchor, self._kind, 1.0 / std**2, out,
        ], signature=(pose.ptr, velocity.ptr if velocity is not None else None, out.ptr, out.shape, out.strides, std))


class motion_global_anchor_position_error_exp(_MotionReward):
    _kind = 0

    def __call__(self, env, out, command_name, std):
        super().__call__(env, out, command_name, std)


class motion_global_anchor_orientation_error_exp(_MotionReward):
    _kind = 1

    def __call__(self, env, out, command_name, std):
        super().__call__(env, out, command_name, std)


class motion_relative_body_position_error_exp(_MotionReward):
    _kind = 2


class motion_relative_body_orientation_error_exp(_MotionReward):
    _kind = 3


class motion_global_body_linear_velocity_error_exp(_MotionReward):
    _kind = 4


class motion_global_body_angular_velocity_error_exp(_MotionReward):
    _kind = 5


@wp.kernel(enable_backward=False)
def _termination(
    pose: wp.array2d(dtype=wp.transform), reference_pos: wp.array2d(dtype=wp.vec3),
    reference_ori: wp.array2d(dtype=wp.quat), relative_pos: wp.array2d(dtype=wp.vec3),
    frames: wp.array(dtype=wp.int64), origins: wp.array(dtype=wp.vec3),
    body_ids: wp.array(dtype=int), indexes: wp.array(dtype=int), anchor: int, reference_anchor: int,
    kind: int, threshold: float, out: wp.array(dtype=bool),
):
    i = wp.tid()
    anchor_pose = pose[i, anchor]
    failed = bool(False)
    if kind == 0:
        target = reference_pos[frames[i], reference_anchor] + origins[i]
        failed = wp.abs(target[2] - wp.transform_get_translation(anchor_pose)[2]) > threshold
    elif kind == 1:
        gravity = wp.vec3(0.0, 0.0, -1.0)
        target = wp.quat_rotate_inv(reference_ori[frames[i], reference_anchor], gravity)
        actual = wp.quat_rotate_inv(wp.transform_get_rotation(anchor_pose), gravity)
        failed = wp.abs(target[2] - actual[2]) > threshold
    else:
        for k in range(indexes.shape[0]):
            j = indexes[k]
            actual = wp.transform_get_translation(pose[i, body_ids[j]])
            failed = failed or wp.abs(relative_pos[i, j][2] - actual[2]) > threshold
    out[i] = failed


class _MotionTermination(_MotionTerm):
    def __call__(self, env, out, command_name, threshold, body_names=None):
        if not wp.get_device(self.device).is_capturing:
            self._pose = self._asset.data.body_link_pose_w.warp
        pose = self._pose
        self._launch(_termination, dim=self.num_envs, inputs=[
            pose, self._reference_pos, self._reference_ori, self._relative_pos,
            self._frames, self._origins, self._body_ids, self._indexes, self._anchor, self._reference_anchor,
            self._kind, threshold, out,
        ], signature=(pose.ptr, out.ptr, out.shape, out.strides, threshold))


class bad_anchor_pos_z_only(_MotionTermination):
    _kind = 0

    def __call__(self, env, out, command_name, threshold):
        super().__call__(env, out, command_name, threshold)


class bad_anchor_ori(_MotionTermination):
    _kind = 1

    def __call__(self, env, out, command_name, threshold):
        super().__call__(env, out, command_name, threshold)


class bad_motion_body_pos_z_only(_MotionTermination):
    _kind = 2


@wp.kernel(enable_backward=False)
def _collision_pairs(history: wp.array4d(dtype=wp.vec3), threshold: float, hit: wp.array2d(dtype=int)):
    i, h, body, other = wp.tid()
    if wp.length(history[i, h, body, other]) > threshold:
        wp.atomic_max(hit, i, h, 1)


@wp.kernel(enable_backward=False)
def _collision_count(hit: wp.array2d(dtype=int), out: wp.array(dtype=float)):
    i = wp.tid()
    count = int(0)
    for h in range(hit.shape[1]):
        count += hit[i, h]
    out[i] += float(count)


class self_collision_cost(ManagerTermBase):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._sensors = [env.scene[name] for name in cfg.params["sensor_names"]]
        self._hits = []
        self._histories = []
        for sensor in self._sensors:
            history = sensor.data.normal_force_matrix_w_history
            self._histories.append(None if history is None else history.warp)
            self._hits.append(
                None if history is None else wp.zeros(history.warp.shape[:2], dtype=int, device=self.device)
            )

    def __call__(self, env, out, sensor_names, force_threshold=10.0):
        out.zero_()
        for index, (sensor, hit) in enumerate(zip(self._sensors, self._hits)):
            if hit is not None:
                if not wp.get_device(self.device).is_capturing:
                    self._histories[index] = sensor.data.normal_force_matrix_w_history.warp
                history = self._histories[index]
                hit.zero_()
                wp.launch(
                    _collision_pairs, dim=history.shape, inputs=[history, force_threshold, hit], device=self.device
                )
                wp.launch(_collision_count, dim=self.num_envs, inputs=[hit, out], device=self.device)
