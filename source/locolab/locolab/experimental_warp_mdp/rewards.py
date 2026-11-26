"""Warp equivalents preserving LocoLab's velocity reward definitions."""

import warp as wp

from isaaclab_experimental.envs.mdp.rewards import (
    ang_vel_xy_l2 as _root_ang_vel_xy_l2,
    flat_orientation_l2 as _root_flat_orientation_l2,
    _track_ang_vel_z_exp_kernel,
    _track_lin_vel_xy_exp_kernel,
)
from isaaclab_experimental.managers import ManagerTermBase, SceneEntityCfg
from isaaclab_tasks_experimental.core.velocity.mdp.rewards import (
    _track_lin_vel_xy_yaw_frame_exp_kernel,
    _track_ang_vel_z_world_exp_kernel,
)

from locolab.tasks.manager_based.locomotion.velocity.mdp.rewards import (
    _posture_inverse_variances,
    base_height_l2 as _torch_base_height_l2,
)


def track_lin_vel_xy_exp(env, out, std, command_name, asset_cfg=SceneEntityCfg("robot")):
    data = env.scene[asset_cfg.name].data
    wp.launch(
        _track_lin_vel_xy_exp_kernel,
        dim=env.num_envs,
        inputs=[
            data.root_link_pose_w.warp,
            data.root_com_vel_w.warp,
            wp.from_torch(env.command_manager.get_command(command_name)),
            1.0 / std ** 2,
            out,
        ],
        device=env.device,
    )


def track_ang_vel_z_exp(env, out, std, command_name, asset_cfg=SceneEntityCfg("robot")):
    data = env.scene[asset_cfg.name].data
    wp.launch(
        _track_ang_vel_z_exp_kernel,
        dim=env.num_envs,
        inputs=[
            data.root_link_pose_w.warp,
            data.root_com_vel_w.warp,
            wp.from_torch(env.command_manager.get_command(command_name)),
            2,
            1.0 / std ** 2,
            out,
        ],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _height(
    pose: wp.array(dtype=wp.transform), rays: wp.array2d(dtype=wp.vec3),
    ray_indices: wp.array(dtype=int), target: float, out: wp.array(dtype=float),
):
    i = wp.tid()
    terrain = float(0.0)
    if ray_indices:
        for j in range(ray_indices.shape[0]):
            terrain += wp.clamp(rays[i, ray_indices[j]][2], -10.0, 10.0)
        terrain /= float(ray_indices.shape[0])
    elif rays:
        for j in range(rays.shape[1]):
            terrain += wp.clamp(rays[i, j][2], -10.0, 10.0)
        terrain /= float(rays.shape[1])
    error = wp.transform_get_translation(pose[i])[2] - target - terrain
    out[i] = error * error


class base_height_l2(ManagerTermBase):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        asset_cfg = cfg.params["asset_cfg"]
        sensor_cfg = cfg.params["sensor_cfg"]
        self._asset = env.scene[asset_cfg.name]
        self._sensor = env.scene[sensor_cfg.name] if sensor_cfg is not None else None
        self._ray_indices = None
        if (size := cfg.params["center_scan_size"]) is not None:
            indices = _torch_base_height_l2._center_height_scan_indices(self._sensor, size)
            self._ray_indices = wp.array(indices.tolist(), dtype=int, device=env.device)

    def __call__(self, env, out, target_height, asset_cfg=SceneEntityCfg("robot"), sensor_cfg=None, center_scan_size=None):
        rays = self._sensor.data.ray_hits_w.warp if self._sensor is not None else None
        wp.launch(
            _height,
            dim=env.num_envs,
            inputs=[self._asset.data.root_link_pose_w.warp, rays, self._ray_indices, target_height, out],
            device=env.device,
        )


@wp.kernel(enable_backward=False)
def _power(
    velocity: wp.array2d(dtype=float),
    effort: wp.array2d(dtype=float),
    ids: wp.array(dtype=int),
    out: wp.array(dtype=float),
):
    i = wp.tid()
    total = float(0.0)
    for k in range(ids.shape[0]):
        j = ids[k]
        total += wp.abs(velocity[i, j] * effort[i, j])
    out[i] = total


def joint_power_l1(env, out, asset_cfg=SceneEntityCfg("robot")):
    asset = env.scene[asset_cfg.name]
    wp.launch(
        _power,
        dim=env.num_envs,
        inputs=[asset.data.joint_vel.warp, asset.actuators.applied_effort.warp, asset_cfg.joint_ids_wp, out],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _variable_posture(
    position: wp.array2d(dtype=float), default_position: wp.array2d(dtype=float),
    command: wp.array2d(dtype=float), inverse_variance: wp.array(dtype=wp.vec3),
    walking_threshold: float, running_threshold: float, out: wp.array(dtype=float),
):
    i = wp.tid()
    speed = wp.length(wp.vec2(command[i, 0], command[i, 1])) + wp.abs(command[i, 2])
    regime = int(speed >= walking_threshold) + int(speed >= running_threshold)
    error = float(0.0)
    for j in range(position.shape[1]):
        delta = position[i, j] - default_position[i, j]
        error += delta * delta * inverse_variance[j][regime]
    out[i] = wp.exp(-error / float(position.shape[1]))


class variable_posture(ManagerTermBase):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        params = cfg.params
        asset = env.scene[params["asset_cfg"].name]
        values = _posture_inverse_variances(asset.joint_names, params["std_standing"], params["std_moving"])
        self._inverse_variance = wp.array(values, dtype=wp.vec3, device=env.device)

    def __call__(self, env, out, asset_cfg, command_name, std_standing, std_moving, walking_threshold, running_threshold):
        data = env.scene[asset_cfg.name].data
        wp.launch(
            _variable_posture, dim=env.num_envs,
            inputs=[
                data.joint_pos.warp, data.default_joint_pos.warp,
                wp.from_torch(env.command_manager.get_command(command_name)), self._inverse_variance,
                walking_threshold, running_threshold, out,
            ],
            device=env.device,
        )


@wp.kernel(enable_backward=False)
def _stand(
    position: wp.array2d(dtype=float),
    defaults: wp.array2d(dtype=float),
    standing: wp.array(dtype=bool),
    ids: wp.array(dtype=int),
    out: wp.array(dtype=float),
):
    i = wp.tid()
    total = float(0.0)
    for k in range(ids.shape[0]):
        j = ids[k]
        difference = position[i, j] - defaults[i, j]
        total += difference * difference
    out[i] = wp.where(standing[i], wp.sqrt(total), 0.0)


def stand_still(env, out, command_name, asset_cfg=SceneEntityCfg("robot")):
    data = env.scene[asset_cfg.name].data
    wp.launch(
        _stand,
        dim=env.num_envs,
        inputs=[
            data.joint_pos.warp,
            data.default_joint_pos.warp,
            wp.from_torch(env.command_manager.get_term(command_name).is_standing_env),
            asset_cfg.joint_ids_wp,
            out,
        ],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _air_time(
    air: wp.array2d(dtype=float),
    first_contact: wp.array2d(dtype=float),
    standing: wp.array(dtype=bool),
    ids: wp.array(dtype=int),
    threshold: float,
    out: wp.array(dtype=float),
):
    i = wp.tid()
    total = float(0.0)
    for k in range(ids.shape[0]):
        j = ids[k]
        total += (air[i, j] - threshold) * first_contact[i, j]
    out[i] = wp.where(standing[i], 0.0, total)


def feet_air_time(env, out, command_name, sensor_cfg, threshold):
    sensor = env.scene.sensors[sensor_cfg.name]
    wp.launch(
        _air_time,
        dim=env.num_envs,
        inputs=[
            sensor.data.last_air_time.warp,
            sensor.compute_first_contact(env.step_dt).warp,
            wp.from_torch(env.command_manager.get_term(command_name).is_standing_env),
            sensor_cfg.body_ids_wp,
            threshold,
            out,
        ],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _stumble(forces: wp.array2d(dtype=wp.vec3), ids: wp.array(dtype=int), out: wp.array(dtype=float)):
    i = wp.tid()
    hit = bool(False)
    for k in range(ids.shape[0]):
        f = forces[i, ids[k]]
        hit = hit or wp.length(wp.vec2(f[0], f[1])) > 4.0 * wp.abs(f[2])
    out[i] = float(hit)


def feet_stumble(env, out, sensor_cfg):
    wp.launch(
        _stumble,
        dim=env.num_envs,
        inputs=[env.scene.sensors[sensor_cfg.name].data.net_forces_w.warp, sensor_cfg.body_ids_wp, out],
        device=env.device,
    )


@wp.func
def _contact_count(forces: wp.array3d(dtype=wp.vec3), ids: wp.array(dtype=int), threshold: float, i: int):
    count = int(0)
    for k in range(ids.shape[0]):
        hit = bool(False)
        for h in range(forces.shape[1]):
            hit = hit or wp.length(forces[i, h, ids[k]]) > threshold
        count += int(hit)
    return count


@wp.kernel(enable_backward=False)
def _undesired_contacts(
    forces: wp.array3d(dtype=wp.vec3), ids: wp.array(dtype=int), threshold: float, out: wp.array(dtype=float)
):
    i = wp.tid()
    out[i] = float(_contact_count(forces, ids, threshold, i))


def undesired_contacts(env, out, threshold, sensor_cfg):
    wp.launch(
        _undesired_contacts,
        dim=env.num_envs,
        inputs=[
            env.scene.sensors[sensor_cfg.name].data.net_forces_w_history.warp,
            sensor_cfg.body_ids_wp,
            threshold,
            out,
        ],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _clearance(
    pose: wp.array2d(dtype=wp.transform),
    velocity: wp.array2d(dtype=wp.spatial_vector),
    ids: wp.array(dtype=int),
    target: float,
    std: float,
    multiplier: float,
    out: wp.array(dtype=float),
):
    i = wp.tid()
    error = float(0.0)
    for k in range(ids.shape[0]):
        j = ids[k]
        height = wp.max(target - wp.transform_get_translation(pose[i, j])[2], 0.0)
        v = wp.spatial_top(velocity[i, j])
        speed = wp.length(wp.vec2(v[0], v[1]))
        error += height * height * wp.tanh(multiplier * speed)
    out[i] = wp.exp(-error / std)


def feet_clearance_flat(env, out, asset_cfg, target_height, std, tanh_mult):
    data = env.scene[asset_cfg.name].data
    wp.launch(
        _clearance,
        dim=env.num_envs,
        inputs=[
            data.body_link_pose_w.warp,
            data.body_com_vel_w.warp,
            asset_cfg.body_ids_wp,
            target_height,
            std,
            tanh_mult,
            out,
        ],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _feet_swing_height_l2(
    pose: wp.array2d(dtype=wp.transform), velocity: wp.array2d(dtype=wp.spatial_vector),
    air_time: wp.array2d(dtype=float), standing: wp.array(dtype=bool),
    body_ids: wp.array(dtype=int), sensor_ids: wp.array(dtype=int), target: float, out: wp.array(dtype=float),
):
    i = wp.tid()
    error = float(0.0)
    if not standing[i]:
        for k in range(sensor_ids.shape[0]):
            if air_time[i, sensor_ids[k]] > 0.0:
                j = body_ids[k]
                height_error = wp.transform_get_translation(pose[i, j])[2] - target
                v = wp.spatial_top(velocity[i, j])
                error += height_error * height_error * wp.length(wp.vec2(v[0], v[1]))
    out[i] = error


def feet_swing_height_l2(env, out, command_name, sensor_cfg, asset_cfg, target_height):
    data = env.scene[asset_cfg.name].data
    sensor = env.scene.sensors[sensor_cfg.name]
    wp.launch(
        _feet_swing_height_l2, dim=env.num_envs,
        inputs=[
            data.body_link_pose_w.warp, data.body_com_vel_w.warp, sensor.data.current_air_time.warp,
            wp.from_torch(env.command_manager.get_term(command_name).is_standing_env), asset_cfg.body_ids_wp,
            sensor_cfg.body_ids_wp, target_height, out,
        ],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _slide(
    velocity: wp.array2d(dtype=wp.spatial_vector),
    force_history: wp.array3d(dtype=wp.vec3),
    body_ids: wp.array(dtype=int),
    sensor_ids: wp.array(dtype=int),
    out: wp.array(dtype=float),
):
    i = wp.tid()
    total = float(0.0)
    for k in range(sensor_ids.shape[0]):
        contact = bool(False)
        for h in range(force_history.shape[1]):
            contact = contact or wp.length(force_history[i, h, sensor_ids[k]]) > 1.0
        v = wp.spatial_top(velocity[i, body_ids[k]])
        total += wp.where(contact, wp.length(wp.vec2(v[0], v[1])), 0.0)
    out[i] = total


@wp.kernel(enable_backward=False)
def _landing_vel_z(
    velocity: wp.array2d(dtype=wp.spatial_vector),
    first_contact: wp.array2d(dtype=float),
    body_ids: wp.array(dtype=int),
    sensor_ids: wp.array(dtype=int),
    out: wp.array(dtype=float),
):
    i = wp.tid()
    total = float(0.0)
    for k in range(sensor_ids.shape[0]):
        vz = wp.min(wp.spatial_top(velocity[i, body_ids[k]])[2], 0.0)
        total += vz * vz * first_contact[i, sensor_ids[k]]
    out[i] = total


def feet_landing_vel_z_l2(env, out, sensor_cfg, asset_cfg):
    sensor = env.scene.sensors[sensor_cfg.name]
    wp.launch(
        _landing_vel_z,
        dim=env.num_envs,
        inputs=[
            env.scene[asset_cfg.name].data.body_com_vel_w.warp,
            sensor.compute_first_contact(env.step_dt).warp,
            asset_cfg.body_ids_wp,
            sensor_cfg.body_ids_wp,
            out,
        ],
        device=env.device,
    )


def feet_slide(env, out, sensor_cfg, asset_cfg=SceneEntityCfg("robot")):
    wp.launch(
        _slide,
        dim=env.num_envs,
        inputs=[
            env.scene[asset_cfg.name].data.body_com_vel_w.warp,
            env.scene.sensors[sensor_cfg.name].data.net_forces_w_history.warp,
            asset_cfg.body_ids_wp,
            sensor_cfg.body_ids_wp,
            out,
        ],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _smoothness(
    action: wp.array2d(dtype=float),
    previous: wp.array2d(dtype=float),
    previous_previous: wp.array2d(dtype=float),
    steps: wp.array(dtype=wp.int64),
    out: wp.array(dtype=float),
):
    i = wp.tid()
    penalty = float(0.0)
    for j in range(action.shape[1]):
        difference = action[i, j] - 2.0 * previous[i, j] + previous_previous[i, j]
        penalty += difference * difference
        previous_previous[i, j] = previous[i, j]
    out[i] = wp.where(steps[i] < wp.int64(3), 0.0, penalty)


@wp.kernel(enable_backward=False)
def _reset_smoothness(mask: wp.array(dtype=bool), history: wp.array2d(dtype=float)):
    i, j = wp.tid()
    if not mask or mask[i]:
        history[i, j] = 0.0


class action_smoothness_l2(ManagerTermBase):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._history = wp.zeros_like(env.action_manager.action)

    def reset(self, env_mask=None):
        wp.launch(_reset_smoothness, dim=self._history.shape, inputs=[env_mask, self._history], device=self.device)

    def __call__(self, env, out):
        wp.launch(
            _smoothness,
            dim=env.num_envs,
            inputs=[
                env.action_manager.action,
                env.action_manager.prev_action,
                self._history,
                env._episode_length_buf_wp,
                out,
            ],
            device=env.device,
        )


def track_lin_vel_xy_yaw_frame_exp(env, out, std, command_name, asset_cfg=SceneEntityCfg("robot")):
    data = env.scene[asset_cfg.name].data
    wp.launch(
        _track_lin_vel_xy_yaw_frame_exp_kernel,
        dim=env.num_envs,
        inputs=[data.root_quat_w.warp, data.root_lin_vel_w.warp,
                wp.from_torch(env.command_manager.get_command(command_name)), 1.0 / std ** 2, out],
        device=env.device,
    )


def track_ang_vel_z_world_exp(env, out, command_name, std, asset_cfg=SceneEntityCfg("robot")):
    wp.launch(
        _track_ang_vel_z_world_exp_kernel,
        dim=env.num_envs,
        inputs=[env.scene[asset_cfg.name].data.root_ang_vel_w.warp,
                wp.from_torch(env.command_manager.get_command(command_name)), 1.0 / std ** 2, out],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _body_ang_vel_xy(
    velocity: wp.array2d(dtype=wp.spatial_vector), ids: wp.array(dtype=int), out: wp.array(dtype=float)
):
    i = wp.tid()
    total = float(0.0)
    for k in range(ids.shape[0]):
        v = wp.spatial_bottom(velocity[i, ids[k]])
        total += v[0] * v[0] + v[1] * v[1]
    out[i] = total


def ang_vel_xy_l2(env, out, asset_cfg=SceneEntityCfg("robot"), select_bodies=False):
    if not select_bodies:
        return _root_ang_vel_xy_l2(env, out, asset_cfg)
    wp.launch(
        _body_ang_vel_xy, dim=env.num_envs,
        inputs=[env.scene[asset_cfg.name].data.body_com_vel_w.warp, asset_cfg.body_ids_wp, out],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _body_flat_orientation(
    pose: wp.array2d(dtype=wp.transform), gravity: wp.array(dtype=wp.vec3),
    ids: wp.array(dtype=int), contact: wp.array2d(dtype=float), contact_ids: wp.array(dtype=int),
    out: wp.array(dtype=float),
):
    i = wp.tid()
    total = float(0.0)
    for k in range(ids.shape[0]):
        if not contact or contact[i, contact_ids[k]] > 0.0:
            q = wp.transform_get_rotation(pose[i, ids[k]])
            g = wp.quat_rotate_inv(q, wp.normalize(gravity[i]))
            total += g[0] * g[0] + g[1] * g[1]
    out[i] = total


def flat_orientation_l2(env, out, asset_cfg=SceneEntityCfg("robot"), select_bodies=False, sensor_cfg=None):
    if not select_bodies:
        return _root_flat_orientation_l2(env, out, asset_cfg)
    data = env.scene[asset_cfg.name].data
    contact = None
    contact_ids = None
    if sensor_cfg is not None:
        contact = env.scene.sensors[sensor_cfg.name].data.current_contact_time.warp
        contact_ids = sensor_cfg.body_ids_wp
    wp.launch(
        _body_flat_orientation, dim=env.num_envs,
        inputs=[data.body_link_pose_w.warp, data.GRAVITY_VEC_W.warp, asset_cfg.body_ids_wp, contact, contact_ids, out],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _feet_too_near(
    pose: wp.array2d(dtype=wp.transform), ids: wp.array(dtype=int), threshold: float, out: wp.array(dtype=float)
):
    i = wp.tid()
    delta = wp.transform_get_translation(pose[i, ids[0]]) - wp.transform_get_translation(pose[i, ids[1]])
    out[i] = wp.max(threshold - wp.length(delta), 0.0)


def feet_too_near_bipedal(env, out, asset_cfg=SceneEntityCfg("robot"), threshold=0.15):
    wp.launch(
        _feet_too_near,
        dim=env.num_envs,
        inputs=[env.scene[asset_cfg.name].data.body_link_pose_w.warp, asset_cfg.body_ids_wp, threshold, out],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _feet_flight(contact: wp.array2d(dtype=float), ids: wp.array(dtype=int), out: wp.array(dtype=float)):
    i = wp.tid()
    contacts = int(0)
    for k in range(ids.shape[0]):
        contacts += int(contact[i, ids[k]] > 0.0)
    out[i] = wp.where(contacts == 0, 1.0, 0.0)


def feet_flight(env, out, sensor_cfg):
    wp.launch(
        _feet_flight, dim=env.num_envs,
        inputs=[env.scene.sensors[sensor_cfg.name].data.current_contact_time.warp, sensor_cfg.body_ids_wp, out],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _standing_contact(
    contact: wp.array2d(dtype=float), standing: wp.array(dtype=bool),
    ids: wp.array(dtype=int), out: wp.array(dtype=float)
):
    i = wp.tid()
    airborne = int(0)
    for k in range(ids.shape[0]):
        airborne += int(contact[i, ids[k]] <= 0.0)
    out[i] = wp.where(standing[i], float(airborne), 0.0)


def feet_standing_contact(env, out, command_name, sensor_cfg):
    wp.launch(
        _standing_contact, dim=env.num_envs,
        inputs=[env.scene.sensors[sensor_cfg.name].data.current_contact_time.warp,
                wp.from_torch(env.command_manager.get_term(command_name).is_standing_env),
                sensor_cfg.body_ids_wp, out],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _short_swing(
    air: wp.array2d(dtype=float), last_contact: wp.array2d(dtype=float),
    first_contact: wp.array2d(dtype=float), command: wp.array2d(dtype=float),
    standing: wp.array(dtype=bool),
    ids: wp.array(dtype=int), minimum: float, out: wp.array(dtype=float),
):
    i = wp.tid()
    total = float(0.0)
    for k in range(ids.shape[0]):
        j = ids[k]
        if last_contact[i, j] > 0.0:
            total += wp.max(1.0 - air[i, j] / minimum, 0.0) * first_contact[i, j]
    scale = wp.min(wp.length(wp.vec2(command[i, 0], command[i, 1])) / 0.5 + wp.abs(command[i, 2]) / 1.5, 1.0)
    out[i] = wp.where(standing[i], 0.0, total * scale)


def feet_short_swing(env, out, command_name, sensor_cfg, min_air_time):
    sensor = env.scene.sensors[sensor_cfg.name]
    term = env.command_manager.get_term(command_name)
    wp.launch(
        _short_swing, dim=env.num_envs,
        inputs=[sensor.data.last_air_time.warp, sensor.data.last_contact_time.warp,
                sensor.compute_first_contact(env.step_dt).warp,
                wp.from_torch(term.command), wp.from_torch(term.is_standing_env), sensor_cfg.body_ids_wp,
                min_air_time, out],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _feet_gait(
    contact: wp.array2d(dtype=float), steps: wp.array(dtype=wp.int64), offsets: wp.array(dtype=float),
    ids: wp.array(dtype=int), standing: wp.array(dtype=bool), dt: float, period: float,
    threshold: float, out: wp.array(dtype=float),
):
    i = wp.tid()
    total = float(0.0)
    if not standing or not standing[i]:
        phase = (float(steps[i]) * dt % period) / period
        for k in range(ids.shape[0]):
            in_stance = (phase + offsets[k]) % 1.0 < threshold
            if in_stance == (contact[i, ids[k]] > 0.0):
                total += 1.0
    out[i] = total


class feet_gait(ManagerTermBase):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._offsets = wp.array(cfg.params.get("offset", [0.0, 0.5]), dtype=float, device=env.device)

    def __call__(self, env, out, period, sensor_cfg, offset=None, threshold=0.5, command_name=None):
        sensor = env.scene.sensors[sensor_cfg.name]
        standing = None
        if command_name is not None:
            standing = wp.from_torch(env.command_manager.get_term(command_name).is_standing_env)
        wp.launch(
            _feet_gait, dim=env.num_envs,
            inputs=[
                sensor.data.current_contact_time.warp, env._episode_length_buf_wp, self._offsets,
                sensor_cfg.body_ids_wp, standing, env.step_dt, period, threshold, out,
            ],
            device=env.device,
        )


@wp.kernel(enable_backward=False)
def _single_support(
    air: wp.array2d(dtype=float), contact: wp.array2d(dtype=float), standing: wp.array(dtype=bool),
    ids: wp.array(dtype=int), threshold: float, target: float, out: wp.array(dtype=float)
):
    i = wp.tid()
    contacts = int(0)
    max_air = float(0.0)
    duration = wp.where(contact[i, ids[0]] > 0.0, contact[i, ids[0]], air[i, ids[0]])
    for k in range(ids.shape[0]):
        j = ids[k]
        contacts += int(contact[i, j] > 0.0)
        max_air = wp.max(max_air, air[i, j])
        duration = wp.min(duration, wp.where(contact[i, j] > 0.0, contact[i, j], air[i, j]))
    reward = 1.0 + wp.min(duration / target, 1.0)
    out[i] = wp.where(contacts == 1 and max_air < threshold and not standing[i], reward, 0.0)


def feet_single_support(env, out, command_name, sensor_cfg, threshold, target_air_time):
    data = env.scene.sensors[sensor_cfg.name].data
    wp.launch(
        _single_support, dim=env.num_envs,
        inputs=[data.current_air_time.warp, data.current_contact_time.warp,
                wp.from_torch(env.command_manager.get_term(command_name).is_standing_env), sensor_cfg.body_ids_wp,
                threshold, target_air_time, out],
        device=env.device,
    )
