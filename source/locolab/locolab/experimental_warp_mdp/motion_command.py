"""Warp motion progression and sampling; retain the proven simulator reset writers."""

import math

import numpy as np
import torch
import warp as wp

from isaaclab.utils.warp.launch_cache import _WarpLaunchCache
from isaaclab.utils.warp.math import yaw_quat
from locolab.tasks.manager_based.locomotion.tracking.mdp.commands.motion_command import MotionCommand

from .tracking import _rotation_error


NoiseVector = wp.types.vector(length=12, dtype=wp.float32)


@wp.kernel(enable_backward=False)
def _advance(
    frames: wp.array(dtype=wp.int64), timer: wp.array(dtype=float), dt: float,
):
    i = wp.tid()
    frames[i] += wp.int64(1)
    timer[i] -= dt


@wp.kernel(enable_backward=False)
def _align(
    frames: wp.array(dtype=wp.int64), origins: wp.array(dtype=wp.vec3),
    reference_pos: wp.array2d(dtype=wp.vec3), reference_ori: wp.array2d(dtype=wp.quat),
    pose: wp.array2d(dtype=wp.transform), anchor: int, reference_anchor: int,
    position: wp.array2d(dtype=wp.vec3), orientation: wp.array2d(dtype=wp.quat),
):
    i, j = wp.tid()
    t = frames[i]
    target = reference_pos[t, reference_anchor] + origins[i]
    actual = wp.transform_get_translation(pose[i, anchor])
    rotation = yaw_quat(
        wp.transform_get_rotation(pose[i, anchor]) * wp.quat_inverse(reference_ori[t, reference_anchor])
    )
    origin = wp.vec3(actual[0], actual[1], target[2])
    position[i, j] = origin + wp.quat_rotate(rotation, reference_pos[t, j] + origins[i] - target)
    orientation[i, j] = rotation * reference_ori[t, j]


@wp.kernel(enable_backward=False)
def _ema(failed: wp.array(dtype=float), current: wp.array(dtype=float), alpha: float):
    i = wp.tid()
    failed[i] = alpha * current[i] + (1.0 - alpha) * failed[i]
    current[i] = 0.0


@wp.kernel(enable_backward=False)
def _metrics(
    frames: wp.array(dtype=wp.int64), origins: wp.array(dtype=wp.vec3),
    reference_pos: wp.array2d(dtype=wp.vec3), reference_ori: wp.array2d(dtype=wp.quat),
    reference_lin: wp.array2d(dtype=wp.vec3), reference_ang: wp.array2d(dtype=wp.vec3),
    reference_joint_pos: wp.array2d(dtype=float), reference_joint_vel: wp.array2d(dtype=float),
    relative_pos: wp.array2d(dtype=wp.vec3), relative_ori: wp.array2d(dtype=wp.quat),
    pose: wp.array2d(dtype=wp.transform), velocity: wp.array2d(dtype=wp.spatial_vector),
    joint_pos: wp.array2d(dtype=float), joint_vel: wp.array2d(dtype=float),
    body_ids: wp.array(dtype=int), joint_ids: wp.array(dtype=int), anchor: int, reference_anchor: int,
    out: wp.array2d(dtype=float),
):
    i = wp.tid()
    t = frames[i]
    actual_pose = pose[i, anchor]
    v = velocity[i, anchor]
    out[i, 0] = wp.length(reference_pos[t, reference_anchor] + origins[i] - wp.transform_get_translation(actual_pose))
    out[i, 1] = _rotation_error(reference_ori[t, reference_anchor], wp.transform_get_rotation(actual_pose))
    out[i, 2] = wp.length(reference_lin[t, reference_anchor] - wp.vec3(v[0], v[1], v[2]))
    out[i, 3] = wp.length(reference_ang[t, reference_anchor] - wp.vec3(v[3], v[4], v[5]))
    position_error = float(0.0)
    rotation_error = float(0.0)
    linear_error = float(0.0)
    angular_error = float(0.0)
    for j in range(body_ids.shape[0]):
        actual = pose[i, body_ids[j]]
        v = velocity[i, body_ids[j]]
        position_error += wp.length(relative_pos[i, j] - wp.transform_get_translation(actual))
        rotation_error += _rotation_error(relative_ori[i, j], wp.transform_get_rotation(actual))
        linear_error += wp.length(reference_lin[t, j] - wp.vec3(v[0], v[1], v[2]))
        angular_error += wp.length(reference_ang[t, j] - wp.vec3(v[3], v[4], v[5]))
    out[i, 4] = position_error / float(body_ids.shape[0])
    out[i, 5] = rotation_error / float(body_ids.shape[0])
    out[i, 6] = linear_error / float(body_ids.shape[0])
    out[i, 7] = angular_error / float(body_ids.shape[0])
    position_squared = float(0.0)
    velocity_squared = float(0.0)
    for j in range(joint_ids.shape[0]):
        dp = reference_joint_pos[t, j] - joint_pos[i, joint_ids[j]]
        dv = reference_joint_vel[t, j] - joint_vel[i, joint_ids[j]]
        position_squared += dp * dp
        velocity_squared += dv * dv
    out[i, 8] = wp.sqrt(position_squared)
    out[i, 9] = wp.sqrt(velocity_squared)


@wp.kernel(enable_backward=False)
def _reset_metrics(ids: wp.array(dtype=wp.int32), metrics: wp.array2d(dtype=float), means: wp.array(dtype=float),
                   counter: wp.array(dtype=wp.int64)):
    k, j = wp.tid()
    i = ids[k]
    wp.atomic_add(means, j, metrics[i, j] / float(ids.shape[0]))
    metrics[i, j] = 0.0
    if j == 0:
        counter[i] = wp.int64(0)


@wp.kernel(enable_backward=False)
def _failures(ids: wp.array(dtype=wp.int32), terminated: wp.array(dtype=bool), frames: wp.array(dtype=wp.int64),
              total: int, counts: wp.array(dtype=float), any_failed: wp.array(dtype=int)):
    k = wp.tid()
    i = ids[k]
    if terminated[i]:
        b = wp.min(int(frames[i] * wp.int64(counts.shape[0]) // wp.int64(total)), counts.shape[0] - 1)
        wp.atomic_add(counts, b, 1.0)
        wp.atomic_add(any_failed, 0, 1)


@wp.kernel(enable_backward=False)
def _distribution(counts: wp.array(dtype=float), any_failed: wp.array(dtype=int), current: wp.array(dtype=float),
                  failed: wp.array(dtype=float), weights: wp.array(dtype=float),
                  uniform: float, cdf: wp.array(dtype=float)):
    total = float(0.0)
    for b in range(failed.shape[0]):
        if any_failed[0] > 0:
            current[b] = counts[b]
        value = float(0.0)
        for k in range(weights.shape[0]):
            value += (failed[wp.min(b + k, failed.shape[0] - 1)] + uniform / float(failed.shape[0])) * weights[k]
        total += value
        cdf[b] = total
    for b in range(cdf.shape[0]):
        cdf[b] /= total


@wp.kernel(enable_backward=False)
def _sample(ids: wp.array(dtype=wp.int32), rng: wp.array(dtype=wp.uint32), cdf: wp.array(dtype=float),
            frames: wp.array(dtype=wp.int64), total: int, adaptive: bool):
    k = wp.tid()
    i = ids[k]
    state = rng[i]
    if adaptive:
        value = wp.randf(state)
        low = int(0)
        high = cdf.shape[0] - 1
        while low < high:
            mid = (low + high) // 2
            if value < cdf[mid]:
                high = mid
            else:
                low = mid + 1
        frames[i] = wp.int64((float(low) + wp.randf(state)) / float(cdf.shape[0]) * float(total - 1))
    else:
        frames[i] = wp.int64(wp.randi(state, 0, total))
    rng[i] = state


class WarpMotionCommand(MotionCommand):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._frames_wp = wp.from_torch(self.time_steps)
        self._timer_wp = wp.from_torch(self.time_left)
        self._origins_wp = wp.from_torch(env.scene.env_origins, dtype=wp.vec3)
        self._positions_wp = wp.from_torch(self.motion.body_pos_w, dtype=wp.vec3)
        self._orientations_wp = wp.from_torch(self.motion.body_quat_w, dtype=wp.quat)
        self._relative_pos_wp = wp.from_torch(self.body_pos_relative_w, dtype=wp.vec3)
        self._relative_ori_wp = wp.from_torch(self.body_quat_relative_w, dtype=wp.quat)
        # Sampling/state writes synchronize this mirror; ordinary progression
        # advances it in place, avoiding a second GPU nonzero/readback every step.
        self._host_frames = np.zeros(self.num_envs, dtype=np.int64)
        self._launch_cache = _WarpLaunchCache(self.device)
        self._align_pose_ptr = None
        self._failed_wp = wp.from_torch(self.bin_failed_count)
        self._current_wp = wp.from_torch(self._current_bin_failed)
        self._weights_wp = wp.from_torch(self.kernel)
        self._counts = wp.zeros(self.bin_count, device=self.device)
        self._any_failed = wp.zeros(1, dtype=int, device=self.device)
        self._cdf = wp.zeros(self.bin_count, device=self.device)
        self._reset_mask_wp = wp.zeros(self.num_envs, dtype=bool, device=self.device)
        self._root_pose_wp = wp.zeros(self.num_envs, dtype=wp.transform, device=self.device)
        self._root_vel_wp = wp.zeros(self.num_envs, dtype=wp.spatial_vector, device=self.device)
        self._reset_position = wp.zeros((self.num_envs, self.robot.num_joints), device=self.device)
        self._reset_velocity = wp.zeros_like(self._reset_position)
        self._joint_ids_wp = wp.array(self._joint_ids, dtype=int, device=self.device)
        self._joint_mask_wp = wp.array(
            [j in self._joint_ids for j in range(self.robot.num_joints)], dtype=bool, device=self.device
        )
        ranges = [cfg.pose_range.get(k, (0., 0.)) for k in ("x", "y", "z", "roll", "pitch", "yaw")]
        ranges += [cfg.velocity_range.get(k, (0., 0.)) for k in ("x", "y", "z", "roll", "pitch", "yaw")]
        self._noise_low = wp.array([v[0] for v in ranges], dtype=float, device=self.device)
        self._noise_high = wp.array([v[1] for v in ranges], dtype=float, device=self.device)
        self._root_reference = [wp.from_torch(value, dtype=dtype) for value, dtype in (
            (self.motion.root_pos, wp.vec3), (self.motion.root_quat, wp.quat),
            (self.motion.root_lin_vel, wp.vec3), (self.motion.root_ang_vel, wp.vec3))]
        self._joint_reference = [wp.from_torch(self.motion.joint_pos), wp.from_torch(self.motion.joint_vel)]
        self._body_ids_wp = wp.array(self._body_ids, dtype=int, device=self.device)
        self._reference_lin_wp = wp.from_torch(self.motion.body_lin_vel_w, dtype=wp.vec3)
        self._reference_ang_wp = wp.from_torch(self.motion.body_ang_vel_w, dtype=wp.vec3)
        self._metrics_wp = wp.zeros((self.num_envs, len(self.metrics)), device=self.device)
        metric_values = wp.to_torch(self._metrics_wp)
        self.metrics = {name: metric_values[:, j] for j, name in enumerate(self.metrics)}
        self._metric_means_wp = wp.zeros(len(self.metrics), device=self.device)
        self._metric_means = wp.to_torch(self._metric_means_wp)
        self._counter_wp = wp.from_torch(self.command_counter)

    def _update_metrics(self):
        self._metrics_update_counter = (self._metrics_update_counter + 1) % self.cfg.metrics_update_interval
        if self._metrics_update_counter != 0:
            return
        data = self.robot.data
        wp.launch(_metrics, dim=self.num_envs, inputs=[
            self._frames_wp, self._origins_wp, self._positions_wp, self._orientations_wp,
            self._reference_lin_wp, self._reference_ang_wp, *self._joint_reference,
            self._relative_pos_wp, self._relative_ori_wp, data.body_link_pose_w.warp, data.body_link_vel_w.warp,
            data.joint_pos.warp, data.joint_vel.warp, self._body_ids_wp, self._joint_ids_wp,
            self.robot_anchor_body_index, self.motion_anchor_body_index, self._metrics_wp,
        ], device=self.device)

    def reset(self, env_ids=None):
        ids = self._as_env_ids(slice(None) if env_ids is None else env_ids)
        self._metric_means_wp.zero_()
        wp.launch(_reset_metrics, dim=(ids.numel(), len(self.metrics)), inputs=[
            wp.from_torch(ids), self._metrics_wp, self._metric_means_wp, self._counter_wp,
        ], device=self.device)
        if ids.numel() == 0:
            self._metric_means_wp.fill_(float("nan"))
        self._resample(ids)
        return {name: self._metric_means[j] for j, name in enumerate(self.metrics)}

    def compute(self, dt):
        # Timers cannot expire within a training episode in this case. Preserve the
        # original timer-resampling order for custom short timers and long Play runs.
        if self.cfg.resampling_time_range[0] <= self._env.max_episode_length_s:
            return super().compute(dt)
        self._update_metrics()
        self._launch_cache.launch(("advance", dt), wp.launch, _advance, dim=self.num_envs,
                                  inputs=[self._frames_wp, self._timer_wp, dt])
        self._finish_update()

    def _update_command(self):
        # Used by CommandTerm.compute after it has already advanced the timer.
        wp.launch(_advance, dim=self.num_envs, inputs=[self._frames_wp, self._timer_wp, 0.0], device=self.device)
        self._finish_update()

    def _finish_update(self):
        self._host_frames += 1
        ids = np.flatnonzero(self._host_frames >= self.motion.time_step_total)
        if ids.size:
            self._resample_command(torch.as_tensor(ids, dtype=torch.int32, device=self.device))
        else:
            self.update_relative_body_poses()
        if self.cfg.sampling_mode == "adaptive":
            self._launch_cache.launch(("ema", self.cfg.adaptive_alpha), wp.launch, _ema, dim=self.bin_count,
                                      inputs=[self._failed_wp, self._current_wp, self.cfg.adaptive_alpha])

    def update_relative_body_poses(self):
        pose = self.robot.data.body_link_pose_w.warp
        if pose.ptr != self._align_pose_ptr:
            self._launch_cache.clear("align")
            self._align_pose_ptr = pose.ptr
        self._launch_cache.launch("align", wp.launch, _align, dim=(self.num_envs, self.num_bodies), inputs=[
            self._frames_wp, self._origins_wp, self._positions_wp, self._orientations_wp,
            pose, self.robot_anchor_body_index, self.motion_anchor_body_index,
            self._relative_pos_wp, self._relative_ori_wp,
        ])

    def _adaptive_sampling(self, env_ids):
        ids = wp.from_torch(env_ids)
        self._counts.zero_()
        self._any_failed.zero_()
        wp.launch(_failures, dim=env_ids.numel(), inputs=[ids,
            wp.from_torch(self._env.termination_manager.terminated), self._frames_wp, self.motion.time_step_total,
            self._counts, self._any_failed], device=self.device)
        wp.launch(_distribution, dim=1, inputs=[self._counts, self._any_failed, self._current_wp, self._failed_wp,
                  self._weights_wp, self.cfg.adaptive_uniform_ratio, self._cdf], device=self.device)
        wp.launch(_sample, dim=env_ids.numel(), inputs=[ids, self._env.rng_state_wp, self._cdf, self._frames_wp,
                  self.motion.time_step_total, True], device=self.device)
        if self._metrics_update_counter == 0:
            # Infrequent diagnostics retain the existing public metrics interface.
            probabilities = wp.to_torch(self._cdf).clone()
            probabilities[1:] -= wp.to_torch(self._cdf)[:-1]
            entropy = -(probabilities * (probabilities + 1e-12).log()).sum()
            self.metrics["sampling_entropy"][:] = entropy / math.log(self.bin_count) if self.bin_count > 1 else 1.0
            top, index = probabilities.max(0)
            self.metrics["sampling_top1_prob"][:] = top
            self.metrics["sampling_top1_bin"][:] = index.float() / self.bin_count

    def _write_reference_state(self, env_ids, perturb):
        # Also covers evaluation's explicit frame seeks before writing state.
        # Keep readbacks on the infrequent sampling/reset path, not every compute.
        self._host_frames[:] = self.time_steps.cpu().numpy()
        self._reset_mask_wp.zero_()
        wp.launch(_reset_state, dim=env_ids.numel(), inputs=[
            wp.from_torch(env_ids), self._frames_wp, self._env.rng_state_wp, self._origins_wp,
            *self._root_reference, *self._joint_reference, self._joint_ids_wp,
            self.robot.data.soft_joint_pos_limits.warp, self._noise_low, self._noise_high,
            *self.cfg.joint_position_range, perturb, self._reset_mask_wp,
            self._root_pose_wp, self._root_vel_wp, self._reset_position, self._reset_velocity,
        ], device=self.device)
        self.robot.write_root_pose_to_sim_mask(
            root_pose=self._root_pose_wp, env_mask=self._reset_mask_wp, skip_forward=True
        )
        self.robot.write_root_velocity_to_sim_mask(
            root_velocity=self._root_vel_wp, env_mask=self._reset_mask_wp, skip_forward=True
        )
        self.robot.write_joint_state_to_sim_mask(
            position=self._reset_position, velocity=self._reset_velocity, joint_mask=self._joint_mask_wp,
            env_mask=self._reset_mask_wp, skip_forward=False,
        )

    def _uniform_sampling(self, env_ids):
        wp.launch(_sample, dim=env_ids.numel(), inputs=[wp.from_torch(env_ids), self._env.rng_state_wp, self._cdf,
                  self._frames_wp, self.motion.time_step_total, False], device=self.device)
        self.metrics["sampling_entropy"][:] = 1.0
        self.metrics["sampling_top1_prob"][:] = 1.0 / self.bin_count
        self.metrics["sampling_top1_bin"][:] = 0.5


@wp.kernel(enable_backward=False)
def _reset_state(
    ids: wp.array(dtype=wp.int32), frames: wp.array(dtype=wp.int64), rng: wp.array(dtype=wp.uint32),
    origins: wp.array(dtype=wp.vec3), reference_pos: wp.array(dtype=wp.vec3), reference_ori: wp.array(dtype=wp.quat),
    reference_lin: wp.array(dtype=wp.vec3), reference_ang: wp.array(dtype=wp.vec3),
    joint_pos: wp.array2d(dtype=float), joint_vel: wp.array2d(dtype=float), joint_ids: wp.array(dtype=int),
    limits: wp.array2d(dtype=wp.vec2), low: wp.array(dtype=float), high: wp.array(dtype=float),
    joint_low: float, joint_high: float, perturb: bool, mask: wp.array(dtype=bool),
    root_pose: wp.array(dtype=wp.transform), root_vel: wp.array(dtype=wp.spatial_vector),
    position: wp.array2d(dtype=float), velocity: wp.array2d(dtype=float),
):
    k = wp.tid()
    i = ids[k]
    t = frames[i]
    state = rng[i]
    noise = NoiseVector()
    if perturb:
        for axis in range(12):
            noise[axis] = wp.randf(state, low[axis], high[axis])
    pos = reference_pos[t] + origins[i] + wp.vec3(noise[0], noise[1], noise[2])
    qx = wp.quat_from_axis_angle(wp.vec3(1., 0., 0.), noise[3])
    qy = wp.quat_from_axis_angle(wp.vec3(0., 1., 0.), noise[4])
    qz = wp.quat_from_axis_angle(wp.vec3(0., 0., 1.), noise[5])
    root_pose[i] = wp.transform(pos, qz * qy * qx * reference_ori[t])
    lin = reference_lin[t] + wp.vec3(noise[6], noise[7], noise[8])
    ang = reference_ang[t] + wp.vec3(noise[9], noise[10], noise[11])
    root_vel[i] = wp.spatial_vector(lin[0], lin[1], lin[2], ang[0], ang[1], ang[2])
    for j in range(joint_ids.shape[0]):
        target = joint_pos[t, j]
        if perturb:
            target += wp.randf(state, joint_low, joint_high)
        asset_joint = joint_ids[j]
        bound = limits[i, asset_joint]
        position[i, asset_joint] = wp.clamp(target, bound[0], bound[1])
        velocity[i, asset_joint] = joint_vel[t, j]
    mask[i] = True
    rng[i] = state
