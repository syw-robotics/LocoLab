"""MID360 adaptation of livox-mid360-lidar-unitree-g1-mjlab (see LICENSE)."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import torch
import warp as wp

from isaaclab.sensors import Imu, MultiMeshRayCaster
from isaaclab.sensors.sensor_base import SensorBase

from . import kernels


class Mid360:
    """Compose the MID360 model with the active Isaac Lab multi-mesh backend."""

    def __new__(cls, cfg):
        return type(cls.__name__, (_Mid360Mixin, MultiMeshRayCaster.resolve_class()), {})(cfg)


class _Mid360Mixin:
    def _initialize_impl(self):
        if self.cfg.point_rate <= 0 or self.cfg.frame_rate <= 0:
            raise ValueError("MID360 point_rate and frame_rate must be positive.")
        dt = self.cfg.update_period
        self._batch = round(self.cfg.point_rate * dt)
        self._frame_size = round(self.cfg.point_rate / self.cfg.frame_rate)
        if (
            dt <= 0
            or abs(dt - self._sim.get_physics_dt()) > 1e-8
            or self._batch <= 0
            or self._batch % 4
            or self._frame_size < self._batch
            or self._frame_size % self._batch
            or abs(self._batch / self.cfg.point_rate - dt) > 1e-8
            or abs(self._frame_size / self.cfg.point_rate - 1 / self.cfg.frame_rate) > 1e-8
        ):
            raise ValueError(
                "MID360 requires update_period=physics dt, integral rays/step divisible by 4, "
                "and an integral number of steps/frame."
            )
        if self.cfg.ray_alignment != "base":
            raise ValueError("MID360 rays must follow the full sensor orientation (ray_alignment='base').")
        self.cfg.pattern_cfg.samples = self._batch
        super()._initialize_impl()
        angles = np.load(Path(__file__).with_name("mid360.npy"))
        theta, phi = angles.T
        directions = np.stack((np.cos(phi) * np.cos(theta), np.cos(phi) * np.sin(theta), np.sin(phi)), axis=-1)
        self._table = wp.array(directions, dtype=wp.vec3, device=self.device)
        rho = []
        for i, target in enumerate(self._raycast_targets_cfg):
            value = self.cfg.target_reflectivity.get(i, 0.5)
            if not 0.0 <= value <= 1.0:
                raise ValueError("Target reflectivity must be between 0 and 1.")
            rho.extend([value] * self._num_meshes_per_env[target.prim_expr])
        self._reflectance = wp.array(rho, dtype=float, device=self.device)
        b, p = self.num_instances, self._frame_size
        self._slots = wp.zeros(b, dtype=wp.int32, device=self.device)
        self._cursor = wp.zeros(b, dtype=wp.int32, device=self.device)
        self._start_pos = wp.zeros(b, dtype=wp.vec3, device=self.device)
        self._start_quat = wp.zeros(b, dtype=wp.quat, device=self.device)
        self._pending = self._allocate_frame(b, p)
        self._published = self._allocate_frame(b, p)
        self._frame_pos = wp.zeros(b, dtype=wp.vec3, device=self.device)
        self._frame_quat = wp.zeros(b, dtype=wp.quat, device=self.device)
        self._frame_id = wp.zeros(b, dtype=wp.int32, device=self.device)
        self._timebase = wp.zeros(b, dtype=float, device=self.device)
        index = torch.arange(p, device=self.device)
        # Published tensors are persistent GPU views, updated only when a complete frame is ready.
        self._data.output = dict(
            zip(("points", "points_w", "valid", "reflectivity"), map(wp.to_torch, self._published))
        )
        self._data.output["tag"] = torch.zeros((b, p), dtype=torch.uint8, device=self.device)
        self._data.frame = SimpleNamespace(
            **self._data.output,
            offset_time=(index.double() * 1e9 / self.cfg.point_rate).round().long(),
            line=(index % 4).to(torch.uint8),
            timebase=wp.to_torch(self._timebase),
            frame_id=wp.to_torch(self._frame_id),
            pos_w=wp.to_torch(self._frame_pos),
            quat_w=wp.to_torch(self._frame_quat),
        )

    def get_debug_vis_pose(self):
        """Current sensor pose, including the configured mount offset (xyzw)."""
        return self.data.pos_w, self.data.quat_w

    def _allocate_frame(self, b, p):
        return [wp.zeros((b, p), dtype=dtype, device=self.device) for dtype in (wp.vec3, wp.vec3, wp.bool, wp.uint8)]

    def update(self, dt, force_recompute=False):
        # Temporal scans must be sampled every physics step, even with scene.lazy_sensor_update=True.
        super().update(dt, force_recompute=True)

    def _update_buffers_impl(self, env_mask):
        self._update_ray_infos(env_mask)
        # RayCaster data tracks the parent frame; cfg.offset is applied only to its rays.
        wp.launch(
            kernels.apply_mount,
            dim=self.num_instances,
            inputs=[
                env_mask,
                wp.vec3(*self.cfg.offset.pos),
                wp.quat(*self.cfg.offset.rot),
                self._data._pos_w,
                self._data._quat_w,
            ],
            device=self.device,
        )
        self._update_mesh_transforms()
        cfg = self.cfg
        wp.launch(
            kernels.scan,
            dim=(self.num_instances, self._batch),
            inputs=[
                env_mask,
                self._table,
                self._cursor,
                self._slots,
                self._data._pos_w,
                self._data._quat_w,
                self._mesh_ids_wp,
                self._mesh_positions_w,
                self._mesh_orientations_w,
                self._reflectance,
                cfg.noise,
                cfg.seed,
                cfg.blind,
                cfg.range_10,
                cfg.max_distance,
                cfg.angle_sigma,
                cfg.false_alarm_rate,
                cfg.resolution,
                *self._pending,
                self._data._ray_hits_w,
            ],
            device=self.device,
        )
        wp.launch(
            kernels.publish,
            dim=(self.num_instances, self._frame_size),
            inputs=[
                env_mask,
                self._slots,
                self._batch,
                *self._pending,
                *self._published,
            ],
            device=self.device,
        )
        wp.launch(
            kernels.advance,
            dim=self.num_instances,
            inputs=[
                env_mask,
                self._slots,
                self._cursor,
                self._batch,
                self._frame_size,
                len(self._table),
                self._timestamp,
                cfg.update_period,
                self._data._pos_w,
                self._data._quat_w,
                self._start_pos,
                self._start_quat,
                self._frame_pos,
                self._frame_quat,
                self._timebase,
                self._frame_id,
            ],
            device=self.device,
        )

    def reset(self, env_ids=None, env_mask=None):
        mask = self._resolve_indices_and_mask(env_ids, env_mask)
        super().reset(env_mask=mask)
        # RayCaster invalidates on reset; a temporal scan must wait for positive simulation time.
        SensorBase.reset(self, env_mask=mask)
        wp.launch(
            kernels.reset_frames,
            dim=(self.num_instances, self._frame_size),
            inputs=[
                mask,
                self._slots,
                *self._published,
                self._frame_id,
                self._timebase,
                self._frame_pos,
                self._frame_quat,
            ],
            device=self.device,
        )
        # Scan phase continues across resets, as on hardware; old pending points are fully overwritten.


class Mid360Imu:
    """Isaac Lab IMU with the original project's white noise and Livox g output."""

    def __new__(cls, cfg):
        return type(cls.__name__, (_Mid360ImuMixin, Imu.resolve_class()), {})(cfg)


class _Mid360ImuMixin:
    def _initialize_impl(self):
        super()._initialize_impl()
        if abs(self.cfg.update_period - self._sim.get_physics_dt()) > 1e-8:
            raise ValueError("MID360 IMU requires update_period=physics dt.")
        self._accel_g = wp.zeros(self.num_instances, dtype=wp.vec3, device=self.device)
        self._sample_index = wp.zeros(self.num_instances, dtype=wp.int32, device=self.device)
        self._data.output = {"gyro": self._data.ang_vel_b.torch, "accel_g": wp.to_torch(self._accel_g)}

    def update(self, dt, force_recompute=False):
        super().update(dt, force_recompute=True)

    def _update_buffers_impl(self, env_mask):
        super()._update_buffers_impl(env_mask)
        fs = 1.0 / self.cfg.update_period
        wp.launch(
            kernels.imu_noise,
            dim=self.num_instances,
            inputs=[
                env_mask,
                self._sample_index,
                self.cfg.seed,
                self.cfg.gyro_noise_density * fs**0.5 if self.cfg.noise else 0.0,
                self.cfg.accel_noise_density * fs**0.5 if self.cfg.noise else 0.0,
                self._data.ang_vel_b.warp,
                self._data.lin_acc_b.warp,
                self._accel_g,
            ],
            device=self.device,
        )

    def reset(self, env_ids=None, env_mask=None):
        mask = self._resolve_indices_and_mask(env_ids, env_mask)
        super().reset(env_mask=mask)
        self._data.output["accel_g"][wp.to_torch(mask)] = 0.0
