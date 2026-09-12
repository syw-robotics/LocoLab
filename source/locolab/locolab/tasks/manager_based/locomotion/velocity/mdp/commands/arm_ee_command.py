# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Arm end-effector pose command generator with trajectory interpolation.

Pose commands are sampled from a base-frame dataset. In the default yaw-aligned
anchor mode, trajectories are interpolated in the sampled Cartesian command frame
while the world-frame target is reconstructed every step as:

    p_w = center_w + R_yaw @ p_cmd

where ``center_w`` follows the robot horizontally and uses a terrain-fixed height.
Policy observations receive the world target in the current base frame, while
rewards compare against the reconstructed world-frame command.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import torch

from isaaclab.assets import Articulation
from isaaclab.managers import CommandTerm
from isaaclab.markers import VisualizationMarkers
from isaaclab.utils.math import (
    combine_frame_transforms,
    compute_pose_error,
    quat_apply,
    quat_apply_inverse,
    quat_unique,
    subtract_frame_transforms,
    yaw_quat,
)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from .commands_cfg import SampledArmEEPoseCommandCfg

# Pose layout: [x, y, z, qw, qx, qy, qz].
_POS_SLICE = slice(0, 3)
_QUAT_SLICE = slice(3, 7)


# ---------------------------------------------------------------------------
# Spherical interpolation helpers (UniFP-style position trajectories)
# ---------------------------------------------------------------------------


def _cart2sphere(cart_coords: torch.Tensor) -> torch.Tensor:
    """Convert Cartesian coordinates to spherical coordinates ``(radius, pitch, yaw)``."""
    sphere_coords = torch.zeros_like(cart_coords)
    xy_len = torch.norm(cart_coords[..., :2], dim=-1)
    sphere_coords[..., 0] = torch.norm(cart_coords, dim=-1)
    sphere_coords[..., 1] = torch.atan2(cart_coords[..., 2], xy_len + 1e-8)
    sphere_coords[..., 2] = torch.atan2(cart_coords[..., 1], cart_coords[..., 0])
    return sphere_coords


def _sphere2cart(sphere_coords: torch.Tensor) -> torch.Tensor:
    """Convert spherical coordinates ``(radius, pitch, yaw)`` to Cartesian coordinates."""
    radius = sphere_coords[..., 0]
    pitch = sphere_coords[..., 1]
    yaw = sphere_coords[..., 2]
    cos_pitch = torch.cos(pitch)
    cart_coords = torch.zeros_like(sphere_coords)
    cart_coords[..., 0] = radius * cos_pitch * torch.cos(yaw)
    cart_coords[..., 1] = radius * cos_pitch * torch.sin(yaw)
    cart_coords[..., 2] = radius * torch.sin(pitch)
    return cart_coords


def _quat_slerp_batch(q0: torch.Tensor, q1: torch.Tensor, blend: torch.Tensor) -> torch.Tensor:
    """Spherical linear interpolation between batched quaternions in ``(w, x, y, z)`` format."""
    if blend.ndim == 1:
        blend = blend.unsqueeze(-1)

    q1_adj = torch.where((torch.sum(q0 * q1, dim=-1, keepdim=True) < 0.0), -q1, q1)
    dot = torch.sum(q0 * q1_adj, dim=-1, keepdim=True).abs().clamp(-1.0, 1.0)

    q_nlerp = quat_unique((1.0 - blend) * q0 + blend * q1_adj)
    linear_mask = (dot > 0.9995).expand_as(q0)

    theta = torch.acos(dot)
    sin_theta = torch.sin(theta)
    w0 = torch.sin((1.0 - blend) * theta) / sin_theta
    w1 = torch.sin(blend * theta) / sin_theta
    q_slerp = quat_unique(w0 * q0 + w1 * q1_adj)

    return torch.where(linear_mask, q_nlerp, q_slerp)


def _broadcast_env_vec(vec: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
    """Broadcast a ``(num_envs, 3)`` tensor to match ``(..., 3)`` target layout."""
    out = vec
    while out.ndim < target.ndim:
        out = out.unsqueeze(-2)
    return out


class SampledArmEePoseCommand(CommandTerm):
    """Sampled end-effector pose commands with yaw-aligned or world-frame anchoring.

    Dataset rows are stored in the robot base frame as
    ``[arm_q..., x, y, z, qw, qx, qy, qz]`` or legacy ``[qw, qx, qy, qz, x, y, z]``.

    In ``yaw_aligned`` mode (default), sampled Cartesian poses are interpolated in
    the command frame while the world target is reconstructed every step via a
    yaw-aligned anchor center. In ``world`` mode, the sampled pose is anchored once
    at resample time using the full root pose and interpolated in world coordinates.
    """

    cfg: SampledArmEEPoseCommandCfg

    def __init__(self, cfg: SampledArmEEPoseCommandCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)

        if cfg.anchor_mode not in {"yaw_aligned", "world"}:
            raise ValueError(f"Unsupported anchor_mode: {cfg.anchor_mode}")
        if cfg.track_orientation and cfg.anchor_mode == "yaw_aligned":
            raise ValueError("track_orientation=True requires anchor_mode='world'.")

        self.robot: Articulation = env.scene[cfg.asset_name]
        self.body_idx = self.robot.find_bodies(cfg.body_name)[0][0]
        self._yaw_aligned_anchor = cfg.anchor_mode == "yaw_aligned"

        self.pose_pool = self._load_pose_pool(cfg)
        self.num_samples = self.pose_pool.shape[0]
        self._validate_interp_modes(cfg)
        if not isinstance(cfg.metrics_update_interval, int) or cfg.metrics_update_interval < 1:
            raise ValueError(
                "Expected metrics_update_interval to be a positive integer, "
                f"got {cfg.metrics_update_interval}."
            )

        modes = set(cfg.interpolation_modes)
        self._only_sphere_mode = modes == {"sphere"}
        self._only_cartesian_mode = modes == {"cartesian"}
        self._mode_is_sphere = torch.tensor(
            [mode == "sphere" for mode in cfg.interpolation_modes],
            device=self.device,
            dtype=torch.bool,
        )

        self.pose_command_w = self._identity_pose_buffer()
        self.pose_start_b = self._identity_pose_buffer()
        self.pose_target_b = self._identity_pose_buffer()
        self.pose_interp_b = self._identity_pose_buffer()
        self.pose_command_b = self._identity_pose_buffer()
        self.pose_start_w = self._identity_pose_buffer()
        self.pose_target_w = self._identity_pose_buffer()
        self.anchor_center_w = torch.zeros(self.num_envs, 3, device=self.device)

        self.interp_progress = torch.ones(self.num_envs, device=self.device)
        self.interp_time_s = torch.full(
            (self.num_envs,),
            cfg.interpolation_time_range[0],
            device=self.device,
            dtype=torch.float32,
        )
        self.use_sphere_interp = torch.full(
            (self.num_envs,),
            cfg.interpolation_modes[0] == "sphere",
            device=self.device,
            dtype=torch.bool,
        )
        self.sphere_center_offset_b = torch.tensor(
            cfg.sphere_center_offset_b, device=self.device, dtype=torch.float32
        ).unsqueeze(0)
        self.anchor_center_offset_b = torch.tensor(
            cfg.anchor_center_offset_b, device=self.device, dtype=torch.float32
        ).unsqueeze(0)

        self.metrics["position_error"] = torch.zeros(self.num_envs, device=self.device)
        if cfg.track_orientation:
            self.metrics["orientation_error"] = torch.zeros(self.num_envs, device=self.device)
        self._metrics_update_counter = 0
        self._metrics_update_scale = (
            cfg.metrics_update_interval * self._env.step_dt / cfg.resampling_time_range[1]
        )

    def __str__(self) -> str:
        msg = "SampledArmEePoseCommand:\n"
        msg += f"\tCommand dimension: {tuple(self.command.shape[1:])}\n"
        msg += f"\tAnchor mode: {self.cfg.anchor_mode}\n"
        msg += f"\tResampling time range: {self.cfg.resampling_time_range}\n"
        msg += f"\tTrack orientation: {self.cfg.track_orientation}\n"
        msg += f"\tMetrics update interval: {self.cfg.metrics_update_interval} control steps\n"
        msg += f"\tInterpolation time range: {self.cfg.interpolation_time_range}\n"
        msg += f"\tInterpolation modes: {self.cfg.interpolation_modes}\n"
        msg += f"\tAnchor z (world): {self.cfg.anchor_z_world}\n"
        msg += f"\tAnchor center offset (yaw frame): {self.cfg.anchor_center_offset_b}\n"
        msg += f"\tSphere center offset (interp): {self.cfg.sphere_center_offset_b}\n"
        msg += f"\tPose dataset: {self.cfg.pose_dataset_path}\n"
        msg += f"\tNumber of pose samples: {self.num_samples}\n"
        msg += f"\tBody name: {self.cfg.body_name}\n"
        return msg

    @property
    def command(self) -> torch.Tensor:
        """The desired end-effector command in the current robot base frame."""
        if self.cfg.track_orientation:
            return self.pose_command_b
        return self.pose_command_b[:, _POS_SLICE]

    @property
    def command_w(self) -> torch.Tensor:
        """The desired end-effector command in the environment world frame."""
        if self.cfg.track_orientation:
            return self.pose_command_w
        return self.pose_command_w[:, _POS_SLICE]

    # ------------------------------------------------------------------
    # CommandTerm interface
    # ------------------------------------------------------------------

    def _update_metrics(self):
        self._metrics_update_counter = (self._metrics_update_counter + 1) % self.cfg.metrics_update_interval
        if self._metrics_update_counter != 0:
            return

        if self.cfg.track_orientation:
            pos_error, rot_error = compute_pose_error(
                self.pose_command_w[:, _POS_SLICE],
                self.pose_command_w[:, _QUAT_SLICE],
                self.robot.data.body_pos_w[:, self.body_idx],
                self.robot.data.body_quat_w[:, self.body_idx],
            )
            self.metrics["position_error"].add_(torch.norm(pos_error, dim=-1), alpha=self._metrics_update_scale)
            self.metrics["orientation_error"].add_(torch.norm(rot_error, dim=-1), alpha=self._metrics_update_scale)
            return

        pos_error = self.robot.data.body_pos_w[:, self.body_idx] - self.pose_command_w[:, _POS_SLICE]
        self.metrics["position_error"].add_(torch.norm(pos_error, dim=-1), alpha=self._metrics_update_scale)

    def _resample_command(self, env_ids: Sequence[int]):
        env_ids_tensor = self._resolve_env_ids(env_ids)
        if len(env_ids_tensor) == 0:
            return

        sample_ids = torch.randint(0, self.num_samples, (len(env_ids_tensor),), device=self.device)
        sampled_pose_b = self.pose_pool[sample_ids]

        if self._yaw_aligned_anchor:
            self.pose_target_b[env_ids_tensor, _POS_SLICE] = sampled_pose_b[:, _POS_SLICE]
        else:
            root_pos = self.robot.data.root_pos_w[env_ids_tensor]
            root_quat = self.robot.data.root_quat_w[env_ids_tensor]
            if self.cfg.track_orientation:
                target_pos_w, target_quat_w = combine_frame_transforms(
                    root_pos,
                    root_quat,
                    sampled_pose_b[:, _POS_SLICE],
                    sampled_pose_b[:, _QUAT_SLICE],
                )
                self.pose_target_w[env_ids_tensor, _POS_SLICE] = target_pos_w
                self.pose_target_w[env_ids_tensor, _QUAT_SLICE] = target_quat_w
            else:
                self.pose_target_w[env_ids_tensor, _POS_SLICE] = root_pos + quat_apply(
                    root_quat, sampled_pose_b[:, _POS_SLICE]
                )

        self.interp_progress[env_ids_tensor] = 0.0
        self._sample_interp_settings(env_ids_tensor)
        self._set_interp_start_pose(env_ids_tensor)

    def _update_command(self):
        if self._yaw_aligned_anchor:
            self._update_command_yaw_aligned()
        else:
            self._update_command_world()

    def _update_command_yaw_aligned(self):
        instant_mask = self.interp_time_s <= 0.0
        if torch.any(instant_mask):
            self.pose_interp_b[instant_mask, _POS_SLICE] = self.pose_target_b[instant_mask, _POS_SLICE]
            self.interp_progress[instant_mask] = 1.0

        active_mask = (self.interp_progress < 1.0) & (~instant_mask)
        if torch.any(active_mask):
            progress_step = self._env.step_dt / self.interp_time_s.clamp(min=1e-6)
            self.interp_progress[active_mask] = torch.clamp(
                self.interp_progress[active_mask] + progress_step[active_mask],
                max=1.0,
            )

            blend = self.interp_progress[active_mask].unsqueeze(-1)
            interp_center = self.sphere_center_offset_b.expand(self.num_envs, -1)[active_mask]
            self.pose_interp_b[active_mask, _POS_SLICE] = self._lerp_positions(
                self.pose_start_b[active_mask, _POS_SLICE],
                self.pose_target_b[active_mask, _POS_SLICE],
                blend,
                use_sphere=self.use_sphere_interp[active_mask],
                center=interp_center,
            )

        self._update_anchor_center_w()
        self.pose_command_w[:, _POS_SLICE] = self._yaw_cmd_b_to_world_pos(self.pose_interp_b[:, _POS_SLICE])
        self._sync_pose_obs_b()

    def _update_command_world(self):
        instant_mask = self.interp_time_s <= 0.0
        if torch.any(instant_mask):
            if self.cfg.track_orientation:
                self.pose_command_w[instant_mask] = self.pose_target_w[instant_mask]
            else:
                self.pose_command_w[instant_mask, _POS_SLICE] = self.pose_target_w[instant_mask, _POS_SLICE]
            self.interp_progress[instant_mask] = 1.0

        active_mask = (self.interp_progress < 1.0) & (~instant_mask)
        if torch.any(active_mask):
            progress_step = self._env.step_dt / self.interp_time_s.clamp(min=1e-6)
            self.interp_progress[active_mask] = torch.clamp(
                self.interp_progress[active_mask] + progress_step[active_mask],
                max=1.0,
            )

            blend = self.interp_progress[active_mask].unsqueeze(-1)
            active_center = self.anchor_center_w[active_mask]
            self.pose_command_w[active_mask, _POS_SLICE] = self._lerp_positions(
                self.pose_start_w[active_mask, _POS_SLICE],
                self.pose_target_w[active_mask, _POS_SLICE],
                blend,
                use_sphere=self.use_sphere_interp[active_mask],
                center=active_center,
            )
            if self.cfg.track_orientation:
                self.pose_command_w[active_mask, _QUAT_SLICE] = _quat_slerp_batch(
                    self.pose_start_w[active_mask, _QUAT_SLICE],
                    self.pose_target_w[active_mask, _QUAT_SLICE],
                    self.interp_progress[active_mask],
                )

        self._sync_pose_obs_b()

    # ------------------------------------------------------------------
    # Interpolation
    # ------------------------------------------------------------------

    def _sample_interp_settings(self, env_ids: torch.Tensor):
        """Sample per-trajectory interpolation settings once per resample."""
        time_min, time_max = self.cfg.interpolation_time_range
        if time_max > time_min:
            self.interp_time_s[env_ids] = torch.rand(len(env_ids), device=self.device) * (time_max - time_min) + time_min
        else:
            self.interp_time_s[env_ids] = time_min

        if len(self.cfg.interpolation_modes) == 1:
            self.use_sphere_interp[env_ids] = self._mode_is_sphere[0]
        else:
            mode_ids = torch.randint(0, len(self._mode_is_sphere), (len(env_ids),), device=self.device)
            self.use_sphere_interp[env_ids] = self._mode_is_sphere[mode_ids]

    def _lerp_positions_sphere(
        self,
        start_pos: torch.Tensor,
        target_pos: torch.Tensor,
        blend: torch.Tensor,
        center: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """Interpolate positions in spherical coordinates about the given sphere center."""
        if center is None:
            center = self.sphere_center_offset_b.expand(start_pos.shape[0], -1)
        center = _broadcast_env_vec(center, start_pos)
        start_rel = start_pos - center
        target_rel = target_pos - center

        start_s = _cart2sphere(start_rel.reshape(-1, 3)).reshape(start_rel.shape)
        target_s = _cart2sphere(target_rel.reshape(-1, 3)).reshape(target_rel.shape)
        curr_rel = _sphere2cart(torch.lerp(start_s, target_s, blend).reshape(-1, 3)).reshape(start_rel.shape)
        return curr_rel + center

    def _lerp_positions(
        self,
        start_pos: torch.Tensor,
        target_pos: torch.Tensor,
        blend: torch.Tensor,
        use_sphere: torch.Tensor | None = None,
        center: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """Interpolate EE positions using per-trajectory mode flags."""
        if self._only_sphere_mode:
            return self._lerp_positions_sphere(start_pos, target_pos, blend, center=center)
        if self._only_cartesian_mode:
            return torch.lerp(start_pos, target_pos, blend)

        if use_sphere is None:
            use_sphere = self.use_sphere_interp

        cart_pos = torch.lerp(start_pos, target_pos, blend)
        if torch.all(use_sphere):
            return self._lerp_positions_sphere(start_pos, target_pos, blend, center=center)
        if not torch.any(use_sphere):
            return cart_pos

        sphere_pos = self._lerp_positions_sphere(start_pos, target_pos, blend, center=center)
        mask = use_sphere
        while mask.ndim < start_pos.ndim:
            mask = mask.unsqueeze(-1)
        return torch.where(mask, sphere_pos, cart_pos)

    def _set_interp_start_pose(self, env_ids: torch.Tensor):
        """Set interpolation start to the current EE pose on the first command, else last command."""
        if self._yaw_aligned_anchor:
            self._set_interp_start_pose_yaw_aligned(env_ids)
            return

        if self.robot.is_initialized:
            ee_pose_w = self._compute_ee_pose_w()
            is_first_command = self.command_counter[env_ids] == 0
            if torch.any(is_first_command):
                first_env_ids = env_ids[is_first_command]
                if self.cfg.track_orientation:
                    self.pose_start_w[first_env_ids] = ee_pose_w[first_env_ids]
                    self.pose_command_w[first_env_ids] = ee_pose_w[first_env_ids]
                else:
                    self.pose_start_w[first_env_ids, _POS_SLICE] = ee_pose_w[first_env_ids, _POS_SLICE]
                    self.pose_command_w[first_env_ids, _POS_SLICE] = ee_pose_w[first_env_ids, _POS_SLICE]
            if torch.any(~is_first_command):
                later_env_ids = env_ids[~is_first_command]
                if self.cfg.track_orientation:
                    self.pose_start_w[later_env_ids] = self.pose_command_w[later_env_ids]
                else:
                    self.pose_start_w[later_env_ids, _POS_SLICE] = self.pose_command_w[later_env_ids, _POS_SLICE]
        elif self.cfg.track_orientation:
            self.pose_start_w[env_ids] = self.pose_command_w[env_ids]
        else:
            self.pose_start_w[env_ids, _POS_SLICE] = self.pose_command_w[env_ids, _POS_SLICE]

        if not self._yaw_aligned_anchor:
            self.anchor_center_w[env_ids] = self.robot.data.root_pos_w[env_ids] + quat_apply(
                self.robot.data.root_quat_w[env_ids],
                self.sphere_center_offset_b.expand(len(env_ids), -1),
            )

    def _set_interp_start_pose_yaw_aligned(self, env_ids: torch.Tensor):
        if self.robot.is_initialized:
            self._update_anchor_center_w()
            ee_pos_w = self.robot.data.body_pos_w[:, self.body_idx]
            is_first_command = self.command_counter[env_ids] == 0
            if torch.any(is_first_command):
                first_env_ids = env_ids[is_first_command]
                start_b = self._world_pos_to_yaw_cmd_b(ee_pos_w[first_env_ids])
                self.pose_start_b[first_env_ids, _POS_SLICE] = start_b
                self.pose_interp_b[first_env_ids, _POS_SLICE] = start_b
            if torch.any(~is_first_command):
                later_env_ids = env_ids[~is_first_command]
                start_b = self._world_pos_to_yaw_cmd_b(self.pose_command_w[later_env_ids, _POS_SLICE])
                self.pose_start_b[later_env_ids, _POS_SLICE] = start_b
                self.pose_interp_b[later_env_ids, _POS_SLICE] = start_b
        else:
            self.pose_start_b[env_ids, _POS_SLICE] = self.pose_interp_b[env_ids, _POS_SLICE]

    # ------------------------------------------------------------------
    # Debug visualization (not on the RL hot path)
    # ------------------------------------------------------------------

    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis:
            if not hasattr(self, "goal_visualizer"):
                if self.cfg.track_orientation:
                    self.goal_visualizer = VisualizationMarkers(self.cfg.goal_ee_visualizer_cfg)
                    self.current_visualizer = VisualizationMarkers(self.cfg.current_ee_visualizer_cfg)
                else:
                    self.goal_visualizer = VisualizationMarkers(self.cfg.goal_pos_visualizer_cfg)
                    self.current_visualizer = VisualizationMarkers(self.cfg.current_pos_visualizer_cfg)
                self.interp_path_visualizer = VisualizationMarkers(self.cfg.interp_path_visualizer_cfg)
            self.goal_visualizer.set_visibility(True)
            self.current_visualizer.set_visibility(True)
            self.interp_path_visualizer.set_visibility(True)
        else:
            if hasattr(self, "goal_visualizer"):
                self.goal_visualizer.set_visibility(False)
                self.current_visualizer.set_visibility(False)
                self.interp_path_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        if not self.robot.is_initialized:
            return

        if self._yaw_aligned_anchor:
            goal_pos_w = self._yaw_cmd_b_to_world_pos(self.pose_target_b[:, _POS_SLICE])
        else:
            goal_pos_w = self.pose_target_w[:, _POS_SLICE]

        if self.cfg.track_orientation:
            body_link_pose_w = self.robot.data.body_link_pose_w[:, self.body_idx]
            self.goal_visualizer.visualize(goal_pos_w, self.pose_target_w[:, _QUAT_SLICE])
            self.current_visualizer.visualize(body_link_pose_w[:, _POS_SLICE], body_link_pose_w[:, _QUAT_SLICE])
        else:
            self.goal_visualizer.visualize(goal_pos_w)
            self.current_visualizer.visualize(self.robot.data.body_pos_w[:, self.body_idx])

        self.interp_path_visualizer.visualize(self._compute_interp_path_positions_w())

    def _compute_interp_path_positions_w(self) -> torch.Tensor:
        """Sample the planned interpolation path in the world frame for debug markers."""
        num_points = max(self.cfg.interp_path_num_points, 2)
        blend = torch.linspace(0.0, 1.0, num_points, device=self.device).view(1, num_points, 1)

        if self._yaw_aligned_anchor:
            start_pos = self.pose_start_b[:, _POS_SLICE].unsqueeze(1).expand(-1, num_points, -1)
            target_pos = self.pose_target_b[:, _POS_SLICE].unsqueeze(1).expand(-1, num_points, -1)
            interp_center = self.sphere_center_offset_b.unsqueeze(1).expand(-1, num_points, -1)
            path_pos_b = self._lerp_positions(start_pos, target_pos, blend, center=interp_center)
            return self._yaw_cmd_b_to_world_pos(path_pos_b)

        start_pos = self.pose_start_w[:, _POS_SLICE].unsqueeze(1).expand(-1, num_points, -1)
        target_pos = self.pose_target_w[:, _POS_SLICE].unsqueeze(1).expand(-1, num_points, -1)
        active_center = self.anchor_center_w.unsqueeze(1).expand(-1, num_points, -1)
        return self._lerp_positions(start_pos, target_pos, blend, center=active_center).reshape(-1, 3)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _load_pose_pool(self, cfg: SampledArmEEPoseCommandCfg) -> torch.Tensor:
        dataset_path = Path(cfg.pose_dataset_path).expanduser().resolve()
        if not dataset_path.is_file():
            raise FileNotFoundError(f"Arm EE pose dataset not found: {dataset_path}")

        dataset = np.load(dataset_path)
        if "arm_joint_q_and_ee_pose" in dataset:
            arm_joint_q_and_ee_pose = torch.as_tensor(dataset["arm_joint_q_and_ee_pose"], device=self.device, dtype=torch.float32)
            if arm_joint_q_and_ee_pose.ndim != 2 or arm_joint_q_and_ee_pose.shape[1] < 7:
                raise ValueError(f"Expected arm_joint_q_and_ee_pose shape (N, n_arm + 7), got {tuple(arm_joint_q_and_ee_pose.shape)}")
            pose_pool = arm_joint_q_and_ee_pose[:, -7:]
        elif "ee_pose" in dataset:
            ee_pose = torch.as_tensor(dataset["ee_pose"], device=self.device, dtype=torch.float32)
            if ee_pose.ndim != 2 or ee_pose.shape[1] != 7:
                raise ValueError(f"Expected ee_pose shape (N, 7), got {tuple(ee_pose.shape)}")
            pose_pool = torch.cat([ee_pose[:, 4:7], ee_pose[:, :4]], dim=1)
        else:
            raise KeyError(
                f"Arm EE pose dataset must contain 'arm_joint_q_and_ee_pose' or 'ee_pose', got keys: {list(dataset.keys())}"
            )

        if cfg.make_quat_unique:
            pose_pool[:, _QUAT_SLICE] = quat_unique(pose_pool[:, _QUAT_SLICE])
        return pose_pool

    @staticmethod
    def _validate_interp_modes(cfg: SampledArmEEPoseCommandCfg):
        if len(cfg.interpolation_modes) == 0:
            raise ValueError("interpolation_modes must contain at least one mode.")
        invalid_modes = set(cfg.interpolation_modes) - {"sphere", "cartesian"}
        if invalid_modes:
            raise ValueError(f"Unsupported interpolation modes: {sorted(invalid_modes)}")

    def _identity_pose_buffer(self) -> torch.Tensor:
        buffer = torch.zeros(self.num_envs, 7, device=self.device)
        buffer[:, 3] = 1.0
        return buffer

    def _compute_ee_pose_w(self) -> torch.Tensor:
        """Return the current end-effector pose expressed in the environment world frame."""
        return torch.cat(
            [
                self.robot.data.body_pos_w[:, self.body_idx],
                self.robot.data.body_quat_w[:, self.body_idx],
            ],
            dim=-1,
        )

    def _get_base_yaw_quat(self) -> torch.Tensor:
        return yaw_quat(self.robot.data.root_quat_w)

    def _update_anchor_center_w(self):
        """Update the yaw-aligned anchor center in the environment world frame."""
        root_pos = self.robot.data.root_pos_w
        self.anchor_center_w[:, 0] = root_pos[:, 0]
        self.anchor_center_w[:, 1] = root_pos[:, 1]
        self.anchor_center_w[:, 2] = self.cfg.anchor_z_world
        offset = self.anchor_center_offset_b.expand(self.num_envs, -1)
        if torch.any(offset != 0.0):
            self.anchor_center_w += quat_apply(self._get_base_yaw_quat(), offset)

    def _yaw_cmd_b_to_world_pos(self, pos_b: torch.Tensor) -> torch.Tensor:
        """Map yaw-command-frame Cartesian positions to the environment world frame."""
        center_w = self.anchor_center_w
        yaw_quat_w = self._get_base_yaw_quat()
        if pos_b.ndim == 2:
            return center_w + quat_apply(yaw_quat_w, pos_b)

        batch_size, num_points, _ = pos_b.shape
        center = center_w.unsqueeze(1).expand(batch_size, num_points, -1)
        yaw = yaw_quat_w.unsqueeze(1).expand(batch_size, num_points, 4).reshape(-1, 4)
        pos_flat = pos_b.reshape(-1, 3)
        return (center + quat_apply(yaw, pos_flat).reshape(batch_size, num_points, 3)).reshape(-1, 3)

    def _world_pos_to_yaw_cmd_b(self, pos_w: torch.Tensor) -> torch.Tensor:
        """Express world-frame positions in the yaw-command frame."""
        rel_w = pos_w - self.anchor_center_w
        return quat_apply_inverse(self._get_base_yaw_quat(), rel_w)

    def _sync_pose_obs_b(self):
        """Express the world-frame command in the current robot base frame for policy observations."""
        pos_b, quat_b = subtract_frame_transforms(
            self.robot.data.root_pos_w,
            self.robot.data.root_quat_w,
            self.pose_command_w[:, _POS_SLICE],
            self.pose_command_w[:, _QUAT_SLICE],
        )
        self.pose_command_b[:, _POS_SLICE] = pos_b
        if self.cfg.track_orientation:
            self.pose_command_b[:, _QUAT_SLICE] = quat_b

    def _resolve_env_ids(self, env_ids: Sequence[int]) -> torch.Tensor:
        if isinstance(env_ids, slice):
            return torch.arange(self.num_envs, device=self.device)[env_ids]
        if isinstance(env_ids, torch.Tensor):
            return env_ids.to(device=self.device, dtype=torch.long)
        return torch.as_tensor(list(env_ids), device=self.device, dtype=torch.long)
