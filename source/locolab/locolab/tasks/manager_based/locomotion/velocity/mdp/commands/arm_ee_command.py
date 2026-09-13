# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Arm end-effector pose command generator with trajectory interpolation.

Pose commands are sampled from a base-frame dataset and interpolated in the
yaw-aligned command frame. The world-frame target is reconstructed every step as:

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
from isaaclab.utils.math import compute_pose_error, quat_unique, yaw_quat

from .utils import (
    POS_SLICE,
    QUAT_SLICE,
    apply_base_assist_expand,
    fill_yaw_anchor_center,
    identity_pose_buffer,
    lerp_positions,
    quat_slerp,
    resolve_env_ids,
    sample_uniform_range,
    sync_pose_command_b,
    world_pos_to_yaw_cmd_b,
    world_quat_to_yaw_cmd_b,
    yaw_cmd_b_to_world_pos,
    yaw_cmd_b_to_world_quat,
)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from .commands_cfg import SampledArmEEPoseCommandCfg


class SampledArmEePoseCommand(CommandTerm):
    """Sampled end-effector pose commands with yaw-aligned anchoring.

    Dataset rows are stored in the robot base frame as
    ``[arm_q..., x, y, z, qw, qx, qy, qz]`` or legacy ``[qw, qx, qy, qz, x, y, z]``.

    Sampled Cartesian poses are interpolated in the command frame while the world
    target is reconstructed every step via a yaw-aligned anchor center.
    """

    cfg: SampledArmEEPoseCommandCfg

    def __init__(self, cfg: SampledArmEEPoseCommandCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)

        self.robot: Articulation = env.scene[cfg.asset_name]
        self.body_idx = self.robot.find_bodies(cfg.body_name)[0][0]

        self.pose_pool = self._load_pose_pool(cfg)
        self.num_samples = self.pose_pool.shape[0]
        if len(cfg.interpolation_modes) == 0:
            raise ValueError("interpolation_modes must contain at least one mode.")
        invalid_modes = set(cfg.interpolation_modes) - {"sphere", "cartesian"}
        if invalid_modes:
            raise ValueError(f"Unsupported interpolation modes: {sorted(invalid_modes)}")
        if cfg.interpolation_mode_probs is None:
            mode_probs = torch.ones(len(cfg.interpolation_modes), device=self.device, dtype=torch.float32)
        else:
            if len(cfg.interpolation_mode_probs) != len(cfg.interpolation_modes):
                raise ValueError(
                    "interpolation_mode_probs must have the same length as interpolation_modes, "
                    f"got {len(cfg.interpolation_mode_probs)} and {len(cfg.interpolation_modes)}."
                )
            if any(prob < 0.0 for prob in cfg.interpolation_mode_probs):
                raise ValueError(
                    f"interpolation_mode_probs must be non-negative, got {cfg.interpolation_mode_probs}."
                )
            if sum(cfg.interpolation_mode_probs) <= 0.0:
                raise ValueError("interpolation_mode_probs must sum to a positive value.")
            mode_probs = torch.tensor(cfg.interpolation_mode_probs, device=self.device, dtype=torch.float32)
        self._mode_probs = mode_probs / mode_probs.sum()

        height_range = cfg.workspace_expand_height_range
        pitch_range = cfg.workspace_expand_pitch_range
        if height_range[1] < height_range[0]:
            raise ValueError(f"workspace_expand_height_range must have min <= max, got {height_range}.")
        if pitch_range[1] < pitch_range[0]:
            raise ValueError(f"workspace_expand_pitch_range must have min <= max, got {pitch_range}.")
        self._workspace_expand_enabled = height_range != (0.0, 0.0) or pitch_range != (0.0, 0.0)
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

        self.pose_command_w = identity_pose_buffer(self.num_envs, self.device)
        self.pose_start_b = identity_pose_buffer(self.num_envs, self.device)
        self.pose_target_b = identity_pose_buffer(self.num_envs, self.device)
        self.pose_interp_b = identity_pose_buffer(self.num_envs, self.device)
        self.pose_command_b = identity_pose_buffer(self.num_envs, self.device)
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
        self._has_anchor_offset = cfg.anchor_center_offset_b != (0.0, 0.0, 0.0)

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
        msg += f"\tResampling time range: {self.cfg.resampling_time_range}\n"
        msg += f"\tTrack orientation: {self.cfg.track_orientation}\n"
        msg += f"\tMetrics update interval: {self.cfg.metrics_update_interval} control steps\n"
        msg += f"\tInterpolation time range: {self.cfg.interpolation_time_range}\n"
        msg += f"\tInterpolation modes: {self.cfg.interpolation_modes}\n"
        msg += f"\tInterpolation mode probs: {tuple(self._mode_probs.tolist())}\n"
        msg += f"\tWorkspace expand height range: {self.cfg.workspace_expand_height_range}\n"
        msg += f"\tWorkspace expand pitch range: {self.cfg.workspace_expand_pitch_range}\n"
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
        return self.pose_command_b[:, POS_SLICE]

    @property
    def command_w(self) -> torch.Tensor:
        """The desired end-effector command in the environment world frame."""
        if self.cfg.track_orientation:
            return self.pose_command_w
        return self.pose_command_w[:, POS_SLICE]

    def _update_metrics(self):
        self._metrics_update_counter = (self._metrics_update_counter + 1) % self.cfg.metrics_update_interval
        if self._metrics_update_counter != 0:
            return

        if self.cfg.track_orientation:
            pos_error, rot_error = compute_pose_error(
                self.pose_command_w[:, POS_SLICE],
                self.pose_command_w[:, QUAT_SLICE],
                self.robot.data.body_pos_w[:, self.body_idx],
                self.robot.data.body_quat_w[:, self.body_idx],
            )
            self.metrics["position_error"].add_(torch.norm(pos_error, dim=-1), alpha=self._metrics_update_scale)
            self.metrics["orientation_error"].add_(torch.norm(rot_error, dim=-1), alpha=self._metrics_update_scale)
            return

        pos_error = self.robot.data.body_pos_w[:, self.body_idx] - self.pose_command_w[:, POS_SLICE]
        self.metrics["position_error"].add_(torch.norm(pos_error, dim=-1), alpha=self._metrics_update_scale)

    def _resample_command(self, env_ids: Sequence[int]):
        env_ids_tensor = resolve_env_ids(env_ids, self.num_envs, self.device)
        if len(env_ids_tensor) == 0:
            return

        sample_ids = torch.randint(0, self.num_samples, (len(env_ids_tensor),), device=self.device)
        sampled_pose_b = self.pose_pool[sample_ids]
        if self._workspace_expand_enabled:
            num = len(env_ids_tensor)
            pitch = sample_uniform_range(self.cfg.workspace_expand_pitch_range, num, self.device)
            height = sample_uniform_range(self.cfg.workspace_expand_height_range, num, self.device)
            quat_in = sampled_pose_b[:, QUAT_SLICE] if self.cfg.track_orientation else None
            pos_out, quat_out = apply_base_assist_expand(
                sampled_pose_b[:, POS_SLICE], pitch, height, quat=quat_in
            )
            sampled_pose_b = sampled_pose_b.clone()
            sampled_pose_b[:, POS_SLICE] = pos_out
            if quat_out is not None:
                sampled_pose_b[:, QUAT_SLICE] = (
                    quat_unique(quat_out) if self.cfg.make_quat_unique else quat_out
                )

        self.pose_target_b[env_ids_tensor, POS_SLICE] = sampled_pose_b[:, POS_SLICE]
        if self.cfg.track_orientation:
            self.pose_target_b[env_ids_tensor, QUAT_SLICE] = sampled_pose_b[:, QUAT_SLICE]

        self.interp_progress[env_ids_tensor] = 0.0
        time_min, time_max = self.cfg.interpolation_time_range
        if time_max > time_min:
            self.interp_time_s[env_ids_tensor] = (
                torch.rand(len(env_ids_tensor), device=self.device) * (time_max - time_min) + time_min
            )
        else:
            self.interp_time_s[env_ids_tensor] = time_min
        if self._mode_probs.numel() == 1:
            self.use_sphere_interp[env_ids_tensor] = self._mode_is_sphere[0]
        else:
            mode_ids = torch.multinomial(self._mode_probs, num_samples=len(env_ids_tensor), replacement=True)
            self.use_sphere_interp[env_ids_tensor] = self._mode_is_sphere[mode_ids]
        self._set_interp_start_pose(env_ids_tensor)

    def _update_command(self):
        progress_step = self._env.step_dt / self.interp_time_s.clamp(min=1e-6)
        self.interp_progress.add_(progress_step).clamp_(max=1.0)
        blend = self.interp_progress.unsqueeze(-1)
        self.pose_interp_b[:, POS_SLICE] = lerp_positions(
            self.pose_start_b[:, POS_SLICE],
            self.pose_target_b[:, POS_SLICE],
            blend,
            center=self.sphere_center_offset_b,
            use_sphere=self.use_sphere_interp,
            only_sphere=self._only_sphere_mode,
            only_cartesian=self._only_cartesian_mode,
        )
        if self.cfg.track_orientation:
            self.pose_interp_b[:, QUAT_SLICE] = quat_slerp(
                self.pose_start_b[:, QUAT_SLICE],
                self.pose_target_b[:, QUAT_SLICE],
                self.interp_progress,
            )

        yaw_quat_w = yaw_quat(self.robot.data.root_quat_w)
        fill_yaw_anchor_center(
            self.anchor_center_w,
            self.robot.data.root_pos_w,
            self.cfg.anchor_z_world,
            self.anchor_center_offset_b,
            yaw_quat_w,
            apply_offset=self._has_anchor_offset,
        )
        self.pose_command_w[:, POS_SLICE] = yaw_cmd_b_to_world_pos(
            self.pose_interp_b[:, POS_SLICE],
            self.anchor_center_w,
            yaw_quat_w,
        )
        if self.cfg.track_orientation:
            self.pose_command_w[:, QUAT_SLICE] = yaw_cmd_b_to_world_quat(
                self.pose_interp_b[:, QUAT_SLICE], yaw_quat_w
            )
        sync_pose_command_b(
            self.pose_command_b,
            self.pose_command_w,
            self.robot.data.root_pos_w,
            self.robot.data.root_quat_w,
            self.cfg.track_orientation,
        )

    def _set_interp_start_pose(self, env_ids: torch.Tensor):
        """Set interpolation start to the current EE pose on the first command, else last command."""
        if not self.robot.is_initialized:
            self.pose_start_b[env_ids, POS_SLICE] = self.pose_interp_b[env_ids, POS_SLICE]
            if self.cfg.track_orientation:
                self.pose_start_b[env_ids, QUAT_SLICE] = self.pose_interp_b[env_ids, QUAT_SLICE]
            return

        yaw_quat_w = yaw_quat(self.robot.data.root_quat_w)
        fill_yaw_anchor_center(
            self.anchor_center_w,
            self.robot.data.root_pos_w,
            self.cfg.anchor_z_world,
            self.anchor_center_offset_b,
            yaw_quat_w,
            apply_offset=self._has_anchor_offset,
        )
        is_first = (self.command_counter[env_ids] == 0).unsqueeze(-1)
        start_pos = torch.where(
            is_first,
            world_pos_to_yaw_cmd_b(
                self.robot.data.body_pos_w[env_ids, self.body_idx],
                self.anchor_center_w[env_ids],
                yaw_quat_w[env_ids],
            ),
            self.pose_interp_b[env_ids, POS_SLICE],
        )
        self.pose_start_b[env_ids, POS_SLICE] = start_pos
        self.pose_interp_b[env_ids, POS_SLICE] = start_pos
        if self.cfg.track_orientation:
            start_quat = torch.where(
                is_first,
                world_quat_to_yaw_cmd_b(
                    self.robot.data.body_quat_w[env_ids, self.body_idx],
                    yaw_quat_w[env_ids],
                ),
                self.pose_interp_b[env_ids, QUAT_SLICE],
            )
            self.pose_start_b[env_ids, QUAT_SLICE] = start_quat
            self.pose_interp_b[env_ids, QUAT_SLICE] = start_quat

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

        yaw_quat_w = yaw_quat(self.robot.data.root_quat_w)
        goal_pos_w = yaw_cmd_b_to_world_pos(
            self.pose_target_b[:, POS_SLICE],
            self.anchor_center_w,
            yaw_quat_w,
        )
        if self.cfg.track_orientation:
            body_link_pose_w = self.robot.data.body_link_pose_w[:, self.body_idx]
            goal_quat_w = yaw_cmd_b_to_world_quat(self.pose_target_b[:, QUAT_SLICE], yaw_quat_w)
            self.goal_visualizer.visualize(goal_pos_w, goal_quat_w)
            self.current_visualizer.visualize(body_link_pose_w[:, POS_SLICE], body_link_pose_w[:, QUAT_SLICE])
        else:
            self.goal_visualizer.visualize(goal_pos_w)
            self.current_visualizer.visualize(self.robot.data.body_pos_w[:, self.body_idx])

        self.interp_path_visualizer.visualize(self._compute_interp_path_positions_w())

    def _compute_interp_path_positions_w(self) -> torch.Tensor:
        """Sample the planned interpolation path in the world frame for debug markers."""
        num_points = max(self.cfg.interp_path_num_points, 2)
        blend = torch.linspace(0.0, 1.0, num_points, device=self.device).view(1, num_points, 1)
        start_pos = self.pose_start_b[:, POS_SLICE].unsqueeze(1).expand(-1, num_points, -1)
        target_pos = self.pose_target_b[:, POS_SLICE].unsqueeze(1).expand(-1, num_points, -1)
        interp_center = self.sphere_center_offset_b.unsqueeze(1).expand(-1, num_points, -1)
        path_pos_b = lerp_positions(
            start_pos,
            target_pos,
            blend,
            center=interp_center,
            use_sphere=self.use_sphere_interp,
            only_sphere=self._only_sphere_mode,
            only_cartesian=self._only_cartesian_mode,
        )
        return yaw_cmd_b_to_world_pos(path_pos_b, self.anchor_center_w, yaw_quat(self.robot.data.root_quat_w))

    def _load_pose_pool(self, cfg: SampledArmEEPoseCommandCfg) -> torch.Tensor:
        dataset_path = Path(cfg.pose_dataset_path).expanduser().resolve()
        if not dataset_path.is_file():
            raise FileNotFoundError(f"Arm EE pose dataset not found: {dataset_path}")

        dataset = np.load(dataset_path)
        if "arm_joint_q_and_ee_pose" in dataset:
            arm_joint_q_and_ee_pose = torch.as_tensor(
                dataset["arm_joint_q_and_ee_pose"], device=self.device, dtype=torch.float32
            )
            if arm_joint_q_and_ee_pose.ndim != 2 or arm_joint_q_and_ee_pose.shape[1] < 7:
                raise ValueError(
                    "Expected arm_joint_q_and_ee_pose shape (N, n_arm + 7), "
                    f"got {tuple(arm_joint_q_and_ee_pose.shape)}"
                )
            pose_pool = arm_joint_q_and_ee_pose[:, -7:]
        elif "ee_pose" in dataset:
            ee_pose = torch.as_tensor(dataset["ee_pose"], device=self.device, dtype=torch.float32)
            if ee_pose.ndim != 2 or ee_pose.shape[1] != 7:
                raise ValueError(f"Expected ee_pose shape (N, 7), got {tuple(ee_pose.shape)}")
            pose_pool = torch.cat([ee_pose[:, 4:7], ee_pose[:, :4]], dim=1)
        else:
            raise KeyError(
                "Arm EE pose dataset must contain 'arm_joint_q_and_ee_pose' or 'ee_pose', "
                f"got keys: {list(dataset.keys())}"
            )

        if cfg.make_quat_unique:
            pose_pool[:, QUAT_SLICE] = quat_unique(pose_pool[:, QUAT_SLICE])
        return pose_pool
