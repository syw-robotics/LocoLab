# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Arm end-effector command that replays offline, physically valid trajectories.

Each dataset trajectory is a dense waypoint sequence in the robot base frame.
Playback interpolates only between adjacent waypoints of the same trajectory, so
the commanded path stays on a pre-validated workspace curve instead of blending
two independently sampled poses.
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
    quat_slerp,
    resolve_env_ids,
    sample_uniform_range,
    sync_pose_command_b,
    yaw_cmd_b_to_world_pos,
    yaw_cmd_b_to_world_quat,
)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from .commands_cfg import SampledArmEETrajCommandCfg


class SampledArmEeTrajCommand(CommandTerm):
    """Replay precomputed EE trajectories with yaw-aligned anchoring.

    Dataset array ``ee_pose`` has shape ``(N, T, 7)`` and layout
    ``[x, y, z, qw, qx, qy, qz]`` in the robot base frame. Optional ``q`` / ``qdot``
    arrays are ignored at command time.

    Waypoint playback stays in the command frame while the world target is
    reconstructed every step from a yaw-aligned anchor center. Policy observations
    receive the current target in the current base frame; rewards should read
    :attr:`command_w`.
    """

    cfg: SampledArmEETrajCommandCfg

    def __init__(self, cfg: SampledArmEETrajCommandCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)

        if not isinstance(cfg.metrics_update_interval, int) or cfg.metrics_update_interval < 1:
            raise ValueError(
                "Expected metrics_update_interval to be a positive integer, "
                f"got {cfg.metrics_update_interval}."
            )
        playback_min, playback_max = cfg.playback_time_range
        if playback_min < 0.0 or playback_max < playback_min:
            raise ValueError(
                "Expected playback_time_range (min, max) with 0 <= min <= max, "
                f"got {cfg.playback_time_range}."
            )

        self.robot: Articulation = env.scene[cfg.asset_name]
        self.body_idx = self.robot.find_bodies(cfg.body_name)[0][0]

        self.traj_pool = self._load_traj_pool(cfg)
        self.num_traj = self.traj_pool.shape[0]
        self.num_waypoints = self.traj_pool.shape[1]
        height_range = cfg.workspace_expand_height_range
        pitch_range = cfg.workspace_expand_pitch_range
        if height_range[1] < height_range[0]:
            raise ValueError(f"workspace_expand_height_range must have min <= max, got {height_range}.")
        if pitch_range[1] < pitch_range[0]:
            raise ValueError(f"workspace_expand_pitch_range must have min <= max, got {pitch_range}.")
        self._workspace_expand_enabled = height_range != (0.0, 0.0) or pitch_range != (0.0, 0.0)
        self.expand_pitch = torch.zeros(self.num_envs, device=self.device)
        self.expand_height = torch.zeros(self.num_envs, device=self.device)

        self.pose_command_w = identity_pose_buffer(self.num_envs, self.device)
        self.pose_command_b = identity_pose_buffer(self.num_envs, self.device)
        self.pose_interp_b = identity_pose_buffer(self.num_envs, self.device)
        self.anchor_center_w = torch.zeros(self.num_envs, 3, device=self.device)

        self.traj_index = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self.playback_progress = torch.ones(self.num_envs, device=self.device)
        self.playback_time_s = torch.full(
            (self.num_envs,),
            playback_min,
            device=self.device,
            dtype=torch.float32,
        )
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
        msg = "SampledArmEeTrajCommand:\n"
        msg += f"\tCommand dimension: {tuple(self.command.shape[1:])}\n"
        msg += f"\tResampling time range: {self.cfg.resampling_time_range}\n"
        msg += f"\tPlayback time range: {self.cfg.playback_time_range}\n"
        msg += f"\tTrack orientation: {self.cfg.track_orientation}\n"
        msg += f"\tWorkspace expand height range: {self.cfg.workspace_expand_height_range}\n"
        msg += f"\tWorkspace expand pitch range: {self.cfg.workspace_expand_pitch_range}\n"
        msg += f"\tTrajectory dataset: {self.cfg.traj_dataset_path}\n"
        msg += f"\tNumber of trajectories: {self.num_traj}\n"
        msg += f"\tWaypoints per trajectory: {self.num_waypoints}\n"
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

        self.traj_index[env_ids_tensor] = torch.randint(
            0, self.num_traj, (len(env_ids_tensor),), device=self.device
        )
        self.playback_progress[env_ids_tensor] = 0.0
        time_min, time_max = self.cfg.playback_time_range
        if time_max > time_min:
            self.playback_time_s[env_ids_tensor] = (
                torch.rand(len(env_ids_tensor), device=self.device) * (time_max - time_min) + time_min
            )
        else:
            self.playback_time_s[env_ids_tensor] = time_min

        if self._workspace_expand_enabled:
            num = len(env_ids_tensor)
            self.expand_pitch[env_ids_tensor] = sample_uniform_range(
                self.cfg.workspace_expand_pitch_range, num, self.device
            )
            self.expand_height[env_ids_tensor] = sample_uniform_range(
                self.cfg.workspace_expand_height_range, num, self.device
            )
        else:
            self.expand_pitch[env_ids_tensor] = 0.0
            self.expand_height[env_ids_tensor] = 0.0

    def _update_command(self):
        progress_step = self._env.step_dt / self.playback_time_s.clamp(min=1e-6)
        self.playback_progress.add_(progress_step).clamp_(max=1.0)
        self._sample_interp_pose(self.playback_progress)
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

    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis:
            if not hasattr(self, "goal_visualizer"):
                if self.cfg.track_orientation:
                    self.goal_visualizer = VisualizationMarkers(self.cfg.goal_ee_visualizer_cfg)
                    self.current_visualizer = VisualizationMarkers(self.cfg.current_ee_visualizer_cfg)
                else:
                    self.goal_visualizer = VisualizationMarkers(self.cfg.goal_pos_visualizer_cfg)
                    self.current_visualizer = VisualizationMarkers(self.cfg.current_pos_visualizer_cfg)
                self.traj_path_visualizer = VisualizationMarkers(self.cfg.traj_path_visualizer_cfg)
            self.goal_visualizer.set_visibility(True)
            self.current_visualizer.set_visibility(True)
            self.traj_path_visualizer.set_visibility(True)
        else:
            if hasattr(self, "goal_visualizer"):
                self.goal_visualizer.set_visibility(False)
                self.current_visualizer.set_visibility(False)
                self.traj_path_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        if not self.robot.is_initialized:
            return

        if self.cfg.track_orientation:
            body_link_pose_w = self.robot.data.body_link_pose_w[:, self.body_idx]
            self.goal_visualizer.visualize(self.pose_command_w[:, POS_SLICE], self.pose_command_w[:, QUAT_SLICE])
            self.current_visualizer.visualize(body_link_pose_w[:, POS_SLICE], body_link_pose_w[:, QUAT_SLICE])
        else:
            self.goal_visualizer.visualize(self.pose_command_w[:, POS_SLICE])
            self.current_visualizer.visualize(self.robot.data.body_pos_w[:, self.body_idx])

        self.traj_path_visualizer.visualize(self._compute_traj_path_positions_w())

    def _load_traj_pool(self, cfg: SampledArmEETrajCommandCfg) -> torch.Tensor:
        dataset_path = Path(cfg.traj_dataset_path).expanduser().resolve()
        if not dataset_path.is_file():
            raise FileNotFoundError(f"Arm EE trajectory dataset not found: {dataset_path}")

        with np.load(dataset_path) as dataset:
            if "ee_pose" not in dataset:
                raise KeyError(
                    f"Arm EE trajectory dataset must contain 'ee_pose', got keys: {list(dataset.keys())}"
                )
            ee_pose = np.array(dataset["ee_pose"], dtype=np.float32, copy=True)

        traj_pool = torch.as_tensor(ee_pose, device=self.device, dtype=torch.float32)
        if traj_pool.ndim != 3 or traj_pool.shape[-1] != 7:
            raise ValueError(f"Expected ee_pose shape (N, T, 7), got {tuple(traj_pool.shape)}")
        if traj_pool.shape[0] < 1 or traj_pool.shape[1] < 2:
            raise ValueError(
                f"Expected at least one trajectory with >= 2 waypoints, got {tuple(traj_pool.shape)}"
            )
        if not torch.isfinite(traj_pool).all():
            raise ValueError("Trajectory dataset contains non-finite values.")

        if cfg.make_quat_unique:
            traj_pool = traj_pool.clone()
            traj_pool[..., QUAT_SLICE] = quat_unique(traj_pool[..., QUAT_SLICE])
        return traj_pool

    def _sample_interp_pose(self, progress: torch.Tensor):
        """Write the active trajectory sample at ``progress`` in ``[0, 1]`` into ``pose_interp_b``."""
        max_idx = self.num_waypoints - 1
        u = progress.clamp(0.0, 1.0) * max_idx
        i0 = torch.floor(u).long().clamp(max=max_idx - 1)
        i1 = i0 + 1
        alpha = (u - i0.float()).unsqueeze(-1)
        start = self.traj_pool[self.traj_index, i0]
        target = self.traj_pool[self.traj_index, i1]
        pos = torch.lerp(start[:, POS_SLICE], target[:, POS_SLICE], alpha)
        quat = None
        if self.cfg.track_orientation:
            quat = quat_slerp(start[:, QUAT_SLICE], target[:, QUAT_SLICE], alpha)
        if self._workspace_expand_enabled:
            pos, quat = apply_base_assist_expand(pos, self.expand_pitch, self.expand_height, quat=quat)
        self.pose_interp_b[:, POS_SLICE] = pos
        if quat is not None:
            self.pose_interp_b[:, QUAT_SLICE] = quat_unique(quat) if self.cfg.make_quat_unique else quat

    def _compute_traj_path_positions_w(self) -> torch.Tensor:
        """Return the active trajectory waypoints in the world frame for debug markers."""
        path_b = self.traj_pool[self.traj_index]
        path_pos = path_b[..., POS_SLICE]
        if self._workspace_expand_enabled:
            path_pos, _ = apply_base_assist_expand(path_pos, self.expand_pitch, self.expand_height)
        return yaw_cmd_b_to_world_pos(path_pos, self.anchor_center_w, yaw_quat(self.robot.data.root_quat_w))
