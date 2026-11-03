# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Single-clip motion command for whole-body tracking.

The reference is one resampled motion shared by every environment. Reset samples
a time index inside that clip and writes the reference state into the robot.

Reading order: initialization, reference/robot state views, command lifecycle,
sampling, simulator state writes, and initialization-time body kinematics.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation
from isaaclab.managers import CommandTerm

from .utils import finite_difference_ang_vel, relative_body_poses

from .motion_loader import MotionLoader

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

    from .motion_command_cfg import MotionCommandCfg


class MotionCommand(CommandTerm):
    """Track one motion clip. The command is reference joint position and velocity."""

    cfg: MotionCommandCfg

    def __init__(self, cfg: MotionCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)

        if not isinstance(cfg.metrics_update_interval, int) or cfg.metrics_update_interval < 1:
            raise ValueError("metrics_update_interval must be a positive integer.")
        if not isinstance(cfg.ghost_max_envs, int) or cfg.ghost_max_envs < 1:
            raise ValueError("ghost_max_envs must be a positive integer.")
        self._metrics_update_counter = 0

        # Resolve robot indices once; all state views below follow the configured order.
        self.robot: Articulation = env.scene[cfg.asset_name]
        joint_ids, joint_names = self.robot.find_joints(list(cfg.joint_names), preserve_order=True)
        body_ids, body_names = self.robot.find_bodies(list(cfg.body_names), preserve_order=True)
        if tuple(joint_names) != tuple(cfg.joint_names):
            raise ValueError(f"Robot joints {joint_names} do not match the configured order {cfg.joint_names}.")
        if tuple(body_names) != tuple(cfg.body_names):
            raise ValueError(f"Robot bodies {body_names} do not match the configured order {cfg.body_names}.")
        if cfg.anchor_body_name not in cfg.body_names:
            raise ValueError(f"Anchor body {cfg.anchor_body_name} is not in body_names.")

        self._joint_ids = list(joint_ids)
        self._body_ids = list(body_ids)
        self.joint_ids = torch.tensor(self._joint_ids, dtype=torch.long, device=self.device)
        self.body_ids = torch.tensor(self._body_ids, dtype=torch.long, device=self.device)
        self.robot_anchor_body_index = self._body_ids[cfg.body_names.index(cfg.anchor_body_name)]
        self.motion_anchor_body_index = cfg.body_names.index(cfg.anchor_body_name)
        self.num_bodies = len(cfg.body_names)

        # Build the shared clip, including body trajectories absent from the qpos file.
        self.motion = MotionLoader(cfg.motion_file, cfg.joint_names, env.step_dt, self.device)
        self._compute_body_kinematics()

        # Each environment has its own frame cursor and robot-aligned reference poses.
        self.time_steps = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.body_pos_relative_w = torch.zeros(self.num_envs, self.num_bodies, 3, device=self.device)
        self.body_quat_relative_w = torch.zeros(self.num_envs, self.num_bodies, 4, device=self.device)
        self.body_quat_relative_w[..., 3] = 1.0

        # Group frames into roughly one-second bins for failure-weighted reset sampling.
        self.bin_count = int(self.motion.time_step_total // (1.0 / env.step_dt)) + 1
        self.bin_failed_count = torch.zeros(self.bin_count, dtype=torch.float, device=self.device)
        self._current_bin_failed = torch.zeros(self.bin_count, dtype=torch.float, device=self.device)
        kernel = torch.tensor(
            [cfg.adaptive_lambda**i for i in range(cfg.adaptive_kernel_size)],
            dtype=torch.float,
            device=self.device,
        )
        self.kernel = kernel / kernel.sum()

        self.metrics["error_anchor_pos"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_anchor_rot"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_anchor_lin_vel"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_anchor_ang_vel"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_body_pos"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_body_rot"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_body_lin_vel"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_body_ang_vel"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_joint_pos"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_joint_vel"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["sampling_entropy"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["sampling_top1_prob"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["sampling_top1_bin"] = torch.zeros(self.num_envs, device=self.device)

        print(
            f"[INFO] Motion tracking clip: {self.motion.motion_file} "
            f"({self.motion.time_step_total} frames at {1.0 / self.motion.output_dt:.1f} Hz, "
            f"source {1.0 / self.motion.input_dt:.1f} Hz)."
        )
        # CommandTerm's constructor runs before the robot and reference are available.
        self.set_debug_vis(cfg.debug_vis)

    # Reference state views: select each environment's frame from the shared clip.
    # Only positions need the scene origin offset to become simulation world positions.
    @property
    def command(self) -> torch.Tensor:
        """Policy command of shape (num_envs, 2 * num_joints): positions then velocities."""
        return torch.cat((self.joint_pos, self.joint_vel), dim=1)

    @property
    def joint_pos(self) -> torch.Tensor:
        return self.motion.joint_pos[self.time_steps]

    @property
    def joint_vel(self) -> torch.Tensor:
        return self.motion.joint_vel[self.time_steps]

    @property
    def body_pos_w(self) -> torch.Tensor:
        return self.motion.body_pos_w[self.time_steps] + self._env.scene.env_origins[:, None, :]

    @property
    def body_quat_w(self) -> torch.Tensor:
        return self.motion.body_quat_w[self.time_steps]

    @property
    def body_lin_vel_w(self) -> torch.Tensor:
        return self.motion.body_lin_vel_w[self.time_steps]

    @property
    def body_ang_vel_w(self) -> torch.Tensor:
        return self.motion.body_ang_vel_w[self.time_steps]

    @property
    def anchor_pos_w(self) -> torch.Tensor:
        return self.motion.body_pos_w[self.time_steps, self.motion_anchor_body_index] + self._env.scene.env_origins

    @property
    def anchor_quat_w(self) -> torch.Tensor:
        return self.motion.body_quat_w[self.time_steps, self.motion_anchor_body_index]

    @property
    def anchor_lin_vel_w(self) -> torch.Tensor:
        return self.motion.body_lin_vel_w[self.time_steps, self.motion_anchor_body_index]

    @property
    def anchor_ang_vel_w(self) -> torch.Tensor:
        return self.motion.body_ang_vel_w[self.time_steps, self.motion_anchor_body_index]

    # Robot state views: expose simulated state in the same joint/body order as the clip.
    @property
    def robot_joint_pos(self) -> torch.Tensor:
        return self.robot.data.joint_pos.torch[:, self.joint_ids]

    @property
    def robot_joint_vel(self) -> torch.Tensor:
        return self.robot.data.joint_vel.torch[:, self.joint_ids]

    @property
    def robot_body_pos_w(self) -> torch.Tensor:
        return self.robot.data.body_link_pos_w.torch[:, self.body_ids]

    @property
    def robot_body_quat_w(self) -> torch.Tensor:
        return self.robot.data.body_link_quat_w.torch[:, self.body_ids]

    @property
    def robot_body_lin_vel_w(self) -> torch.Tensor:
        return self.robot.data.body_link_lin_vel_w.torch[:, self.body_ids]

    @property
    def robot_body_ang_vel_w(self) -> torch.Tensor:
        return self.robot.data.body_link_ang_vel_w.torch[:, self.body_ids]

    @property
    def robot_anchor_pos_w(self) -> torch.Tensor:
        return self.robot.data.body_link_pos_w.torch[:, self.robot_anchor_body_index]

    @property
    def robot_anchor_quat_w(self) -> torch.Tensor:
        return self.robot.data.body_link_quat_w.torch[:, self.robot_anchor_body_index]

    @property
    def robot_anchor_lin_vel_w(self) -> torch.Tensor:
        return self.robot.data.body_link_lin_vel_w.torch[:, self.robot_anchor_body_index]

    @property
    def robot_anchor_ang_vel_w(self) -> torch.Tensor:
        return self.robot.data.body_link_ang_vel_w.torch[:, self.robot_anchor_body_index]

    # CommandTerm lifecycle hooks: diagnostics, reset sampling, and per-step advancement.
    def _update_metrics(self):
        """Refresh diagnostic errors at the configured interval."""
        self._metrics_update_counter = (self._metrics_update_counter + 1) % self.cfg.metrics_update_interval
        if self._metrics_update_counter != 0:
            return

        self.metrics["error_anchor_pos"] = torch.norm(self.anchor_pos_w - self.robot_anchor_pos_w, dim=-1)
        self.metrics["error_anchor_rot"] = math_utils.quat_error_magnitude(self.anchor_quat_w, self.robot_anchor_quat_w)
        self.metrics["error_anchor_lin_vel"] = torch.norm(self.anchor_lin_vel_w - self.robot_anchor_lin_vel_w, dim=-1)
        self.metrics["error_anchor_ang_vel"] = torch.norm(self.anchor_ang_vel_w - self.robot_anchor_ang_vel_w, dim=-1)
        self.metrics["error_body_pos"] = torch.norm(self.body_pos_relative_w - self.robot_body_pos_w, dim=-1).mean(dim=-1)
        self.metrics["error_body_rot"] = math_utils.quat_error_magnitude(
            self.body_quat_relative_w, self.robot_body_quat_w
        ).mean(dim=-1)
        self.metrics["error_body_lin_vel"] = torch.norm(self.body_lin_vel_w - self.robot_body_lin_vel_w, dim=-1).mean(
            dim=-1
        )
        self.metrics["error_body_ang_vel"] = torch.norm(self.body_ang_vel_w - self.robot_body_ang_vel_w, dim=-1).mean(
            dim=-1
        )
        self.metrics["error_joint_pos"] = torch.norm(self.joint_pos - self.robot_joint_pos, dim=-1)
        self.metrics["error_joint_vel"] = torch.norm(self.joint_vel - self.robot_joint_vel, dim=-1)

    def _resample_command(self, env_ids: Sequence[int] | slice):
        """Choose new clip frames and reset the selected robots around those states."""
        ids = self._as_env_ids(env_ids)
        if ids.numel() == 0:
            return
        # sampling_mode includes "start", "uniform", and "adaptive" (failure-weighted). The first two are deterministic.
        if self.cfg.sampling_mode == "start":
            self.time_steps[ids] = 0
        elif self.cfg.sampling_mode == "uniform":
            self._uniform_sampling(ids)
        else:
            self._adaptive_sampling(ids)
        self._write_reference_state(ids, perturb=True)
        # Refresh simulated link poses before aligning the reference to the new robot pose.
        self._env.sim.forward()
        self.update_relative_body_poses()

    def _update_command(self):
        """Advance one control frame; reaching the clip end triggers a fresh reset sample."""
        self.time_steps += 1
        wrap_ids = torch.where(self.time_steps >= self.motion.time_step_total)[0]
        if wrap_ids.numel() > 0:
            self._resample_command(wrap_ids)
        else:
            self.update_relative_body_poses()

        if self.cfg.sampling_mode == "adaptive":
            # Smooth recent failures into the history used by subsequent reset samples.
            self.bin_failed_count = (
                self.cfg.adaptive_alpha * self._current_bin_failed
                + (1.0 - self.cfg.adaptive_alpha) * self.bin_failed_count
            )
            self._current_bin_failed.zero_()

    def update_relative_body_poses(self) -> None:
        """Yaw-align the reference bodies to the robot anchor, keeping the reference height."""
        position, orientation = relative_body_poses(
            self.anchor_pos_w,
            self.anchor_quat_w,
            self.robot_anchor_pos_w,
            self.robot_anchor_quat_w,
            self.body_pos_w,
            self.body_quat_w,
        )
        self.body_pos_relative_w[:] = position
        self.body_quat_relative_w[:] = orientation

    # Reset sampling: bias towards difficult clip segments or draw frames uniformly.
    def _adaptive_sampling(self, env_ids: torch.Tensor) -> None:
        """Record termination bins, then sample a bin and a random frame within it."""
        # Count failures at the outgoing frames, before replacing their time indices.
        episode_failed = self._env.termination_manager.terminated[env_ids]
        if torch.any(episode_failed):
            current_bin_index = torch.clamp(
                (self.time_steps * self.bin_count) // max(self.motion.time_step_total, 1),
                0,
                self.bin_count - 1,
            )
            fail_bins = current_bin_index[env_ids][episode_failed]
            self._current_bin_failed[:] = torch.bincount(fail_bins, minlength=self.bin_count).float()

        # Keep every bin reachable; spread failure weights towards earlier nearby bins.
        sampling_probabilities = self.bin_failed_count + self.cfg.adaptive_uniform_ratio / float(self.bin_count)
        sampling_probabilities = torch.nn.functional.pad(
            sampling_probabilities.view(1, 1, -1),
            (0, self.cfg.adaptive_kernel_size - 1),
            mode="replicate",
        )
        sampling_probabilities = torch.nn.functional.conv1d(sampling_probabilities, self.kernel.view(1, 1, -1)).view(-1)
        sampling_probabilities = sampling_probabilities / sampling_probabilities.sum()

        sampled_bins = torch.multinomial(sampling_probabilities, len(env_ids), replacement=True)
        self.time_steps[env_ids] = (
            (sampled_bins + math_utils.sample_uniform(0.0, 1.0, (len(env_ids),), device=self.device))
            / self.bin_count
            * (self.motion.time_step_total - 1)
        ).long()

        if self._metrics_update_counter == 0:
            entropy = -(sampling_probabilities * (sampling_probabilities + 1.0e-12).log()).sum()
            entropy_norm = entropy / math.log(self.bin_count) if self.bin_count > 1 else torch.ones((), device=self.device)
            top_prob, top_bin = sampling_probabilities.max(dim=0)
            self.metrics["sampling_entropy"][:] = entropy_norm
            self.metrics["sampling_top1_prob"][:] = top_prob
            self.metrics["sampling_top1_bin"][:] = top_bin.float() / self.bin_count

    def _uniform_sampling(self, env_ids: torch.Tensor) -> None:
        self.time_steps[env_ids] = torch.randint(
            0, self.motion.time_step_total, (len(env_ids),), device=self.device
        )
        self.metrics["sampling_entropy"][:] = 1.0
        self.metrics["sampling_top1_prob"][:] = 1.0 / self.bin_count
        self.metrics["sampling_top1_bin"][:] = 0.5

    def _write_reference_state(self, env_ids: torch.Tensor, perturb: bool) -> None:
        """Write the selected reference frames, optionally adding configured reset noise."""
        root_pos = self.motion.root_pos[self.time_steps[env_ids]] + self._env.scene.env_origins[env_ids]
        root_quat = self.motion.root_quat[self.time_steps[env_ids]].clone()
        root_lin_vel = self.motion.root_lin_vel[self.time_steps[env_ids]].clone()
        root_ang_vel = self.motion.root_ang_vel[self.time_steps[env_ids]].clone()
        joint_pos = self.motion.joint_pos[self.time_steps[env_ids]].clone()
        joint_vel = self.motion.joint_vel[self.time_steps[env_ids]]

        if perturb:
            pose_noise = self._sample_range(("x", "y", "z", "roll", "pitch", "yaw"), self.cfg.pose_range, len(env_ids))
            root_pos = root_pos + pose_noise[:, 0:3]
            root_quat = math_utils.quat_mul(
                math_utils.quat_from_euler_xyz(pose_noise[:, 3], pose_noise[:, 4], pose_noise[:, 5]),
                root_quat,
            )
            vel_noise = self._sample_range(
                ("x", "y", "z", "roll", "pitch", "yaw"), self.cfg.velocity_range, len(env_ids)
            )
            root_lin_vel = root_lin_vel + vel_noise[:, 0:3]
            root_ang_vel = root_ang_vel + vel_noise[:, 3:6]
            joint_pos = joint_pos + math_utils.sample_uniform(
                self.cfg.joint_position_range[0],
                self.cfg.joint_position_range[1],
                joint_pos.shape,
                device=self.device,
            )

        # Noise may move joints outside their permitted range; clamp before writing.
        soft_limits = self.robot.data.soft_joint_pos_limits.torch[env_ids][:, self.joint_ids]
        joint_pos = torch.clip(joint_pos, soft_limits[..., 0], soft_limits[..., 1])
        self.robot.write_root_pose_to_sim_index(
            root_pose=torch.cat((root_pos, root_quat), dim=-1),
            env_ids=env_ids,
            skip_forward=True,
        )
        self.robot.write_root_velocity_to_sim_index(
            root_velocity=torch.cat((root_lin_vel, root_ang_vel), dim=-1),
            env_ids=env_ids,
            skip_forward=True,
        )
        # Position writes must invalidate link poses; velocity writes alone do not on ovphysx.
        self.robot.write_joint_position_to_sim_index(
            position=joint_pos,
            joint_ids=self._joint_ids,
            env_ids=env_ids,
            skip_forward=False,
        )
        self.robot.write_joint_velocity_to_sim_index(
            velocity=joint_vel,
            joint_ids=self._joint_ids,
            env_ids=env_ids,
            skip_forward=False,
        )

    def _compute_body_kinematics(self) -> None:
        """Precompute body trajectories once, using simulator forward kinematics.

        Temporarily use parallel environments to evaluate different clip frames;
        stored positions exclude scene origins so every environment can reuse them.
        """
        frames = self.motion.time_step_total
        body_pos = torch.empty(frames, self.num_bodies, 3, device=self.device)
        body_quat = torch.empty(frames, self.num_bodies, 4, device=self.device)
        origins = self._env.scene.env_origins
        batch_size = self.num_envs
        # One batch evaluates up to num_envs frames without advancing simulation time.
        for start in range(0, frames, batch_size):
            stop = min(start + batch_size, frames)
            count = stop - start
            env_ids = torch.arange(count, dtype=torch.int32, device=self.device)
            root_pose = torch.cat((self.motion.root_pos[start:stop] + origins[:count], self.motion.root_quat[start:stop]), dim=-1)
            root_vel = torch.cat((self.motion.root_lin_vel[start:stop], self.motion.root_ang_vel[start:stop]), dim=-1)
            self.robot.write_root_pose_to_sim_index(root_pose=root_pose, env_ids=env_ids, skip_forward=True)
            self.robot.write_root_velocity_to_sim_index(root_velocity=root_vel, env_ids=env_ids, skip_forward=True)
            # Invalidate pose caches for each batch before reading link transforms.
            self.robot.write_joint_position_to_sim_index(
                position=self.motion.joint_pos[start:stop],
                joint_ids=self._joint_ids,
                env_ids=env_ids,
                skip_forward=False,
            )
            self.robot.write_joint_velocity_to_sim_index(
                velocity=self.motion.joint_vel[start:stop],
                joint_ids=self._joint_ids,
                env_ids=env_ids,
                skip_forward=False,
            )
            self._env.sim.forward()
            body_pos[start:stop] = self.robot.data.body_link_pos_w.torch[:count, self.body_ids] - origins[:count, None, :]
            body_quat[start:stop] = self.robot.data.body_link_quat_w.torch[:count, self.body_ids]

        body_lin_vel = torch.gradient(body_pos, spacing=self.motion.output_dt, dim=0)[0]
        body_ang_vel = finite_difference_ang_vel(body_quat, self.motion.output_dt)
        self.motion.set_body_state(body_pos, body_quat, body_lin_vel, body_ang_vel)

    # Debug rendering uses visual meshes only; it never writes simulator state.
    def _set_debug_vis_impl(self, debug_vis: bool):
        self._ghost_debug_vis = debug_vis
        if hasattr(self, "_ghost") and hasattr(self._ghost, "markers"):
            self._ghost.markers.set_visibility(debug_vis)

    def get_motion_ghost(self):
        """Lazily share mesh and CPU motion caches across native and browser views."""
        if not hasattr(self, "_ghost"):
            from .motion_ghost import MotionGhost

            self._ghost = MotionGhost(self)
        return self._ghost

    def _debug_vis_callback(self, event):
        if not self._ghost_debug_vis or not self.robot.is_initialized:
            return
        sim = self._env.sim
        # Live-plot callbacks can run without a marker consumer. Leave browser-only
        # rendering to Viser rather than building and updating a second ghost here.
        if not (
            sim.has_gui
            or sim.has_offscreen_render
            or sim.get_setting("/isaaclab/render/rtx_sensors")
            or any(viz.supports_markers() and viz.cfg.enable_markers for viz in sim.visualizers)
        ):
            return
        ghost = self.get_motion_ghost()
        if not hasattr(ghost, "markers"):
            ghost.create_markers(self)
        ghost.update_markers(self)

    # Small input helpers shared by reset and state-writing paths.
    def _sample_range(self, keys: tuple[str, ...], ranges: dict[str, tuple[float, float]], count: int) -> torch.Tensor:
        """Sample independent noise components; unspecified keys receive zero noise."""
        bounds = torch.tensor([ranges.get(key, (0.0, 0.0)) for key in keys], device=self.device)
        return math_utils.sample_uniform(bounds[:, 0], bounds[:, 1], (count, len(keys)), device=self.device)

    def _as_env_ids(self, env_ids: Sequence[int] | slice | torch.Tensor) -> torch.Tensor:
        """Normalize environment selections to flat device-local simulator indices."""
        if isinstance(env_ids, torch.Tensor):
            return env_ids.to(device=self.device, dtype=torch.int32).reshape(-1)
        if isinstance(env_ids, slice):
            return torch.arange(self.num_envs, device=self.device, dtype=torch.int32)[env_ids]
        return torch.tensor(list(env_ids), dtype=torch.int32, device=self.device)

