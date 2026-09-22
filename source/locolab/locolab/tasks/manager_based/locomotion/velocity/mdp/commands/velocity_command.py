# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from __future__ import annotations

import logging
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation
from isaaclab.managers import CommandTerm
from isaaclab.markers import VisualizationMarkers

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from .commands_cfg import UniformVelocityCommandByTerrainCfg, UniformVelocityCommandCfg

# import logger
logger = logging.getLogger(__name__)


class UniformVelocityCommand(CommandTerm):
    r"""Command generator that generates a velocity command in SE(2) from uniform distribution.

    The command comprises of a linear velocity in x and y direction and an angular velocity around
    the z-axis. It is given in the robot's base frame.

    If the :attr:`cfg.heading_command` flag is set to True, the angular velocity is computed from the heading
    error similar to doing a proportional control on the heading error. The target heading is sampled uniformly
    from the provided range. Otherwise, the angular velocity is sampled uniformly from the provided range.

    Mathematically, the angular velocity is computed as follows from the heading command:

    .. math::

        \omega_z = \frac{1}{2} \text{wrap_to_pi}(\theta_{\text{target}} - \theta_{\text{current}})

    """

    cfg: UniformVelocityCommandCfg
    """The configuration of the command generator."""

    def __init__(self, cfg: UniformVelocityCommandCfg, env: ManagerBasedEnv):
        """Initialize the command generator.

        Args:
            cfg: The configuration of the command generator.
            env: The environment.

        Raises:
            ValueError: If the heading command is active but the heading range is not provided.
        """
        # initialize the base class
        super().__init__(cfg, env)

        # check configuration
        if self.cfg.heading_command and self.cfg.ranges.heading is None:
            raise ValueError(
                "The velocity command has heading commands active (heading_command=True) but the `ranges.heading`"
                " parameter is set to None."
            )
        if self.cfg.ranges.heading and not self.cfg.heading_command:
            logger.warning(
                f"The velocity command has the 'ranges.heading' attribute set to '{self.cfg.ranges.heading}'"
                " but the heading command is not active. Consider setting the flag for the heading command to True."
            )
        mode_probabilities = {
            "rel_standing_envs": self.cfg.rel_standing_envs,
            "rel_zero_lin_vel_envs": self.cfg.rel_zero_lin_vel_envs,
            "rel_only_lin_vel_x_envs": self.cfg.rel_only_lin_vel_x_envs,
        }
        for name, probability in mode_probabilities.items():
            if not 0.0 <= probability <= 1.0:
                raise ValueError(f"Expected {name} to be in [0, 1], got {probability}.")
        mode_probability = sum(mode_probabilities.values())
        self._use_command_modes = mode_probability > 0.0
        if mode_probability > 1.0:
            raise ValueError(
                "Expected the standing, in-place, and x-only probabilities to sum to at most 1, "
                f"got {mode_probability}."
            )
        if not isinstance(self.cfg.metrics_update_interval, int) or self.cfg.metrics_update_interval < 1:
            raise ValueError(
                "Expected metrics_update_interval to be a positive integer, "
                f"got {self.cfg.metrics_update_interval}."
            )

        # obtain the robot asset
        # -- robot
        self.robot: Articulation = env.scene[cfg.asset_name]

        # crete buffers to store the command
        # -- command: x vel, y vel, yaw vel, heading
        self.vel_command_b = torch.zeros(self.num_envs, 3, device=self.device)
        self.heading_target = torch.zeros(self.num_envs, device=self.device)
        self.is_heading_env = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._vel_bounds = torch.tensor(
            [self.cfg.ranges.lin_vel_x, self.cfg.ranges.lin_vel_y, self.cfg.ranges.ang_vel_z],
            device=self.device,
            dtype=torch.float32,
        )
        # commands are set to zero if the velocity xy is below the threshold
        self.zero_velocity_threshold = self.cfg.zero_velocity_threshold
        # -- metrics
        self.metrics["error_vel_xy"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["error_vel_yaw"] = torch.zeros(self.num_envs, device=self.device)
        self._metrics_update_counter = 0
        self._metrics_update_scale = (
            self.cfg.metrics_update_interval * self._env.step_dt / self.cfg.resampling_time_range[1]
        )

    def __str__(self) -> str:
        """Return a string representation of the command generator."""
        msg = "UniformVelocityCommand:\n"
        msg += f"\tCommand dimension: {tuple(self.command.shape[1:])}\n"
        msg += f"\tResampling time range: {self.cfg.resampling_time_range}\n"
        msg += f"\tHeading command: {self.cfg.heading_command}\n"
        if self.cfg.heading_command:
            msg += f"\tHeading probability: {self.cfg.rel_heading_envs}\n"
        msg += f"\tStanding probability: {self.cfg.rel_standing_envs}\n"
        msg += f"\tIn-place probability: {self.cfg.rel_zero_lin_vel_envs}\n"
        msg += f"\tOnly lin_vel_x probability: {self.cfg.rel_only_lin_vel_x_envs}\n"
        if self.zero_velocity_threshold is not None:
            msg += f"\tVelocity threshold: {self.zero_velocity_threshold}\n"
        msg += f"\tMetrics update interval: {self.cfg.metrics_update_interval} control steps\n"
        return msg

    """
    Properties
    """

    @property
    def command(self) -> torch.Tensor:
        """The desired base velocity command in the base frame. Shape is (num_envs, 3)."""
        return self.vel_command_b

    """
    Implementation specific functions.
    """

    def _update_metrics(self):
        self._metrics_update_counter = (self._metrics_update_counter + 1) % self.cfg.metrics_update_interval
        if self._metrics_update_counter != 0:
            return

        # logs data
        self.metrics["error_vel_xy"].add_(
            torch.norm(self.vel_command_b[:, :2] - self.robot.data.root_lin_vel_b[:, :2], dim=-1),
            alpha=self._metrics_update_scale,
        )
        self.metrics["error_vel_yaw"].add_(
            torch.abs(self.vel_command_b[:, 2] - self.robot.data.root_ang_vel_b[:, 2]),
            alpha=self._metrics_update_scale,
        )

    def _resample_command(self, env_ids: Sequence[int]):
        num_envs = len(env_ids)
        rand = torch.rand(num_envs, 3, device=self.device)
        commands = self._vel_bounds[:, 0] + (self._vel_bounds[:, 1] - self._vel_bounds[:, 0]) * rand
        r = torch.empty(num_envs, device=self.device)
        if self.cfg.heading_command:
            self.heading_target[env_ids] = r.uniform_(*self.cfg.ranges.heading)
            self.is_heading_env[env_ids] = r.uniform_(0.0, 1.0) <= self.cfg.rel_heading_envs
        self._assign_resampled_command(env_ids, commands, r)

    def _assign_resampled_command(self, env_ids: Sequence[int], commands: torch.Tensor, r: torch.Tensor) -> None:
        """Apply standing / in-place / x-only modes and write the command once."""
        commands[:, :2] *= (torch.norm(commands[:, :2], dim=1) > self.cfg.zero_velocity_threshold).unsqueeze(1)
        if self._use_command_modes:
            mode_sample = r.uniform_(0.0, 1.0)
            standing_end = self.cfg.rel_standing_envs
            in_place_end = standing_end + self.cfg.rel_zero_lin_vel_envs
            only_lin_vel_x_end = in_place_end + self.cfg.rel_only_lin_vel_x_envs
            is_standing = mode_sample < standing_end
            is_zero_lin_vel = (mode_sample >= standing_end) & (mode_sample < in_place_end)
            is_only_lin_vel_x = (mode_sample >= in_place_end) & (mode_sample < only_lin_vel_x_end)

            # Standing and x-only modes never need per-step heading updates.
            if self.cfg.heading_command:
                self.is_heading_env[env_ids] = self.is_heading_env[env_ids] & ~(is_standing | is_only_lin_vel_x)

            commands[is_zero_lin_vel, :2] = 0.0
            commands[is_standing] = 0.0
            commands[is_only_lin_vel_x, 1:] = 0.0
        self.vel_command_b[env_ids] = commands

    def _update_command(self):
        """Compute yaw rate from heading error for heading environments."""
        if self.cfg.heading_command:
            self._update_heading_velocity(self.cfg.ranges.ang_vel_z[0], self.cfg.ranges.ang_vel_z[1])

    def _update_heading_velocity(self, ang_min: float | torch.Tensor, ang_max: float | torch.Tensor) -> None:
        heading_error = math_utils.wrap_to_pi(self.heading_target - self.robot.data.heading_w)
        ang_vel = (self.cfg.heading_control_stiffness * heading_error).clamp(min=ang_min, max=ang_max)
        self.vel_command_b[:, 2] = torch.where(self.is_heading_env, ang_vel, self.vel_command_b[:, 2])

    def _set_debug_vis_impl(self, debug_vis: bool):
        # set visibility of markers
        # note: parent only deals with callbacks. not their visibility
        if debug_vis:
            # create markers if necessary for the first time
            if not hasattr(self, "goal_vel_visualizer"):
                # -- goal
                self.goal_vel_visualizer = VisualizationMarkers(self.cfg.goal_vel_visualizer_cfg)
                # -- current
                self.current_vel_visualizer = VisualizationMarkers(self.cfg.current_vel_visualizer_cfg)
            # set their visibility to true
            self.goal_vel_visualizer.set_visibility(True)
            self.current_vel_visualizer.set_visibility(True)
        else:
            if hasattr(self, "goal_vel_visualizer"):
                self.goal_vel_visualizer.set_visibility(False)
                self.current_vel_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        # check if robot is initialized
        # note: this is needed in-case the robot is de-initialized. we can't access the data
        if not self.robot.is_initialized:
            return
        # get marker location
        # -- base state
        base_pos_w = self.robot.data.root_pos_w.clone()
        base_pos_w[:, 2] += self.cfg.vel_visualizer_offset_z
        # -- resolve the scales and quaternions
        vel_des_arrow_scale, vel_des_arrow_quat = self._resolve_xy_velocity_to_arrow(self.command[:, :2])
        vel_arrow_scale, vel_arrow_quat = self._resolve_xy_velocity_to_arrow(self.robot.data.root_lin_vel_b[:, :2])
        # display markers
        self.goal_vel_visualizer.visualize(base_pos_w, vel_des_arrow_quat, vel_des_arrow_scale)
        self.current_vel_visualizer.visualize(base_pos_w, vel_arrow_quat, vel_arrow_scale)

    """
    Internal helpers.
    """

    def _resolve_xy_velocity_to_arrow(self, xy_velocity: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Converts the XY base velocity command to arrow direction rotation."""
        # obtain default scale of the marker
        default_scale = self.goal_vel_visualizer.cfg.markers["arrow"].scale
        # arrow-scale
        arrow_scale = torch.tensor(default_scale, device=self.device).repeat(xy_velocity.shape[0], 1)
        arrow_scale[:, 0] *= torch.linalg.norm(xy_velocity, dim=1) * 3.0
        # arrow-direction
        heading_angle = torch.atan2(xy_velocity[:, 1], xy_velocity[:, 0])
        zeros = torch.zeros_like(heading_angle)
        arrow_quat = math_utils.quat_from_euler_xyz(zeros, zeros, heading_angle)
        # convert everything back from base to world frame
        base_quat_w = self.robot.data.root_quat_w
        arrow_quat = math_utils.quat_mul(base_quat_w, arrow_quat)

        return arrow_scale, arrow_quat


_COMMAND_RANGE_KEYS = ("lin_vel_x", "lin_vel_y", "ang_vel_z", "heading")


class UniformVelocityCommandByTerrain(UniformVelocityCommand):
    """Uniform velocity command whose ranges depend on the sub-terrain.

    ``cfg.ranges`` is the default row. ``cfg.terrain_groups`` overrides a subset of
    ``lin_vel_x`` / ``lin_vel_y`` / ``ang_vel_z`` / ``heading`` for named sub-terrains.
    The per-type table is built once. Resampling and heading clip index that table.
    """

    cfg: UniformVelocityCommandByTerrainCfg

    def __init__(self, cfg: UniformVelocityCommandByTerrainCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)

        default_ranges = {
            "lin_vel_x": self.cfg.ranges.lin_vel_x,
            "lin_vel_y": self.cfg.ranges.lin_vel_y,
            "ang_vel_z": self.cfg.ranges.ang_vel_z,
            "heading": self.cfg.ranges.heading if self.cfg.ranges.heading is not None else (0.0, 0.0),
        }
        default_bounds = torch.tensor(
            [default_ranges[key] for key in _COMMAND_RANGE_KEYS], device=self.device, dtype=torch.float32
        )

        terrain = self._env.scene.terrain
        type_indices = terrain.env_terrain_indices
        sub_terrains = terrain.cfg.terrain_generator.sub_terrains
        if type_indices is None or not sub_terrains:
            raise ValueError(
                "UniformVelocityCommandByTerrain requires a terrain generator with per-environment sub-terrain indices."
            )

        terrain_names = list(sub_terrains.keys())
        name_to_index = {name: index for index, name in enumerate(terrain_names)}
        range_table = default_bounds.unsqueeze(0).expand(len(terrain_names), -1, -1).clone()

        unknown_names: list[str] = []
        for group in (self.cfg.terrain_groups or {}).values():
            if group is None:
                continue
            if not isinstance(group, dict):
                raise TypeError("Each velocity command terrain group must be a dict with a 'terrain_names' list.")
            names = group.get("terrain_names")
            if not names:
                raise ValueError("Each velocity command terrain group must provide a non-empty 'terrain_names' list.")
            overrides = group.get("ranges") or {}
            unknown_keys = set(overrides) - set(_COMMAND_RANGE_KEYS)
            if unknown_keys:
                raise ValueError(f"Unknown velocity command range keys: {sorted(unknown_keys)}.")
            merged = {**default_ranges, **overrides}
            bounds = torch.tensor(
                [merged[key] for key in _COMMAND_RANGE_KEYS], device=self.device, dtype=torch.float32
            )
            for name in names:
                index = name_to_index.get(name)
                if index is None:
                    unknown_names.append(name)
                    continue
                range_table[index] = bounds
        if unknown_names:
            logger.warning(
                "UniformVelocityCommandByTerrain ignored unknown sub-terrains: %s. Available: %s.",
                sorted(set(unknown_names)),
                terrain_names,
            )

        self._terrain_type_indices = type_indices.to(device=self.device, dtype=torch.long)
        self._command_range_table = range_table
        # Terrain type does not change when the curriculum row changes, so clip bounds are fixed per env.
        ang_bounds = range_table[self._terrain_type_indices, 2]
        self._ang_vel_min = ang_bounds[:, 0]
        self._ang_vel_max = ang_bounds[:, 1]

    def __str__(self) -> str:
        msg = super().__str__().replace("UniformVelocityCommand:", "UniformVelocityCommandByTerrain:", 1)
        group_names = list(self.cfg.terrain_groups) if self.cfg.terrain_groups else []
        msg += f"\tTerrain groups: {group_names}\n"
        return msg

    def _resample_command(self, env_ids: Sequence[int]):
        num_envs = len(env_ids)
        bounds = self._command_range_table[self._terrain_type_indices[env_ids]]
        samples = bounds[:, :, 0] + (bounds[:, :, 1] - bounds[:, :, 0]) * torch.rand(num_envs, 4, device=self.device)
        r = torch.empty(num_envs, device=self.device)
        if self.cfg.heading_command:
            self.heading_target[env_ids] = samples[:, 3]
            self.is_heading_env[env_ids] = r.uniform_(0.0, 1.0) <= self.cfg.rel_heading_envs
        self._assign_resampled_command(env_ids, samples[:, :3], r)

    def _update_command(self):
        if self.cfg.heading_command:
            self._update_heading_velocity(self._ang_vel_min, self._ang_vel_max)
