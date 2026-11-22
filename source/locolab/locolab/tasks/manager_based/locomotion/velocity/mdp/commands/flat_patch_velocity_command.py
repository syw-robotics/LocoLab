from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation
from isaaclab.managers import CommandTerm

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from .commands_cfg import FlatPatchVelocityCommandCfg


class FlatPatchVelocityCommand(CommandTerm):
    """Drive toward flat patches sampled on the robot's current sub-terrain."""

    cfg: FlatPatchVelocityCommandCfg

    def __init__(self, cfg: FlatPatchVelocityCommandCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)

        if not 0.0 <= cfg.rel_standing_envs <= 1.0:
            raise ValueError("rel_standing_envs must be in [0, 1].")
        if cfg.max_linear_velocity <= 0.0 or cfg.max_angular_velocity <= 0.0:
            raise ValueError("Maximum linear and angular velocities must be positive.")
        if cfg.velocity_control_stiffness <= 0.0 or cfg.heading_control_stiffness <= 0.0:
            raise ValueError("Velocity and heading control stiffness must be positive.")
        if cfg.target_distance_threshold < 0.0:
            raise ValueError("target_distance_threshold must be non-negative.")
        terrain = env.scene.terrain
        if cfg.flat_patch_key not in terrain.flat_patches:
            raise ValueError(f"Terrain has no flat patches named '{cfg.flat_patch_key}'.")
        if terrain.terrain_origins is None:
            raise ValueError("FlatPatchVelocityCommand requires terrain-based environment origins.")
        if terrain.flat_patches[cfg.flat_patch_key].shape[2] == 0:
            raise ValueError(f"Terrain flat patches named '{cfg.flat_patch_key}' are empty.")

        self.robot: Articulation = env.scene[cfg.asset_name]
        self._flat_patches = terrain.flat_patches[cfg.flat_patch_key]
        self._terrain = terrain
        self.target_pos_w = torch.zeros(self.num_envs, 3, device=self.device)
        self.vel_command_b = torch.zeros(self.num_envs, 3, device=self.device)
        self._sampled_standing = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.is_standing_env = torch.ones_like(self._sampled_standing)

    @property
    def command(self) -> torch.Tensor:
        """Desired base velocity (x, y, yaw) in the base frame."""
        return self.vel_command_b

    def _resample_command(self, env_ids: Sequence[int] | slice):
        num_envs = len(range(self.num_envs)[env_ids]) if isinstance(env_ids, slice) else len(env_ids)
        patch_ids = torch.randint(self._flat_patches.shape[2], (num_envs,), device=self.device)
        self.target_pos_w[env_ids] = self._flat_patches[
            self._terrain.terrain_levels[env_ids], self._terrain.terrain_types[env_ids], patch_ids
        ]
        if self.cfg.rel_standing_envs > 0.0:
            self._sampled_standing[env_ids] = torch.rand(num_envs, device=self.device) < self.cfg.rel_standing_envs

    def _update_command(self):
        target_delta = self.target_pos_w[:, :2] - self.robot.data.root_pos_w.torch[:, :2]
        distance = torch.linalg.vector_norm(target_delta, dim=1)
        heading_error = math_utils.wrap_to_pi(
            torch.atan2(target_delta[:, 1], target_delta[:, 0]) - self.robot.data.heading_w.torch
        )
        speed = (distance * self.cfg.velocity_control_stiffness).clamp(max=self.cfg.max_linear_velocity)
        active = (distance > self.cfg.target_distance_threshold) & ~self._sampled_standing
        speed *= active
        self.vel_command_b[:, 0] = speed * torch.cos(heading_error)
        self.vel_command_b[:, 1] = speed * torch.sin(heading_error)
        self.vel_command_b[:, 2] = (
            heading_error * self.cfg.heading_control_stiffness
        ).clamp(-self.cfg.max_angular_velocity, self.cfg.max_angular_velocity) * active
        torch.logical_not(active, out=self.is_standing_env)

    def _update_metrics(self):
        """Update the metrics for the task related metrics."""
        pass

    def compute_command_metrics(self):
        """Return a compute function for the instantaneous velocity errors."""

        def compute() -> dict[str, torch.Tensor]:
            command = self.vel_command_b
            data = self.robot.data
            return {
                "error_vel_xy": torch.norm(command[:, :2] - data.root_lin_vel_b.torch[:, :2], dim=-1).mean(),
                "error_vel_yaw": torch.abs(command[:, 2] - data.root_ang_vel_b.torch[:, 2]).mean(),
            }

        return compute
