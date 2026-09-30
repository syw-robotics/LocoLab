from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.envs.mdp.commands.pose_2d_command import UniformPose2dCommand
from isaaclab.utils.math import wrap_to_pi

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from .commands_cfg import TerrainBasedPose2dCommandCfg


class TerrainBasedPose2dCommand(UniformPose2dCommand):
    """Sample 2D pose targets from flat patches on the current sub-terrain."""

    cfg: TerrainBasedPose2dCommandCfg

    def __init__(self, cfg: TerrainBasedPose2dCommandCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)

        self.terrain = env.scene.terrain
        if cfg.flat_patch_key not in self.terrain.flat_patches:
            raise ValueError(f"Terrain has no flat patches named '{cfg.flat_patch_key}'.")
        if self.terrain.terrain_origins is None:
            raise ValueError("TerrainBasedPose2dCommand requires terrain-based environment origins.")
        self.valid_targets = self.terrain.flat_patches[cfg.flat_patch_key]
        if self.valid_targets.shape[2] == 0:
            raise ValueError(f"Terrain flat patches named '{cfg.flat_patch_key}' are empty.")

    def _resample_command(self, env_ids: Sequence[int]):
        patch_ids = torch.randint(self.valid_targets.shape[2], (len(env_ids),), device=self.device)
        self.pos_command_w[env_ids] = self.valid_targets[
            self.terrain.terrain_levels[env_ids], self.terrain.terrain_types[env_ids], patch_ids
        ]
        self.pos_command_w[env_ids, 2] += self.robot.data.default_root_state[env_ids, 2]

        if self.cfg.simple_heading:
            target_delta = self.pos_command_w[env_ids, :2] - self.robot.data.root_pos_w[env_ids, :2]
            target_heading = torch.atan2(target_delta[:, 1], target_delta[:, 0])
            flipped_heading = wrap_to_pi(target_heading + torch.pi)
            current_heading = self.robot.data.heading_w[env_ids]
            self.heading_command_w[env_ids] = torch.where(
                wrap_to_pi(target_heading - current_heading).abs()
                < wrap_to_pi(flipped_heading - current_heading).abs(),
                target_heading,
                flipped_heading,
            )
        else:
            self.heading_command_w[env_ids] = torch.empty(len(env_ids), device=self.device).uniform_(
                *self.cfg.ranges.heading
            )
