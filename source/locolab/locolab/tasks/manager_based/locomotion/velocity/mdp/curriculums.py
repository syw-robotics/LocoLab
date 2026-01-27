# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Common functions that can be used to create curriculum for the learning environment.

The functions can be passed to the :class:`isaaclab.managers.CurriculumTermCfg` object to enable
the curriculum introduced by the function.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch
from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from locolab.utils.terrains import TerrainImporter

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def terrain_levels(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    log_by_terrain_type: bool = False,
) -> torch.Tensor:
    """Curriculum based on the distance the robot walked when commanded to move at a desired velocity.

    This term is used to increase the difficulty of the terrain when the robot walks far enough and decrease the
    difficulty when the robot walks less than half of the distance required by the commanded velocity.

    .. note::
        It is only possible to use this term with the terrain type ``generator``. For further information
        on different terrain types, check the :class:`isaaclab.terrains.TerrainImporter` class.

    Returns:
        The mean terrain level for the given environment ids.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    terrain: TerrainImporter = env.scene.terrain
    command = env.command_manager.get_command("base_velocity")
    # compute the distance the robot walked
    distance = torch.norm(asset.data.root_pos_w[env_ids, :2] - env.scene.env_origins[env_ids, :2], dim=1)
    # robots that walked far enough progress to harder terrains
    move_up = distance > terrain.cfg.terrain_generator.size[0] / 2
    # robots that walked less than half of their required distance go to simpler terrains
    move_down = distance < torch.norm(command[env_ids, :2], dim=1) * env.max_episode_length_s * 0.5
    move_down *= ~move_up
    # update terrain levels
    terrain.update_env_origins(env_ids, move_up, move_down)
    # compute the mean terrain level
    terrain_level_all = torch.mean(terrain.terrain_levels.float())

    if not log_by_terrain_type:  # return mean terrain level for all terrains
        return terrain_level_all
    else:  # return mean terrain level for each terrain type
        return_dict = {"all": terrain_level_all.item()}
        env_terrain_indices = getattr(terrain, "env_terrain_indices", None)
        if env_terrain_indices is not None:
            for sub_terrain_index, sub_terrain_name in enumerate(terrain.cfg.terrain_generator.sub_terrains.keys()):
                # Find all environments that belong to this sub_terrain type
                mask = env_terrain_indices == sub_terrain_index
                if mask.any():
                    # Get terrain levels for all environments of this sub_terrain type
                    selected_levels = terrain.terrain_levels[mask]
                    return_dict[sub_terrain_name] = torch.mean(selected_levels.float()).item()
                else:
                    raise ValueError(f'No environments of terrain "{sub_terrain_name}" found.')
        else:
            raise ValueError(
                "TerrainImporter does not have `env_terrain_indices` variable. This curriculum function requires"
                " terrain type 'generator' with use_terrain_origins=True."
            )
        return return_dict


def lin_vel_cmd_levels(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    reward_term_name: str = "track_lin_vel_xy_exp",
    percentage_threshold: float = 0.8,
    max_lin_vel_x_ranges: tuple[float, float] = (-1.5, 1.5),
    max_lin_vel_y_ranges: tuple[float, float] = (-1.5, 1.5),
) -> torch.Tensor:
    """Curriculum based on the linear velocity command levels.

    This term is used to increase the difficulty of the linear velocity command levels when the robot achieves a high reward.

    Returns:
        The maximum linear velocity command level.
    """
    command_term = env.command_manager.get_term("base_velocity")
    ranges = command_term.cfg.ranges
    max_ranges_x = max_lin_vel_x_ranges
    max_ranges_y = max_lin_vel_y_ranges

    reward_term = env.reward_manager.get_term_cfg(reward_term_name)
    reward = torch.mean(env.reward_manager._episode_sums[reward_term_name][env_ids]) / env.max_episode_length_s

    if env.common_step_counter % env.max_episode_length == 0:
        if reward > reward_term.weight * percentage_threshold:
            delta_command = torch.tensor([-0.1, 0.1], device=env.device)
            ranges.lin_vel_x = torch.clamp(
                torch.tensor(ranges.lin_vel_x, device=env.device) + delta_command,
                max_ranges_x[0],
                max_ranges_x[1],
            ).tolist()
            ranges.lin_vel_y = torch.clamp(
                torch.tensor(ranges.lin_vel_y, device=env.device) + delta_command,
                max_ranges_y[0],
                max_ranges_y[1],
            ).tolist()

    return torch.tensor(ranges.lin_vel_x[1], device=env.device)


def ang_vel_cmd_levels(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    reward_term_name: str = "track_ang_vel_z_exp",
    percentage_threshold: float = 0.8,
    max_ang_vel_z_ranges: tuple[float, float] = (-2.0, 2.0),
) -> torch.Tensor:
    """Curriculum based on the angular velocity command levels.

    This term is used to increase the difficulty of the angular velocity command levels when the robot achieves a high reward.

    Returns:
        The maximum angular velocity command level.
    """
    command_term = env.command_manager.get_term("base_velocity")
    ranges = command_term.cfg.ranges
    max_ranges = max_ang_vel_z_ranges

    reward_term = env.reward_manager.get_term_cfg(reward_term_name)
    reward = torch.mean(env.reward_manager._episode_sums[reward_term_name][env_ids]) / env.max_episode_length_s

    if env.common_step_counter % env.max_episode_length == 0:
        if reward > reward_term.weight * percentage_threshold:
            delta_command = torch.tensor([-0.1, 0.1], device=env.device)
            ranges.ang_vel_z = torch.clamp(
                torch.tensor(ranges.ang_vel_z, device=env.device) + delta_command,
                max_ranges[0],
                max_ranges[1],
            ).tolist()

    return torch.tensor(ranges.ang_vel_z[1], device=env.device)
