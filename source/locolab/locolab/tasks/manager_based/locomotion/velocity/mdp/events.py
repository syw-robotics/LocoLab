# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Locomotion events that are not provided by Isaac Lab.

* :func:`reset_root_state_uniform_by_terrain` — root reset whose pose and velocity
  ranges depend on the sub-terrain an environment sits on.
* :func:`register_virtual_obstacle_to_sensor` — bind terrain virtual obstacles to a
  volume-points sensor at startup.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Sequence, Literal

import torch

import isaaclab.utils.math as math_utils
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors.ray_caster import RayCaster, MultiMeshRayCaster

from locolab.utils.terrains.terrain_utils import (
    build_terrain_reset_range_tables,
    get_terrain_mask,
    range_dict_to_tensor,
)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

logger = logging.getLogger(__name__)

# Unknown sub-terrain names already reported. Each name set is warned once.
_UNKNOWN_TERRAIN_RESET_WARNINGS: set[tuple[str, ...]] = set()

_RangeDict = dict[str, tuple[float, float]]


# ===== root reset by terrain =====


def reset_root_state_uniform_by_terrain(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | Sequence[int] | slice | None,
    pose_range: _RangeDict,
    velocity_range: _RangeDict,
    terrain_groups: Sequence[Any] | dict[str, Any],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """Reset root state with per-sub-terrain pose and velocity ranges.

    Grouped variant of :func:`isaaclab.envs.mdp.events.reset_root_state_uniform`.
    Environments whose sub-terrain is not listed in any group use ``pose_range`` and
    ``velocity_range``. Each group may override a subset of those keys.

    Example::

        reset_base = EventTerm(
            func=mdp.reset_root_state_uniform_by_terrain,
            mode="reset",
            params={
                "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
                "velocity_range": {},
                "terrain_groups": {
                    "x_forward": {
                        "terrain_names": ["gap", "hurdle"],
                        "pose_range": {"yaw": (-0.5, 0.5)},
                    },
                },
            },
        )
    """
    asset = env.scene[asset_cfg.name]
    if env_ids is None or isinstance(env_ids, slice):
        env_ids = torch.arange(env.num_envs, device=asset.device)
    elif not isinstance(env_ids, torch.Tensor):
        env_ids = torch.as_tensor(env_ids, device=asset.device, dtype=torch.long)
    else:
        env_ids = env_ids.to(device=asset.device)
    if env_ids.numel() == 0:
        return

    root_states = asset.data.default_root_state[env_ids].clone()

    # Per-env (min, max) bounds, shape (num_reset, 6, 2). Unlisted sub-terrains keep the defaults.
    default_pose = range_dict_to_tensor(pose_range, asset.device)
    default_vel = range_dict_to_tensor(velocity_range, asset.device)
    terrain = getattr(env.scene, "terrain", None)
    type_indices = getattr(terrain, "env_terrain_indices", None) if terrain is not None else None
    terrain_cfg = getattr(terrain, "cfg", None) if terrain is not None else None
    generator = getattr(terrain_cfg, "terrain_generator", None) if terrain_cfg is not None else None
    sub_terrains = getattr(generator, "sub_terrains", None) if generator is not None else None
    if type_indices is None or not sub_terrains:
        pose_bounds = default_pose.unsqueeze(0).expand(len(env_ids), -1, -1)
        vel_bounds = default_vel.unsqueeze(0).expand(len(env_ids), -1, -1)
    else:
        terrain_names = list(sub_terrains.keys())
        pose_table, vel_table, unknown_names = build_terrain_reset_range_tables(
            terrain_names, pose_range, velocity_range, terrain_groups, asset.device
        )
        if unknown_names:
            warning_key = tuple(sorted(set(unknown_names)))
            if warning_key not in _UNKNOWN_TERRAIN_RESET_WARNINGS:
                _UNKNOWN_TERRAIN_RESET_WARNINGS.add(warning_key)
                logger.warning(
                    "reset_root_state_uniform_by_terrain ignored unknown sub-terrains: %s. Available: %s.",
                    list(warning_key),
                    terrain_names,
                )
        selected = type_indices[env_ids]
        pose_bounds = pose_table[selected]
        vel_bounds = vel_table[selected]

    pose_samples = math_utils.sample_uniform(
        pose_bounds[:, :, 0], pose_bounds[:, :, 1], (len(env_ids), 6), device=asset.device
    )
    positions = root_states[:, 0:3] + env.scene.env_origins[env_ids] + pose_samples[:, 0:3]
    orientation_delta = math_utils.quat_from_euler_xyz(pose_samples[:, 3], pose_samples[:, 4], pose_samples[:, 5])
    orientations = math_utils.quat_mul(root_states[:, 3:7], orientation_delta)

    vel_samples = math_utils.sample_uniform(
        vel_bounds[:, :, 0], vel_bounds[:, :, 1], (len(env_ids), 6), device=asset.device
    )
    velocities = root_states[:, 7:13] + vel_samples

    asset.write_root_pose_to_sim(torch.cat([positions, orientations], dim=-1), env_ids=env_ids)
    asset.write_root_velocity_to_sim(velocities, env_ids=env_ids)


# ===== virtual obstacles =====


def register_virtual_obstacle_to_sensor(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    sensor_cfgs: list[SceneEntityCfg] | SceneEntityCfg,
):
    """Connect terrain virtual obstacles to the volume points sensor.

    Environments that are not on a virtual-obstacle terrain skip the penetration query.
    Link poses are still refreshed so debug markers stay on the bodies.
    """
    if isinstance(sensor_cfgs, SceneEntityCfg):
        sensor_cfgs = [sensor_cfgs]
    virtual_obstacles: dict = env.scene.terrain.virtual_obstacles

    # None means every environment needs the query. An empty selection queries nobody.
    terrain = getattr(env.scene, "terrain", None)
    if terrain is None or getattr(terrain, "env_terrain_indices", None) is None:
        enabled_env_mask = None
    else:
        selected: list[str] = []
        query_all = False
        for virtual_obstacle in virtual_obstacles.values():
            names = virtual_obstacle.cfg.selected_terrain_names()
            if names is None:
                query_all = True
                break
            selected.extend(names)
        if query_all:
            enabled_env_mask = None
        elif not selected:
            enabled_env_mask = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        else:
            enabled_env_mask = get_terrain_mask(env, sorted(set(selected)))

    for sensor_cfg in sensor_cfgs:
        sensor = env.scene[sensor_cfg.name]
        if not hasattr(sensor, "register_virtual_obstacles"):
            raise ValueError(f"Sensor {sensor_cfg.name} does not support virtual obstacles.")
        sensor.register_virtual_obstacles(virtual_obstacles, enabled_env_mask=enabled_env_mask)


# ===== ray cast camera =====


def randomize_ray_offsets(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    offset_pose_ranges: dict[str, tuple[float, float]],
    distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
):
    """
    Randomize the ray_starts and ray_directions of the sensor to mimic the sensor installation errors.

    Args:
        - offset_pose_ranges: (dict[str, tuple[float, float]])
            where keys are ["x", "y", "z", "roll", "pitch", "yaw"]
            and values are tuples representing the range for each component.
        - distribution: (str) "uniform" or "log_uniform" or "gaussian", determines the distribution of the randomization.
    """
    if distribution != "uniform":
        raise NotImplementedError(
            f"[randomize_ray_offsets] Distribution '{distribution}' is not implemented yet. Only support 'uniform' now"
        )

    num_env_ids = env.scene.num_envs if env_ids is None else len(env_ids)
    # extract the used quantities (to enable type-hinting)
    sensor: RayCaster | MultiMeshRayCaster = env.scene[asset_cfg.name]
    ray_starts = sensor.ray_starts[env_ids]  # (num_envs, num_rays, 3)
    ray_directions = sensor.ray_directions[env_ids]  # (num_envs, num_rays, 3)
    # sample from given range
    range_list = [offset_pose_ranges.get(key, (0.0, 0.0)) for key in ["x", "y", "z", "roll", "pitch", "yaw"]]
    ranges = torch.tensor(range_list, device=ray_starts.device)  # (6, 2)
    rand_samples = (
        math_utils.sample_uniform(
            ranges[:, 0],
            ranges[:, 1],
            (num_env_ids, 6),
            device=ray_starts.device,
        )[..., None, :]
        .repeat(1, sensor.num_rays, 1)
        .flatten(0, 1)
    )
    position_samples = rand_samples[..., :3]  # (num_envs * num_rays, 3)
    rotation_samples = math_utils.quat_from_euler_xyz(
        rand_samples[..., 3],
        rand_samples[..., 4],
        rand_samples[..., 5],
    )  # (num_envs * num_rays, 4) (w, x, y, z)
    # apply the randomization
    ray_starts += position_samples.reshape(ray_starts.shape)
    ray_directions = math_utils.quat_apply(rotation_samples.reshape(*ray_directions.shape[:-1], 4), ray_directions)

    sensor.ray_starts[env_ids] = ray_starts
    sensor.ray_directions[env_ids] = ray_directions
