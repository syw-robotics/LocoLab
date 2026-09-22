# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Sequence

import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

PoseRange = dict[str, tuple[float, float]]
"""Axis ranges used by root-state reset, keyed by ``x/y/z/roll/pitch/yaw``."""

RANGE_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


def range_dict_to_tensor(range_dict: PoseRange, device: torch.device | str) -> torch.Tensor:
    """Convert a pose/velocity range dict to a ``(6, 2)`` tensor of ``(min, max)`` bounds."""
    return torch.tensor(
        [range_dict.get(key, (0.0, 0.0)) for key in RANGE_KEYS],
        device=device,
        dtype=torch.float32,
    )


def normalize_terrain_groups(terrain_groups: Sequence[Any] | dict[str, Any] | None) -> list[dict[str, Any]]:
    """Normalize named or unnamed terrain reset groups into a list of dicts.

    Accepted forms:

    * ``[{"terrain_names": [...], "pose_range": {...}, "velocity_range": {...}}, ...]``
    * ``{"x_forward": {"terrain_names": [...], "pose_range": {...}}, ...}``
    * a list of objects with ``terrain_names`` / ``pose_range`` / ``velocity_range`` attributes

    Later groups override earlier groups when the same sub-terrain is listed twice.
    """
    if not terrain_groups:
        return []

    if isinstance(terrain_groups, dict):
        items: Sequence[Any] = (
            [terrain_groups] if "terrain_names" in terrain_groups else list(terrain_groups.values())
        )
    else:
        items = terrain_groups

    normalized: list[dict[str, Any]] = []
    for group in items:
        if group is None:
            continue
        if isinstance(group, dict):
            names = group.get("terrain_names")
            pose_range = group.get("pose_range")
            velocity_range = group.get("velocity_range")
        else:
            names = getattr(group, "terrain_names", None)
            pose_range = getattr(group, "pose_range", None)
            velocity_range = getattr(group, "velocity_range", None)
        if not names:
            raise ValueError("Each terrain reset group must provide a non-empty 'terrain_names' list.")
        normalized.append(
            {
                "terrain_names": list(names),
                "pose_range": pose_range,
                "velocity_range": velocity_range,
            }
        )
    return normalized


def build_terrain_reset_range_tables(
    terrain_type_names: Sequence[str],
    default_pose_range: PoseRange,
    default_velocity_range: PoseRange,
    terrain_groups: Sequence[Any] | dict[str, Any] | None,
    device: torch.device | str,
) -> tuple[torch.Tensor, torch.Tensor, list[str]]:
    """Build per-sub-terrain pose/velocity range tables.

    Returns:
        pose_table: ``(num_terrain_types, 6, 2)`` bounds, one row per sub-terrain type.
        vel_table: same shape as ``pose_table``.
        unknown_names: group terrain names that are not in ``terrain_type_names``.
    """
    groups = normalize_terrain_groups(terrain_groups)
    name_to_index = {name: index for index, name in enumerate(terrain_type_names)}
    num_types = len(terrain_type_names)

    pose_table = range_dict_to_tensor(default_pose_range, device).unsqueeze(0).expand(num_types, -1, -1).clone()
    vel_table = range_dict_to_tensor(default_velocity_range, device).unsqueeze(0).expand(num_types, -1, -1).clone()
    unknown_names: list[str] = []

    for group in groups:
        pose_range = {**default_pose_range, **(group["pose_range"] or {})}
        velocity_range = {**default_velocity_range, **(group["velocity_range"] or {})}
        pose_bounds = range_dict_to_tensor(pose_range, device)
        vel_bounds = range_dict_to_tensor(velocity_range, device)
        for name in group["terrain_names"]:
            index = name_to_index.get(name)
            if index is None:
                unknown_names.append(name)
                continue
            pose_table[index] = pose_bounds
            vel_table[index] = vel_bounds

    return pose_table, vel_table, unknown_names


def get_terrain_mask(env: ManagerBasedEnv, target_terrains: list[str]) -> torch.Tensor:
    """Return a boolean mask for environments on the named sub-terrains.

    The terrain-type index lookup is cached on the terrain object.
    """
    terrain = getattr(env.scene, "terrain", None)
    if terrain is None:
        return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    env_terrain_indices = getattr(terrain, "env_terrain_indices", None)
    if env_terrain_indices is None:
        return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    cache_key = f"_cached_indices_{'_'.join(sorted(target_terrains))}"
    if not hasattr(terrain, cache_key):
        terrain_cfg = getattr(terrain, "cfg", None)
        if terrain_cfg is None:
            return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

        sub_terrains = getattr(terrain_cfg.terrain_generator, "sub_terrains", None)
        if sub_terrains is None:
            return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

        terrain_type_names = list(sub_terrains.keys())
        target_indices = [terrain_type_names.index(name) for name in target_terrains if name in terrain_type_names]
        setattr(
            terrain,
            cache_key,
            (
                torch.tensor(target_indices, device=env_terrain_indices.device, dtype=env_terrain_indices.dtype)
                if target_indices
                else torch.tensor([], device=env_terrain_indices.device, dtype=env_terrain_indices.dtype)
            ),
        )

    cached_indices = getattr(terrain, cache_key)
    if cached_indices.numel() == 0:
        return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    return torch.isin(env_terrain_indices, cached_indices)
