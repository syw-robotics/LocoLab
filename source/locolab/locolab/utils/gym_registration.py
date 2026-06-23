# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from __future__ import annotations

from collections.abc import Mapping

import gymnasium as gym


MANAGER_BASED_RL_ENV_ENTRY_POINT = "isaaclab.envs:ManagerBasedRLEnv"


def register_manager_based_rl_env(
    task_id: str,
    env_cfg_entry_point: str | None = None,
    agent_cfg_entry_points: Mapping[str, str] | None = None,
    *,
    env_cfg_module: str | None = None,
    env_cfg_name: str | None = None,
    register_play: bool = True,
    play_env_cfg_name: str | None = None,
) -> None:
    """Register a manager-based RL Gym environment.

    When ``env_cfg_module`` and ``env_cfg_name`` are provided, the ``-Play`` task is registered automatically
    with the ``{env_cfg_name}_PLAY`` environment config.
    """
    if env_cfg_entry_point is None:
        if env_cfg_module is None or env_cfg_name is None:
            raise ValueError("Either env_cfg_entry_point or both env_cfg_module and env_cfg_name must be provided.")
        env_cfg_entry_point = f"{env_cfg_module}:{env_cfg_name}"

    _register_manager_based_rl_env(task_id, env_cfg_entry_point, agent_cfg_entry_points)

    if register_play and env_cfg_module is not None and env_cfg_name is not None:
        _register_manager_based_rl_env(
            f"{task_id}-Play",
            f"{env_cfg_module}:{play_env_cfg_name or f'{env_cfg_name}_PLAY'}",
            agent_cfg_entry_points,
        )


def _register_manager_based_rl_env(
    task_id: str,
    env_cfg_entry_point: str,
    agent_cfg_entry_points: Mapping[str, str] | None,
) -> None:
    kwargs = _build_registration_kwargs(env_cfg_entry_point, agent_cfg_entry_points)
    gym.register(
        id=task_id,
        entry_point=MANAGER_BASED_RL_ENV_ENTRY_POINT,
        disable_env_checker=True,
        kwargs=kwargs,
    )


def _build_registration_kwargs(
    env_cfg_entry_point: str,
    agent_cfg_entry_points: Mapping[str, str] | None = None,
) -> dict[str, str]:
    kwargs = {"env_cfg_entry_point": env_cfg_entry_point}
    if agent_cfg_entry_points is not None:
        kwargs.update(agent_cfg_entry_points)
    return kwargs
