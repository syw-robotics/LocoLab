# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Select the environment runtime without changing task registrations."""

import argparse


def add_mdp_backend_arg(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--mdp_backend",
        choices=("torch", "warp"),
        default="torch",
        help="Environment runtime: torch (default), or Warp for supported built-in and external tasks.",
    )


def apply_mdp_backend(env_cfg, backend: str):
    """Return the original Torch config or prepare an isolated copy for Warp."""
    if backend == "torch":
        return env_cfg
    if backend != "warp":
        raise ValueError(f"Unknown MDP backend: {backend!r}")

    import copy

    from locolab.experimental_warp_mdp import prepare_cfg

    env_cfg = copy.deepcopy(env_cfg)
    prepare_cfg(env_cfg)
    return env_cfg


def make_mdp_env(task: str, env_cfg, backend: str):
    """Construct the selected environment after simulation launch."""
    if backend == "torch":
        import gymnasium as gym

        return gym.make(task, cfg=env_cfg)
    if backend != "warp":
        raise ValueError(f"Unknown MDP backend: {backend!r}")
    if getattr(env_cfg, "video_recorders", None):
        raise ValueError(
            "Video recording requires --mdp_backend torch; the experimental Warp runtime does not support it."
        )
    entry_point = getattr(env_cfg, "warp_env_entry_point", None)
    if entry_point is not None:
        from gymnasium.envs.registration import load_env_creator

        return load_env_creator(entry_point)(cfg=env_cfg)
    from locolab.experimental_warp_mdp.env import LocoLabWarpEnv

    return LocoLabWarpEnv(cfg=env_cfg)


def wrap_z_rl_env(env, clip_actions):
    """Keep Warp observation-history metadata visible to Z-RL's symmetry and time slicing."""
    wrap = getattr(env.unwrapped.cfg, "wrap_z_rl_env", None)
    if wrap is not None:
        return wrap(env, clip_actions)
    if getattr(env.unwrapped.cfg, "_locolab_warp", False):
        from locolab.experimental_warp_mdp.z_rl import ZRlWarpVecEnvWrapper

        wrapper = ZRlWarpVecEnvWrapper(env, clip_actions=clip_actions)
    else:
        from z_rl.adaptor.isaaclab import ZRlVecEnvWrapper

        wrapper = ZRlVecEnvWrapper(env, clip_actions=clip_actions)
    return wrapper
