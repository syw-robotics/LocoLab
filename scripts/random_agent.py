# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Script to an environment with random action agent."""

import argparse
import sys

from locolab.utils.viewer import add_play_viewer_args, open_play_viewer, prepare_play_viewer

from isaaclab.app import add_launcher_args, launch_simulation

# add argparse arguments
parser = argparse.ArgumentParser(description="Random agent for Isaac Lab environments.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--realtime", action="store_true", default=False, help="Run in real time, if possible.")
add_play_viewer_args(parser)
# append simulation launcher cli args
add_launcher_args(parser)
parser.set_defaults(device=None)
from isaaclab_tasks.utils.hydra import resolve_task_config, setup_preset_cli

args_cli, hydra_args = setup_preset_cli(parser)

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

"""Rest everything follows."""

import gymnasium as gym
import locolab.tasks  # noqa: F401
import time

import torch


def print_joint_order(env):
    """Print the joint order in observation and action spaces."""
    # Get the scene and robot asset
    scene = env.unwrapped.scene
    robot = scene["robot"]

    # Get joint names from the robot
    joint_names = robot.joint_names

    # Try to extract joint names from observation and action configs
    env_cfg = env.unwrapped.cfg

    print("\n" + "=" * 80)
    print("JOINT ORDER INFORMATION")
    print("=" * 80)

    # Print all joint names
    print(f"\n[Joint Names] Total: {len(joint_names)} joints")
    for idx, name in enumerate(joint_names):
        print(f"  {idx:2d}: {name}")

    # Print observation joint_pos order
    if hasattr(env_cfg, "observations") and hasattr(env_cfg.observations, "policy"):
        policy_obs = env_cfg.observations.policy
        if hasattr(policy_obs, "joint_pos"):
            print("\n[Observation - joint_pos] Indices and names:")
            joint_pos_cfg = policy_obs.joint_pos
            if hasattr(joint_pos_cfg, "params"):
                scene_cfg = joint_pos_cfg.params.get("asset_cfg")
                if scene_cfg and hasattr(scene_cfg, "joint_names"):
                    for idx, name in enumerate(scene_cfg.joint_names):
                        print(f"  {idx:2d}: {name}")
            else:
                print("  [All movable joints in order]")
                for idx, name in enumerate(joint_names):
                    print(f"  {idx:2d}: {name}")

    # Print action order
    if hasattr(env_cfg, "actions") and hasattr(env_cfg.actions, "joint_pos"):
        print("\n[Action - joint_pos] Indices and names:")
        action_cfg = env_cfg.actions.joint_pos
        if hasattr(action_cfg, "joint_names"):
            for idx, name in enumerate(action_cfg.joint_names):
                print(f"  {idx:2d}: {name}")
        else:
            print("  [All movable joints in order]")
            for idx, name in enumerate(joint_names):
                print(f"  {idx:2d}: {name}")

    print("\n" + "=" * 80)


def _noop_policy_reset(_dones) -> None:
    """Viser reset button hook; random agent has no policy state to clear."""


def main():
    """Random actions agent with Isaac Lab environment."""
    env_cfg, _ = resolve_task_config(args_cli.task, "", overrides=hydra_args)
    if args_cli.num_envs is not None:
        env_cfg.scene.num_envs = args_cli.num_envs
    if args_cli.disable_fabric:
        env_cfg.sim.use_fabric = False
    if args_cli.device is not None:
        env_cfg.sim.device = args_cli.device
    if hasattr(env_cfg.curriculum, "terrain_levels"):
        env_cfg.curriculum.terrain_levels.params["log_by_terrain_type"] = False
        env_cfg.scene.terrain.max_init_terrain_level = None
        env_cfg.scene.terrain.debug_vis = True
        num_terrains = len(env_cfg.scene.terrain.terrain_generator.sub_terrains)
        env_cfg.scene.terrain.terrain_generator.num_cols = num_terrains
        equal_proportion = 1.0 / num_terrains
        for terrain_cfg in env_cfg.scene.terrain.terrain_generator.sub_terrains.values():
            terrain_cfg.proportion = equal_proportion

    prepare_play_viewer(args_cli, env_cfg)

    with launch_simulation(env_cfg, args_cli):
        env = gym.make(args_cli.task, cfg=env_cfg)
        viewer = open_play_viewer(env, args_cli)

        print_joint_order(env)
        print(f"[INFO]: Gym observation space: {env.observation_space}")
        print(f"[INFO]: Gym action space: {env.action_space}")

        try:
            env.reset()
            dt = env.unwrapped.step_dt
            while env.unwrapped.sim.is_running():
                start_time = time.time()
                with torch.inference_mode():
                    viewer.apply_reset(env, _noop_policy_reset)
                    if not viewer.paused:
                        actions = 2 * torch.rand(env.action_space.shape, device=env.unwrapped.device) - 1
                        env.step(actions)
                if viewer.after_step():
                    continue
                if args_cli.realtime and not viewer.paces:
                    sleep_time = dt - (time.time() - start_time)
                    if sleep_time > 0:
                        time.sleep(sleep_time)
        finally:
            viewer.close()
            env.close()


if __name__ == "__main__":
    # run the main function
    main()
