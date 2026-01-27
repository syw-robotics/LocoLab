# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Script to an environment with random action agent."""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Random agent for Isaac Lab environments.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import locolab.tasks  # noqa: F401
import torch
from isaaclab_tasks.utils import parse_env_cfg


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


def main():
    """Random actions agent with Isaac Lab environment."""
    # create environment configuration
    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    if hasattr(env_cfg.curriculum, "terrain_levels"):
        env_cfg.curriculum.terrain_levels.params["log_by_terrain_type"] = False
        env_cfg.scene.terrain.max_init_terrain_level = None

    # create environment
    env = gym.make(args_cli.task, cfg=env_cfg)

    # print info (this is vectorized environment)
    print_joint_order(env)
    print(f"[INFO]: Gym observation space: {env.observation_space}")
    print(f"[INFO]: Gym action space: {env.action_space}")
    # reset environment
    env.reset()
    # simulate environment
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            # sample actions from -1 to 1
            actions = 2 * torch.rand(env.action_space.shape, device=env.unwrapped.device) - 1
            # apply actions
            env.step(actions)

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
