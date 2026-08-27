# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to play a checkpoint if an RL agent from Z-RL."""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys

from isaaclab.app import AppLauncher

# local imports
import cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with Z-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--agent", type=str, default="z_rl_cfg_entry_point", help="Name of the RL agent configuration entry point."
)
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--export_jit", action="store_true", default=False, help="Export policy as a JIT model.")
parser.add_argument("--export_onnx", action="store_true", default=False, help="Export policy as an ONNX model.")
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    help="Use the pre-trained checkpoint from Nucleus.",
)
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")
# append Z-RL cli arguments
cli_args.add_z_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()
if args_cli.task is not None and not args_cli.task.endswith("-Play"):
    args_cli.task = f"{args_cli.task}-Play"
    print(f"[INFO] Using play task: {args_cli.task}")
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


"""Rest everything follows."""

import os
import time

import gymnasium as gym
import torch
from z_rl.runners import DistillationRunner, OnPolicyRunner

from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.dict import print_dict

from z_rl.adaptor.isaaclab import ZRlBaseRunnerCfg, ZRlVecEnvWrapper
from isaaclab_rl.utils.pretrained_checkpoint import get_published_pretrained_checkpoint

import locolab.tasks  # noqa: F401
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg, agent_cfg: ZRlBaseRunnerCfg):
    """Play with Z-RL agent."""
    # grab task name for checkpoint path
    task_name = args_cli.task.split(":")[-1]
    train_task_name = task_name.replace("-Play", "")

    # override configurations with non-hydra CLI arguments
    agent_cfg: ZRlBaseRunnerCfg = cli_args.update_z_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else 10

    # set the environment seed
    # note: certain randomizations occur in the environment initialization so we set the seed here
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    if hasattr(env_cfg.curriculum, "terrain_levels"):
        env_cfg.curriculum.terrain_levels.params["log_by_terrain_type"] = False
        #  env_cfg.scene.terrain.terrain_generator.num_cols = 20
        num_terrains = len(env_cfg.scene.terrain.terrain_generator.sub_terrains)
        env_cfg.scene.terrain.terrain_generator.num_cols = num_terrains
        equal_proportion = 1.0 / num_terrains
        for terrain_cfg in env_cfg.scene.terrain.terrain_generator.sub_terrains.values():
            terrain_cfg.proportion = equal_proportion

    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "z_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    load_cfg = None
    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("z_rl", train_task_name)
        if not resume_path:
            print("[INFO] Unfortunately a pre-trained checkpoint is currently unavailable for this task.")
            return
    elif args_cli.load_run:
        checkpoint = args_cli.checkpoint if args_cli.checkpoint is not None else "model_.*.pt"
        if os.path.isabs(args_cli.load_run) or os.sep in args_cli.load_run:
            resume_path = get_checkpoint_path(os.path.dirname(args_cli.load_run), os.path.basename(args_cli.load_run), checkpoint)
        else:
            resume_path = get_checkpoint_path(log_root_path, args_cli.load_run, checkpoint)
    elif args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        try:
            resume_path = get_checkpoint_path(log_root_path, ".*", "model_.*.pt")
        except ValueError:
            load_cfg = getattr(agent_cfg, "load_cfg", None)
            if os.path.isabs(agent_cfg.load_run) or os.sep in agent_cfg.load_run:
                resume_path = get_checkpoint_path(
                    os.path.dirname(agent_cfg.load_run), os.path.basename(agent_cfg.load_run), agent_cfg.load_checkpoint
                )
            else:
                resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    log_dir = os.path.dirname(resume_path)

    # set the log directory for the environment (works for all environment types)
    env_cfg.log_dir = log_dir

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap around environment for z-rl
    env = ZRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    print(f"[INFO]: Loading model checkpoint from: {resume_path}")
    # load previously trained model
    if agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    else:
        raise ValueError(f"Unsupported runner class: {agent_cfg.class_name}")
    runner.load(resume_path, load_cfg=load_cfg)

    # obtain the trained policy for inference
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    # export the trained policy only when requested
    if args_cli.export_jit:
        export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")
        runner.export_policy_to_jit(path=export_model_dir, filename="policy.pt")
    if args_cli.export_onnx:
        export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")
        default_device = torch.get_default_device()
        try:
            torch.set_default_device("cpu")
            runner.export_policy_to_onnx(path=export_model_dir, filename="policy.onnx")
        finally:
            torch.set_default_device(default_device)

    dt = env.unwrapped.step_dt

    # reset environment
    obs = env.get_observations()
    timestep = 0
    # simulate environment
    while simulation_app.is_running():
        start_time = time.time()
        # run everything in inference mode
        with torch.inference_mode():
            # agent stepping
            actions = policy(obs)
            # env stepping
            obs, _, dones, _ = env.step(actions)
            # reset recurrent states for episodes that have terminated
            policy.reset(dones)

        # -------------------- Print Data for Debugging --------------------
        # example:
        #  robot = env.unwrapped.scene["robot"]
        #  torso_body_id = robot.body_names.index("torso_link")
        #  base_body_id = robot.body_names.index("base_link")
        #  pelvis_body_id = robot.body_names.index("pelvis")
        #  print("base_link height (env 0-10):", robot.data.body_pos_w[:11, base_body_id, 2])
        #  print("torso_link height (env 0-10):", robot.data.body_pos_w[:11, torso_body_id, 2])
        #  print("pelvis height (env 0-10):", robot.data.body_pos_w[:11, pelvis_body_id, 2])
        # -------------------- Print Data for Debugging --------------------

        if args_cli.video:
            timestep += 1
            # Exit the play loop after recording one video
            if timestep == args_cli.video_length:
                break

        # time delay for real-time evaluation
        sleep_time = dt - (time.time() - start_time)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
