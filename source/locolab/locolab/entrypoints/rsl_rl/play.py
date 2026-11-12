# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Script to play a checkpoint if an RL agent from RSL-RL."""

from __future__ import annotations


def main(*, add_arguments=None, configure=None):
    """Play with RSL-RL using the process command line.

    Args:
        add_arguments: Optional callback ``(parser)`` to extend the CLI before parsing.
        configure: Optional callback ``(args, env_cfg, agent_cfg)`` to mutate configs
            after CLI overrides, before seed/device setup and simulation startup.
    """
    import argparse
    import os
    import sys

    from locolab.utils.mdp_backend import add_mdp_backend_arg, apply_mdp_backend
    from locolab.utils.startup import (
        PLAY_NUM_STAGES,
        add_motion_file_arg,
        apply_motion_file_override,
        loading_screen,
        prepend_motion_file_hydra_override,
        report_run_summary,
        restore_motion_file,
    )
    from locolab.utils.viewer import add_play_viewer_args, prepare_play_viewer

    from isaaclab.app import add_launcher_args, launch_simulation  # isort: skip

    from . import cli_args

    # add argparse arguments
    parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
    parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
    parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
    parser.add_argument(
        "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
    )
    parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
    parser.add_argument("--task", type=str, default=None, help="Name of the task.")
    parser.add_argument(
        "--agent", type=str, default="rsl_rl_cfg_entry_point", help="Name of the RL agent configuration entry point."
    )
    parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
    parser.add_argument(
        "--use_pretrained_checkpoint",
        action="store_true",
        help="Use the pre-trained checkpoint from Nucleus.",
    )
    parser.add_argument("--realtime", action="store_true", default=False, help="Run in real time, if possible.")

    # for viser remote web viewer
    add_play_viewer_args(parser)
    add_motion_file_arg(parser)
    add_mdp_backend_arg(parser)
    # append RSL-RL cli arguments
    cli_args.add_rsl_rl_args(parser)
    # append simulation launcher cli args
    add_launcher_args(parser)
    # parse the arguments
    from isaaclab_tasks.utils.hydra import setup_preset_cli

    if add_arguments is not None:
        add_arguments(parser)

    args_cli, hydra_args = setup_preset_cli(parser)

    # clear out sys.argv for Hydra
    sys.argv = [sys.argv[0]] + hydra_args

    import locolab.tasks  # noqa: F401

    from isaaclab.envs.utils.video_recorder_cfg import VideoRecorderCfg
    from isaaclab.utils.assets import retrieve_file_path

    from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg
    from isaaclab_rl.utils.pretrained_checkpoint import get_published_pretrained_checkpoint

    from isaaclab_tasks.utils import get_checkpoint_path
    from isaaclab_tasks.utils.hydra import resolve_task_config

    motion_overrides = prepend_motion_file_hydra_override(args_cli.motion_file, hydra_args)
    env_cfg, agent_cfg = resolve_task_config(args_cli.task, args_cli.agent, overrides=motion_overrides)
    env_cfg = apply_mdp_backend(env_cfg, args_cli.mdp_backend)
    # grab task name for checkpoint path
    task_name = args_cli.task.split(":")[-1]
    train_task_name = task_name.replace("-Play", "")

    # override configurations with non-hydra CLI arguments
    agent_cfg: RslRlBaseRunnerCfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
    apply_motion_file_override(env_cfg, args_cli.motion_file)

    if configure is not None:
        configure(args_cli, env_cfg, agent_cfg)

    # set the environment seed
    # note: certain randomizations occur in the environment initialization so we set the seed here
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("rsl_rl", train_task_name)
        if not resume_path:
            print("[INFO] Unfortunately a pre-trained checkpoint is currently unavailable for this task.")
            return
    elif args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    restore_motion_file(env_cfg, resume_path, args_cli.motion_file, hydra_args)
    log_dir = os.path.dirname(resume_path)

    # set the log directory for the environment (works for all environment types)
    env_cfg.log_dir = log_dir

    # for viser remote web viewer
    prepare_play_viewer(args_cli, env_cfg)

    if args_cli.video:
        env_cfg.video_recorders = [
            VideoRecorderCfg(
                source="viz:kit",
                output_dir=os.path.join(log_dir, "videos", "play"),
                video_length=args_cli.video_length,
                video_interval=0,
            )
        ]
        print("[INFO] Recording a video during playback.")

    with loading_screen(args_cli, num_stages=PLAY_NUM_STAGES) as screen:
        screen.stage("Launching simulation")
        with launch_simulation(env_cfg, args_cli):
            report_run_summary(screen, args_cli, env_cfg, library="rsl_rl", action="play")
            _play(args_cli, env_cfg, agent_cfg, resume_path, screen)


def _play(args_cli, env_cfg, agent_cfg, resume_path, screen):
    """Play a checkpoint after the simulation runtime has started."""
    import os
    import time

    import torch
    from locolab.utils.mdp_backend import make_mdp_env
    from locolab.utils.viewer import open_play_viewer
    from rsl_rl.runners import DistillationRunner, OnPolicyRunner

    from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent

    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

    screen.stage("Creating environment")
    env = make_mdp_env(args_cli.task, env_cfg, args_cli.mdp_backend)

    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    screen.stage("Loading policy")
    print(f"[INFO]: Loading model checkpoint from: {resume_path}")
    # load previously trained model
    if agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    elif ":" in agent_cfg.class_name:
        from isaaclab.utils.string import string_to_callable

        runner_class = string_to_callable(agent_cfg.class_name)
        runner = runner_class(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    else:
        raise ValueError(f"Unsupported runner class: {agent_cfg.class_name}")
    runner.load(resume_path)

    # obtain the trained policy for inference
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    # export the trained policy to JIT and ONNX formats
    export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")

    runner.export_policy_to_jit(path=export_model_dir, filename="policy.pt")
    runner.export_policy_to_onnx(path=export_model_dir, filename="policy.onnx")

    from locolab.utils.deployment import export_deployment

    export_deployment(env.unwrapped, os.path.join(export_model_dir, "policy.onnx"), agent_cfg.clip_actions)

    dt = env.unwrapped.step_dt

    screen.close()
    viewer = open_play_viewer(env, args_cli)

    obs = env.get_observations()
    timestep = 0
    # simulate environment
    try:
        while env.unwrapped.sim.is_running():
            start_time = time.time()
            # run everything in inference mode
            with torch.inference_mode():
                if viewer.apply_reset(env, policy.reset):
                    obs = env.get_observations()
                if not viewer.paused:
                    # agent stepping
                    actions = policy(obs)
                    # env stepping
                    obs, _, dones, _ = env.step(actions)
                    # reset recurrent states for episodes that have terminated
                    policy.reset(dones)
            if viewer.after_step(obs):
                continue
            if args_cli.video:
                timestep += 1
                # Exit the play loop after recording one video
                if timestep == args_cli.video_length:
                    break

            # time delay for real time evaluation
            sleep_time = dt - (time.time() - start_time)
            if args_cli.realtime and not viewer.paces and sleep_time > 0:
                time.sleep(sleep_time)
    finally:
        viewer.close()
        env.close()


if __name__ == "__main__":
    main()
