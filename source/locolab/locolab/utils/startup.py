# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Loading-screen helpers for LocoLab train and play scripts."""

from __future__ import annotations

import argparse
import os
from typing import Any

from isaaclab.app.loading_screen import LoadingScreen
from isaaclab.envs import DirectMARLEnvCfg, ManagerBasedRLEnvCfg
from isaaclab.renderers.renderer_cfg import RendererCfg
from isaaclab_rl.entrypoints.common import startup_screen

__all__ = [
    "PLAY_NUM_STAGES",
    "TRAIN_NUM_STAGES",
    "add_motion_file_arg",
    "prepend_motion_file_hydra_override",
    "apply_motion_file_override",
    "loading_screen",
    "report_run_summary",
]

RUN_NUM_STAGES = 3
PLAY_NUM_STAGES = RUN_NUM_STAGES
TRAIN_NUM_STAGES = RUN_NUM_STAGES

loading_screen = startup_screen


def add_motion_file_arg(parser: argparse.ArgumentParser) -> None:
    """Register ``--motion_file`` for motion-tracking tasks."""
    parser.add_argument(
        "--motion_file",
        type=str,
        default=None,
        help="Path to a converted Lafan .npz clip (overrides commands.motion.motion_file).",
    )


def prepend_motion_file_hydra_override(motion_file: str | None, hydra_args: list[str]) -> list[str]:
    """Apply ``--motion_file`` before env ``__post_init__`` runs inside ``resolve_task_config``."""
    if motion_file is None:
        return hydra_args
    path = os.path.abspath(os.path.expanduser(motion_file))
    return [*hydra_args, f"commands.motion.motion_file={path}"]


def apply_motion_file_override(env_cfg: Any, motion_file: str | None) -> None:
    """Point ``commands.motion`` at another clip when ``--motion_file`` is set."""
    if motion_file is None:
        return
    motion = getattr(getattr(env_cfg, "commands", None), "motion", None)
    if motion is None:
        raise ValueError("This task has no commands.motion configuration; --motion_file is not supported.")
    path = os.path.abspath(os.path.expanduser(motion_file))
    motion.motion_file = path
    print(f"[INFO] Motion clip: {path}")


_PHYSICS_ALIASES = {"PhysxCfg": "isaacsim_physx", "OvPhysxCfg": "ovphysx"}
_RENDERER_ALIASES = {"isaac_rtx": "isaacsim_rtx", "newton_warp": "newton_renderer"}


def report_run_summary(
    screen: LoadingScreen,
    args_cli: argparse.Namespace,
    env_cfg: Any,
    *,
    library: str,
    action: str,
) -> None:
    """Print the boxed run summary (``Loco Lab · {action}``) beside the startup greeting."""
    title = f"Loco Lab · {action}"
    fields = _summary_fields(args_cli, env_cfg, library)
    screen.summary(title, fields)


def _summary_fields(args_cli: argparse.Namespace, env_cfg: Any, library: str) -> dict[str, str]:
    renderer = _first_renderer_name(env_cfg)
    renderer_label = "n/a (no camera sensors)" if renderer is None else renderer
    visualizers = [cfg.backend.visualizer_type for cfg in env_cfg.sim.visualizer_cfgs]
    num_envs = getattr(args_cli, "num_envs", None) or env_cfg.scene.num_envs

    return {
        "Task": args_cli.task,
        "Workflow": _workflow_name(env_cfg),
        "RL library": library,
        "Physics": _physics_backend_name(env_cfg.sim.physics),
        "Renderer": renderer_label,
        "Visualizer": ", ".join(visualizers) or "headless",
        "Device": env_cfg.sim.device,
        "Environments": str(num_envs),
    }


def _workflow_name(env_cfg: Any) -> str:
    if isinstance(env_cfg, ManagerBasedRLEnvCfg):
        return "manager-based"
    if isinstance(env_cfg, DirectMARLEnvCfg):
        return "direct (multi-agent)"
    return "direct"


def _physics_backend_name(physics_cfg: Any) -> str:
    class_name = type(physics_cfg).__name__
    if class_name in _PHYSICS_ALIASES:
        return _PHYSICS_ALIASES[class_name]

    backend = class_name.removesuffix("Cfg").lower()
    solver_cfg = getattr(physics_cfg, "solver_cfg", None)
    if solver_cfg is None:
        return backend

    solver_name = type(solver_cfg).__name__.removesuffix("SolverCfg").lower()
    return f"{backend}_{solver_name}"


def _first_renderer_name(env_cfg: Any) -> str | None:
    for container in (env_cfg, getattr(env_cfg, "scene", None)):
        if container is None:
            continue
        found = _renderer_in_container(container)
        if found is not None:
            return found
    return None


def _renderer_in_container(container: Any) -> str | None:
    for value in vars(container).values():
        if "renderer_cfg" not in getattr(type(value), "__dataclass_fields__", {}):
            continue
        renderer_cfg = value.renderer_cfg
        if not isinstance(renderer_cfg, RendererCfg):
            continue
        renderer_type = renderer_cfg.renderer_type
        return _RENDERER_ALIASES.get(renderer_type, renderer_type)
    return None
