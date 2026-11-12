# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from collections.abc import Sequence
from dataclasses import MISSING
from typing import Literal

from isaaclab.managers import CommandTermCfg
from isaaclab.utils import configclass

from .motion_command import MotionCommand


@configclass
class MotionCommandCfg(CommandTermCfg):
    """Configuration for a motion-library tracking command."""

    class_type: type = MotionCommand

    ghost_max_envs: int = 10
    """Maximum visible reference robots per view; bounds debug rendering cost."""

    asset_name: str = MISSING
    """Robot scene entity that follows the motion."""

    motion_file: str = MISSING
    """Path to one converted ``.npz`` clip or a directory of clips (recursive)."""

    anchor_body_name: str = MISSING
    """Tracking frame for observations, rewards and termination; G1 supports pelvis or torso_link."""

    body_names: Sequence[str] = MISSING
    """Bodies tracked by the rewards, in the order stored by the command."""

    joint_names: Sequence[str] = MISSING
    """Joints in the same order as the motion ``qpos`` hinge columns."""

    command_terms: tuple[str, ...] = ("joint_pos", "joint_vel")
    """Ordered reference fields: joint_pos, joint_vel, base_height, base_lin_vel,
    base_ang_vel, projected_gravity. Base is the motion root, independent of anchor;
    velocities and unit gravity are expressed in its reference frame.
    """

    command_frame_offsets: tuple[int, ...] = (0,)
    """Reference frame offsets in control steps, concatenated frame first, then term.

    Negative offsets read past reference frames; positive offsets read future ones.
    Each index is clamped to the current clip's first/last frame, without wrapping.
    """

    pose_range: dict[str, tuple[float, float]] = {}
    """Uniform root pose noise applied when a reference state is sampled."""

    velocity_range: dict[str, tuple[float, float]] = {}
    """Uniform root velocity noise applied when a reference state is sampled."""

    joint_position_range: tuple[float, float] = (-0.1, 0.1)
    """Uniform joint-position noise applied when a reference state is sampled."""

    metrics_update_interval: int = 200
    """Number of control steps between diagnostic metric updates, like velocity."""

    adaptive_kernel_size: int = 1
    adaptive_lambda: float = 0.8
    adaptive_uniform_ratio: float = 0.1
    adaptive_alpha: float = 0.001
    sampling_mode: Literal["adaptive", "uniform", "start"] = "adaptive"
    """How to pick the next reference frame. ``start`` cycles through clips at their first frame."""
