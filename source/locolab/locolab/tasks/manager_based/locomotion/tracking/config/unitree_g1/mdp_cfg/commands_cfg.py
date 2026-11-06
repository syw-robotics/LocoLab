# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from isaaclab.utils import configclass

import locolab.tasks.manager_based.locomotion.tracking.mdp as mdp

from . import ANCHOR_BODY_NAME, BODY_NAMES, DEFAULT_MOTION_FILE, JOINT_NAMES, VELOCITY_RANGE


@configclass
class CommandsCfg:
    """One Lafan clip shared by every environment."""

    motion = mdp.MotionCommandCfg(
        asset_name="robot",
        resampling_time_range=(1.0e9, 1.0e9),
        debug_vis=False,
        motion_file=DEFAULT_MOTION_FILE,
        anchor_body_name=ANCHOR_BODY_NAME,
        body_names=BODY_NAMES,
        joint_names=JOINT_NAMES,
        # Select reference fields and their concatenation order (one frame).
        # Available: joint_pos, joint_vel, base_height, base_lin_vel, base_ang_vel, projected_gravity.
        command_terms=("joint_pos", "joint_vel"),
        pose_range={
            "x": (-0.05, 0.05),
            "y": (-0.05, 0.05),
            "z": (-0.01, 0.01),
            "roll": (-0.1, 0.1),
            "pitch": (-0.1, 0.1),
            "yaw": (-0.2, 0.2),
        },
        velocity_range=VELOCITY_RANGE,
        joint_position_range=(-0.1, 0.1),
        sampling_mode="adaptive",
    )
