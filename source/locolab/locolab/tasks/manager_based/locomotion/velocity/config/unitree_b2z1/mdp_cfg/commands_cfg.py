# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

import math

from isaaclab.utils import configclass

import locolab.tasks.manager_based.locomotion.velocity.mdp as mdp

from . import ARM_EE_LINK_NAME, DEFAULT_ARM_EE_POSE_DATASET, DEFAULT_ARM_EE_TRAJ_DATASET, NOMINAL_BASE_HEIGHT_Z


@configclass
class BaseVelocityCommandsCfg:
    """Base velocity command shared by B2Z1 quad-manipulation tasks."""

    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(8.0, 12.0),
        rel_heading_envs=1.0,
        rel_only_lin_vel_x_envs=0.1,
        rel_standing_envs=0.1,
        rel_zero_lin_vel_envs=0.1,
        zero_velocity_threshold=0.2,
        heading_command=True,
        heading_control_stiffness=0.8,
        debug_vis=False,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-1.0, 1.0),
            lin_vel_y=(-0.8, 0.8),
            ang_vel_z=(-1.5, 1.5),
            heading=(-math.pi, math.pi),
        ),
        vel_visualizer_offset_z=-0.4,
    )

    def __post_init__(self):
        marker_scale = (0.35, 0.35, 0.35)
        self.base_velocity.goal_vel_visualizer_cfg.markers["arrow"].scale = marker_scale
        self.base_velocity.current_vel_visualizer_cfg.markers["arrow"].scale = marker_scale


@configclass
class VelocityEEPositionCmdCfg(BaseVelocityCommandsCfg):
    """Velocity + arm EE position command (3D)."""

    ee_position = mdp.SampledArmEEPoseCommandCfg(
        asset_name="robot",
        body_name=ARM_EE_LINK_NAME,
        pose_dataset_path=DEFAULT_ARM_EE_POSE_DATASET,
        resampling_time_range=(5.0, 10.0),
        interpolation_time_range=(2.0, 4.0),
        anchor_z_world=NOMINAL_BASE_HEIGHT_Z,
        anchor_center_offset_b=(0.0, 0.0, 0.0),
        interpolation_modes=("cartesian", "sphere"),
        interpolation_mode_probs=(0.3, 0.7),
        workspace_expand_height_range=(-0.08, 0.05),
        workspace_expand_pitch_range=(-0.15, 0.15),
        track_orientation=False,
        debug_vis=False,
    )



@configclass
class VelocityEETrajPositionCmdCfg(BaseVelocityCommandsCfg):
    """Velocity + replayed arm EE trajectory, published as a 3D position command."""

    ee_position = mdp.SampledArmEETrajCommandCfg(
        asset_name="robot",
        body_name=ARM_EE_LINK_NAME,
        traj_dataset_path=DEFAULT_ARM_EE_TRAJ_DATASET,
        resampling_time_range=(5.0, 10.0),
        playback_time_range=(2.0, 4.0),
        anchor_z_world=NOMINAL_BASE_HEIGHT_Z,
        anchor_center_offset_b=(0.0, 0.0, 0.0),
        workspace_expand_height_range=(-0.05, 0.08),
        workspace_expand_pitch_range=(-0.15, 0.15),
        track_orientation=False,
        debug_vis=False,
    )