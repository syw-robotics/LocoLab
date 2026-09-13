# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.


from dataclasses import MISSING

from isaaclab.managers import CommandTermCfg
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.utils import configclass

import isaaclab.sim as sim_utils
from isaaclab.markers.config import FRAME_MARKER_CFG

from locolab.utils.markers import GREEN_ARROW_X_MARKER_CFG, RED_ARROW_X_MARKER_CFG

from .arm_ee_command import SampledArmEePoseCommand
from .arm_ee_traj_command import SampledArmEeTrajCommand
from .velocity_command import UniformVelocityCommand


@configclass
class UniformVelocityCommandCfg(CommandTermCfg):
    """Configuration for the uniform velocity command generator."""

    class_type: type = UniformVelocityCommand

    asset_name: str = MISSING
    """Name of the asset in the environment for which the commands are generated."""

    zero_velocity_threshold: float = 0.2
    """Velocity threshold for the command generator. Defaults to 0.2."""

    metrics_update_interval: int = 500
    """Number of control steps between metric updates. Defaults to 500."""

    heading_command: bool = False
    """Whether to use heading command or angular velocity command. Defaults to False.

    If True, the angular velocity command is computed from the heading error, where the
    target heading is sampled uniformly from provided range. Otherwise, the angular velocity
    command is sampled uniformly from provided range.
    """

    heading_control_stiffness: float = 1.0
    """Scale factor to convert the heading error to angular velocity command. Defaults to 1.0."""

    rel_heading_envs: float = 1.0
    """The sampled probability of environments where the robots follow the heading-based angular velocity command
    (the others follow the sampled angular velocity command). Defaults to 1.0.

    This parameter is only used if :attr:`heading_command` is True.
    """

    rel_only_lin_vel_x_envs: float = 0.05
    """The sampled probability of environments where only x-direction velocity command is non-zero
    (y velocity and angular velocity are set to zero). Defaults to 0.05.

    This allows creating a mix of environments where some robots only move forward/backward in x-direction
    while others have full 3-DOF velocity commands.
    """

    rel_zero_lin_vel_envs: float = 0.0
    """The sampled probability of in-place environments with zero x/y velocity.

    The yaw command is preserved so these environments can learn in-place turning. This mode,
    :attr:`rel_standing_envs`, and :attr:`rel_only_lin_vel_x_envs` are mutually exclusive. Defaults to 0.0.
    """

    rel_standing_envs: float = 0.0
    """The sampled probability of environments with a fully zero velocity command.

    This is a separate, mutually exclusive command mode. Defaults to 0.0.
    """

    @configclass
    class Ranges:
        """Uniform distribution ranges for the velocity commands."""

        lin_vel_x: tuple[float, float] = MISSING
        """Range for the linear-x velocity command (in m/s)."""

        lin_vel_y: tuple[float, float] = MISSING
        """Range for the linear-y velocity command (in m/s)."""

        ang_vel_z: tuple[float, float] = MISSING
        """Range for the angular-z velocity command (in rad/s)."""

        heading: tuple[float, float] | None = None
        """Range for the heading command (in rad). Defaults to None.

        This parameter is only used if :attr:`~UniformVelocityCommandCfg.heading_command` is True.
        """

    ranges: Ranges = MISSING
    """Distribution ranges for the velocity commands."""

    goal_vel_visualizer_cfg: VisualizationMarkersCfg = GREEN_ARROW_X_MARKER_CFG.replace(
        prim_path="/Visuals/Command/velocity_goal"
    )
    """The configuration for the goal velocity visualization marker. Defaults to GREEN_ARROW_X_MARKER_CFG."""

    current_vel_visualizer_cfg: VisualizationMarkersCfg = RED_ARROW_X_MARKER_CFG.replace(
        prim_path="/Visuals/Command/velocity_current"
    )
    """The configuration for the current velocity visualization marker. Defaults to BLUE_ARROW_X_MARKER_CFG."""

    # Set the scale of the visualization markers to (0.5, 0.5, 0.5)
    goal_vel_visualizer_cfg.markers["arrow"].scale = (0.5, 0.5, 0.5)
    current_vel_visualizer_cfg.markers["arrow"].scale = (0.5, 0.5, 0.5)

    # Marker z offset
    vel_visualizer_offset_z: float = 0.5


@configclass
class SampledArmEEPoseCommandCfg(CommandTermCfg):
    """Configuration for sampling arm end-effector pose commands from a dataset.

    Dataset poses are defined in the robot base frame. Sampled Cartesian poses are
    interpolated in the yaw-aligned command frame while the world target is reconstructed
    every step from a yaw-aligned anchor center. Policy observations receive the command
    in the current base frame, while reward terms should read the world-frame command via
    :attr:`SampledArmEePoseCommand.command_w`.
    """

    class_type: type = SampledArmEePoseCommand

    asset_name: str = MISSING
    """Name of the asset in the environment for which the commands are generated."""

    body_name: str = MISSING
    """Name of the end-effector body link in the asset."""

    pose_dataset_path: str = MISSING
    """Path to the ``.npz`` dataset of valid end-effector poses in the robot base frame.

    Supported layouts:

    * ``arm_joint_q_and_ee_pose``: shape ``(N, n_arm + 7)``, each row ``[arm_q..., x, y, z, qw, qx, qy, qz]``.
    * ``ee_pose`` (legacy): shape ``(N, 7)``, each row ``[qw, qx, qy, qz, x, y, z]``.

    Poses are consumed directly in the command frame; no conversion of the dataset is required.
    """

    anchor_z_world: float = 0.0
    """Terrain-fixed world-frame height used by the yaw-aligned anchor center.

    When the pose dataset stores full base-frame end-effector positions, set this to the
    robot's nominal standing base height. Leave :attr:`anchor_center_offset_b` at zero in
    that case to avoid double-counting shoulder offsets.
    """

    anchor_center_offset_b: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """Optional yaw-frame offset added to the yaw-aligned anchor center.

    Use only when command-frame poses are already expressed relative to a custom anchor
    point. For full base-frame dataset poses, keep this at ``(0, 0, 0)``.
    """

    make_quat_unique: bool = True
    """Whether to enforce a positive real part on sampled quaternions. Defaults to True."""

    track_orientation: bool = False
    """Whether the command tracks full pose ``(7,)`` or position only ``(3,)``.

    If ``False``, only position is sampled, interpolated, published, and logged in metrics.
    """

    metrics_update_interval: int = 500
    """Number of control steps between metric updates. Defaults to 500."""

    interpolation_time_range: tuple[float, float] = (1.5, 1.5)
    """Per-trajectory interpolation duration sampled uniformly from this range when a new target is drawn."""

    interpolation_modes: tuple[str, ...] = ("sphere", "cartesian")
    """Candidate position interpolation modes. One mode is sampled per trajectory at resample time."""

    interpolation_mode_probs: tuple[float, ...] | None = None
    """Sampling weights aligned with :attr:`interpolation_modes`.

    ``None`` samples modes uniformly. Otherwise the tuple length must match
    :attr:`interpolation_modes`, entries must be non-negative, and they are
    normalized to a probability distribution.
    """

    workspace_expand_height_range: tuple[float, float] = (0.0, 0.0)
    """Virtual base-height residual added to the yaw-aligned command, in meters.

    Sampled once per resample and applied as ``p.z += Δh`` after the pitch map.
    ``(0, 0)`` disables height expansion.
    """

    workspace_expand_pitch_range: tuple[float, float] = (0.0, 0.0)
    """Virtual base-pitch residual in radians, applied as ``p ← R_y(θ) p``.

    Expands forward/height coupling without changing command-frame y.
    ``(0, 0)`` disables pitch expansion.
    """

    sphere_center_offset_b: tuple[float, float, float] = (0.2, 0.0, 0.8)
    """Sphere interpolation center in the command frame.

    Used only by spherical position interpolation. Independent from the yaw-aligned
    anchor center used for world-frame reconstruction.
    """

    interp_path_num_points: int = 6
    """Number of samples used to visualize the interpolation path in debug mode."""

    goal_ee_visualizer_cfg: VisualizationMarkersCfg = FRAME_MARKER_CFG.replace(
        prim_path="/Visuals/Command/arm_ee_goal_pose"
    )
    """Goal EE frame marker when :attr:`track_orientation` is enabled."""

    current_ee_visualizer_cfg: VisualizationMarkersCfg = FRAME_MARKER_CFG.replace(
        prim_path="/Visuals/Command/arm_ee_current_pose"
    )
    """Current EE frame marker when :attr:`track_orientation` is enabled."""

    goal_pos_visualizer_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/Command/arm_ee_goal_pos",
        markers={
            "sphere": sim_utils.SphereCfg(
                radius=0.03,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.9, 0.2)),
            ),
        },
    )
    """Goal position sphere marker when :attr:`track_orientation` is disabled."""

    current_pos_visualizer_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/Command/arm_ee_current_pos",
        markers={
            "sphere": sim_utils.SphereCfg(
                radius=0.03,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.2, 0.2)),
            ),
        },
    )
    """Current EE position sphere marker when :attr:`track_orientation` is disabled."""

    interp_path_visualizer_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/Command/arm_ee_interp_path",
        markers={
            "point": sim_utils.SphereCfg(
                radius=0.015,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.65, 0.1)),
            ),
        },
    )
    """Position interpolation path markers shown in debug mode."""

    goal_ee_visualizer_cfg.markers["frame"].scale = (0.12, 0.12, 0.12)
    goal_ee_visualizer_cfg.markers["connecting_line"].visual_material = sim_utils.PreviewSurfaceCfg(
        diffuse_color=(0.2, 0.9, 0.2)
    )
    current_ee_visualizer_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
    current_ee_visualizer_cfg.markers["connecting_line"].visual_material = sim_utils.PreviewSurfaceCfg(
        diffuse_color=(1.0, 0.2, 0.2)
    )


@configclass
class SampledArmEETrajCommandCfg(CommandTermCfg):
    """Configuration for replaying precomputed arm EE trajectories.

    Dataset trajectories are defined in the robot base frame as ``ee_pose`` with shape
    ``(N, T, 7)`` and layout ``[x, y, z, qw, qx, qy, qz]``. Playback interpolates only
    between adjacent waypoints of the sampled trajectory. The world target is reconstructed
    every step from a yaw-aligned anchor center, matching :class:`SampledArmEEPoseCommandCfg`.
    """

    class_type: type = SampledArmEeTrajCommand

    asset_name: str = MISSING
    """Name of the asset in the environment for which the commands are generated."""

    body_name: str = MISSING
    """Name of the end-effector body link in the asset."""

    traj_dataset_path: str = MISSING
    """Path to the ``.npz`` dataset of valid end-effector trajectories in the robot base frame.

    Required key:

    * ``ee_pose``: shape ``(N, T, 7)``, each waypoint ``[x, y, z, qw, qx, qy, qz]``.

    Optional keys such as ``q`` / ``qdot`` are ignored at command time.
    """

    anchor_z_world: float = 0.0
    """Terrain-fixed world-frame height used by the yaw-aligned anchor center."""

    anchor_center_offset_b: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """Optional yaw-frame offset added to the yaw-aligned anchor center."""

    make_quat_unique: bool = True
    """Whether to enforce a positive real part on trajectory quaternions. Defaults to True."""

    track_orientation: bool = True
    """Whether the command tracks full pose ``(7,)`` or position only ``(3,)``."""

    metrics_update_interval: int = 500
    """Number of control steps between metric updates. Defaults to 500."""

    playback_time_range: tuple[float, float] = (1.5, 3.0)
    """Duration used to play one trajectory from the first waypoint to the last.

    After playback finishes, the command holds the final waypoint until the next resample.
    """

    workspace_expand_height_range: tuple[float, float] = (0.0, 0.0)
    """Virtual base-height residual added to the yaw-aligned command, in meters.

    Sampled once per trajectory and applied as ``p.z += Δh`` after the pitch map.
    ``(0, 0)`` disables height expansion.
    """

    workspace_expand_pitch_range: tuple[float, float] = (0.0, 0.0)
    """Virtual base-pitch residual in radians, applied as ``p ← R_y(θ) p``.

    Expands forward/height coupling without changing command-frame y.
    ``(0, 0)`` disables pitch expansion.
    """

    goal_ee_visualizer_cfg: VisualizationMarkersCfg = FRAME_MARKER_CFG.replace(
        prim_path="/Visuals/Command/arm_ee_goal_pose"
    )
    """Goal EE frame marker when :attr:`track_orientation` is enabled."""

    current_ee_visualizer_cfg: VisualizationMarkersCfg = FRAME_MARKER_CFG.replace(
        prim_path="/Visuals/Command/arm_ee_current_pose"
    )
    """Current EE frame marker when :attr:`track_orientation` is enabled."""

    goal_pos_visualizer_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/Command/arm_ee_goal_pos",
        markers={
            "sphere": sim_utils.SphereCfg(
                radius=0.03,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.9, 0.2)),
            ),
        },
    )
    """Goal position sphere marker when :attr:`track_orientation` is disabled."""

    current_pos_visualizer_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/Command/arm_ee_current_pos",
        markers={
            "sphere": sim_utils.SphereCfg(
                radius=0.03,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.2, 0.2)),
            ),
        },
    )
    """Current EE position sphere marker when :attr:`track_orientation` is disabled."""

    traj_path_visualizer_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/Command/arm_ee_traj_path",
        markers={
            "point": sim_utils.SphereCfg(
                radius=0.015,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.65, 0.1)),
            ),
        },
    )
    """Active trajectory waypoint markers shown in debug mode."""

    goal_ee_visualizer_cfg.markers["frame"].scale = (0.12, 0.12, 0.12)
    goal_ee_visualizer_cfg.markers["connecting_line"].visual_material = sim_utils.PreviewSurfaceCfg(
        diffuse_color=(0.2, 0.9, 0.2)
    )
    current_ee_visualizer_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
    current_ee_visualizer_cfg.markers["connecting_line"].visual_material = sim_utils.PreviewSurfaceCfg(
        diffuse_color=(1.0, 0.2, 0.2)
    )
