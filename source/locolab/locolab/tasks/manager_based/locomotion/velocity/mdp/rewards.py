# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import ManagerTermBase, RewardTermCfg, SceneEntityCfg
from isaaclab.utils.math import quat_apply, quat_apply_inverse, yaw_quat

if TYPE_CHECKING:
    from isaaclab.assets import Articulation, RigidObject
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.sensors import ContactSensor, RayCaster


def track_lin_vel_xy_exp(
    env: ManagerBasedRLEnv, std: float, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward tracking of linear velocity commands (xy axes) using exponential kernel."""
    asset: RigidObject = env.scene[asset_cfg.name]
    lin_vel_error = torch.sum(
        torch.square(env.command_manager.get_command(command_name)[:, :2] - asset.data.root_lin_vel_b[:, :2]),
        dim=1,
    )
    reward = torch.exp(-lin_vel_error / std**2)
    #  reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 1.0)  # don't penalize when upside down
    return reward


def track_ang_vel_z_exp(
    env: ManagerBasedRLEnv, std: float, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward tracking of angular velocity commands (yaw) using exponential kernel."""
    asset: RigidObject = env.scene[asset_cfg.name]
    ang_vel_error = torch.square(env.command_manager.get_command(command_name)[:, 2] - asset.data.root_ang_vel_b[:, 2])
    reward = torch.exp(-ang_vel_error / std**2)
    #  reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 1.0)  # don't penalize when upside down
    return reward


def track_lin_vel_xy_yaw_frame_exp(
    env, std: float, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward tracking of linear velocity commands (xy axes) in the gravity aligned
    robot frame using an exponential kernel.
    """
    # extract the used quantities (to enable type-hinting)
    asset = env.scene[asset_cfg.name]
    vel_yaw = quat_apply_inverse(yaw_quat(asset.data.root_quat_w), asset.data.root_lin_vel_w[:, :3])
    lin_vel_error = torch.sum(
        torch.square(env.command_manager.get_command(command_name)[:, :2] - vel_yaw[:, :2]), dim=1
    )
    return torch.exp(-lin_vel_error / std**2)


def track_ang_vel_z_world_exp(
    env, command_name: str, std: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward tracking of angular velocity commands (yaw) in world frame using exponential kernel."""
    # extract the used quantities (to enable type-hinting)
    asset = env.scene[asset_cfg.name]
    ang_vel_error = torch.square(env.command_manager.get_command(command_name)[:, 2] - asset.data.root_ang_vel_w[:, 2])
    return torch.exp(-ang_vel_error / std**2)


def lin_vel_z_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize z-axis base linear velocity using L2 squared kernel."""
    asset: RigidObject = env.scene[asset_cfg.name]
    return torch.square(asset.data.root_lin_vel_b[:, 2])


def ang_vel_xy_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize xy-axis base angular velocity using L2 squared kernel."""
    asset: RigidObject = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.root_ang_vel_b[:, :2]), dim=1)


def lin_vel_z_body_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize z-axis selected body linear velocity using L2 squared kernel."""
    asset: RigidObject = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.body_lin_vel_w[:, asset_cfg.body_ids, 2]), dim=1)


def ang_vel_xy_body_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize xy-axis selected body angular velocity using L2 squared kernel."""
    asset: RigidObject = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.body_ang_vel_w[:, asset_cfg.body_ids, :2]), dim=(1, 2))


def flat_orientation_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize non-flat base orientation using L2 squared kernel by penalizing projected gravity xy-components."""
    asset: RigidObject = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.projected_gravity_b[:, :2]), dim=1)


def joint_torques_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize joint torques applied on the articulation using L2 squared kernel."""
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.applied_torque[:, asset_cfg.joint_ids]), dim=1)


def joint_vel_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize joint velocities on the articulation using L2 squared kernel."""
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.joint_vel[:, asset_cfg.joint_ids]), dim=1)


def joint_acc_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize joint accelerations on the articulation using L2 squared kernel."""
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.sum(torch.square(asset.data.joint_acc[:, asset_cfg.joint_ids]), dim=1)


def base_height_l2(
    env: ManagerBasedRLEnv,
    target_height: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    sensor_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Penalize asset height from its target using L2 squared kernel.

    Note:
        For flat terrain, target height is in the world frame. For rough terrain,
        sensor readings can adjust the target height to account for the terrain.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    if sensor_cfg is not None:
        sensor: RayCaster = env.scene[sensor_cfg.name]
        # Adjust the target height using the sensor data
        ray_hits = sensor.data.ray_hits_w[..., 2]
        # if ray hits are invalid, clip them to a reasonable range
        if torch.isnan(ray_hits).any() or torch.isinf(ray_hits).any() or torch.max(torch.abs(ray_hits)) > 1e3:
            print(
                "\033[91m Ray hits are nan or inf or too large, using default target height \033[0m"
            )  # print red for warning
            ray_hits = torch.clip(ray_hits, min=-10.0, max=10.0)
        adjusted_target_height = target_height + torch.mean(ray_hits, dim=1)
    else:
        # Use the provided target height directly for flat terrain
        adjusted_target_height = target_height
    # Compute the L2 squared penalty
    reward = torch.square(asset.data.root_pos_w[:, 2] - adjusted_target_height)
    #  reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 1.0)  # don't penalize when upside down
    return reward


def root_height_below_target_l2(
    env: ManagerBasedRLEnv,
    std: float,
    target_margin: float = 0.05,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize only the amount by which the root height falls below the nominal height."""
    asset: Articulation = env.scene[asset_cfg.name]
    desired_height = asset.data.default_root_state[:, 2] - target_margin
    height_deficit = torch.relu(desired_height - asset.data.root_pos_w[:, 2])
    return torch.square(height_deficit / std)


def joint_power_l1(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Reward joint_power l1"""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # compute the reward
    reward = torch.sum(
        torch.abs(asset.data.joint_vel[:, asset_cfg.joint_ids] * asset.data.applied_torque[:, asset_cfg.joint_ids]),
        dim=1,
    )
    return reward


def joint_deviation_l1(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize joint positions that deviate from the default one."""
    asset: Articulation = env.scene[asset_cfg.name]
    angle = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    return torch.sum(torch.abs(angle), dim=1)


def joint_deviation_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize joint positions that deviate from the default one using L2 squared kernel."""
    asset: Articulation = env.scene[asset_cfg.name]
    angle = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    return torch.sum(torch.square(angle), dim=1)


def joint_deviation_humanoid_hip_yaw_l1(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    ang_vel_threshold: float = 0.1,
    zero_ang_vel_command_weight_scale: float = 1.0,
) -> torch.Tensor:
    """Penalize joint positions that deviate from the default one."""
    asset: Articulation = env.scene[asset_cfg.name]
    angle = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    command_vel_z = torch.abs(env.command_manager.get_command("base_velocity")[:, 2])
    reward = torch.sum(torch.abs(angle), dim=1)
    return torch.where(
        command_vel_z > ang_vel_threshold,
        reward,
        reward * zero_ang_vel_command_weight_scale,
    )


def joint_pos_limits(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize joint positions if they cross the soft limits."""
    asset: Articulation = env.scene[asset_cfg.name]
    out_of_limits = -(
        asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.soft_joint_pos_limits[:, asset_cfg.joint_ids, 0]
    ).clip(max=0.0)
    out_of_limits += (
        asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.soft_joint_pos_limits[:, asset_cfg.joint_ids, 1]
    ).clip(min=0.0)
    return torch.sum(out_of_limits, dim=1)


def joint_vel_hinge_l2_cost(
    env: ManagerBasedRLEnv,
    max_vel: float,
    std: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize selected joint velocities only after they exceed a soft limit."""
    asset: Articulation = env.scene[asset_cfg.name]
    joint_vel_abs = torch.abs(asset.data.joint_vel[:, asset_cfg.joint_ids])
    excess = torch.clamp(joint_vel_abs - max_vel, min=0.0)
    return torch.mean(torch.square(excess / std), dim=-1)


def joint_pos_deviation_from_default_l2_cost(
    env: ManagerBasedRLEnv,
    std: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize selected joint position deviation from the default pose."""
    asset: Articulation = env.scene[asset_cfg.name]
    error = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    return torch.mean(torch.square(error / std), dim=-1)


def body_relative_z_l2_cost(
    env: ManagerBasedRLEnv,
    target_z: float,
    std: float,
    body_cfg: SceneEntityCfg,
    anchor_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Penalize selected body z height relative to an anchor body."""
    asset: Articulation = env.scene[body_cfg.name]
    body_z = asset.data.body_pos_w[:, body_cfg.body_ids, 2]
    anchor_z = asset.data.body_pos_w[:, anchor_cfg.body_ids[0], 2]
    error = body_z - anchor_z[:, None] - target_z
    return torch.mean(torch.square(error / std), dim=-1)


def body_side_height_l2_cost(
    env: ManagerBasedRLEnv,
    target_z: float,
    z_std: float,
    min_lateral_abs_y: float,
    y_std: float,
    body_cfg: SceneEntityCfg,
    anchor_cfg: SceneEntityCfg,
    max_lateral_abs_y: float | None = None,
    max_y_std: float | None = None,
) -> torch.Tensor:
    """Penalize hand height and keep left/right bodies inside a lateral side band."""
    asset: Articulation = env.scene[body_cfg.name]
    anchor_id = anchor_cfg.body_ids[0]
    anchor_pos_w = asset.data.body_pos_w[:, anchor_id, :]
    anchor_yaw_quat = yaw_quat(asset.data.body_quat_w[:, anchor_id])

    rel_pos_w = asset.data.body_pos_w[:, body_cfg.body_ids] - anchor_pos_w[:, None, :]
    rel_quat = anchor_yaw_quat[:, None, :].expand(-1, rel_pos_w.shape[1], -1).reshape(-1, 4)
    rel_pos_h = quat_apply_inverse(rel_quat, rel_pos_w.reshape(-1, 3)).reshape(rel_pos_w.shape)

    rel_y = rel_pos_h[..., 1]
    rel_z = rel_pos_h[..., 2]
    z_cost = torch.mean(torch.square((rel_z - target_z) / z_std), dim=-1)

    if rel_y.shape[1] != 2:
        raise ValueError("body_side_height_l2_cost expects exactly two left/right body links.")
    side_y = torch.stack((rel_y[:, 0], -rel_y[:, 1]), dim=-1)
    near_deficit = torch.clamp(min_lateral_abs_y - side_y, min=0.0)
    near_cost = torch.mean(torch.square(near_deficit / y_std), dim=-1)

    far_cost = torch.zeros_like(near_cost)
    if max_lateral_abs_y is not None:
        far_std = y_std if max_y_std is None else max_y_std
        far_deficit = torch.clamp(side_y - max_lateral_abs_y, min=0.0)
        far_cost = torch.mean(torch.square(far_deficit / far_std), dim=-1)

    return z_cost + near_cost + far_cost


def body_relative_lin_acc_l2_cost(
    env: ManagerBasedRLEnv,
    std: float,
    body_cfg: SceneEntityCfg,
    anchor_cfg: SceneEntityCfg,
) -> torch.Tensor:
    """Penalize selected body linear acceleration relative to an anchor body."""
    asset: Articulation = env.scene[body_cfg.name]
    body_acc_w = asset.data.body_lin_acc_w[:, body_cfg.body_ids]
    anchor_acc_w = asset.data.body_lin_acc_w[:, anchor_cfg.body_ids[0], :]
    rel_acc = torch.linalg.norm(body_acc_w - anchor_acc_w[:, None, :], dim=-1)
    return torch.mean(torch.square(rel_acc / std), dim=-1)


def body_rot_kinetic_energy_hinge_cost(
    env: ManagerBasedRLEnv,
    free_energy: float,
    energy_scale: float,
    inertia_diag: tuple[tuple[float, float, float], ...],
    axis_weights: tuple[float, float, float] = (1.0, 1.0, 1.0),
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize rotational kinetic energy of selected bodies above a free margin."""
    asset: Articulation = env.scene[asset_cfg.name]
    body_ids = asset_cfg.body_ids
    if len(inertia_diag) != len(body_ids):
        raise ValueError(f"Expected {len(body_ids)} inertia entries, got {len(inertia_diag)}.")

    ang_vel_w = asset.data.body_ang_vel_w[:, body_ids, :]
    quat_w = asset.data.body_quat_w[:, body_ids, :]
    ang_vel_b = quat_apply_inverse(quat_w.reshape(-1, 4), ang_vel_w.reshape(-1, 3)).reshape(ang_vel_w.shape)

    inertia = torch.tensor(inertia_diag, dtype=ang_vel_b.dtype, device=ang_vel_b.device)
    weights = torch.tensor(axis_weights, dtype=ang_vel_b.dtype, device=ang_vel_b.device)
    axis_energy = 0.5 * inertia[None, :, :] * weights[None, None, :] * torch.square(ang_vel_b)
    total_energy = torch.sum(axis_energy, dim=(-1, -2))
    excess = torch.clamp(total_energy - free_energy, min=0.0)
    return torch.square(excess / energy_scale)


def stand_still(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    velocity_threshold: float = 0.2,
) -> torch.Tensor:
    """Penalize joint position error from default on the articulation."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    command_vel_xy = torch.linalg.norm(env.command_manager.get_command("base_velocity")[:, :2], dim=1)
    body_vel_xy = torch.linalg.norm(asset.data.root_lin_vel_b[:, :2], dim=1)
    return torch.where(
        torch.logical_or(command_vel_xy > 0.0, body_vel_xy > velocity_threshold),
        0.0,
        torch.linalg.norm((asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]), dim=1)
    )


def stand_still_contacts(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    sensor_cfg: SceneEntityCfg = SceneEntityCfg("contact_forces"),
    command_threshold: float | None = None,
    lin_command_threshold: float = 0.1,
    ang_command_threshold: float = 0.1,
    body_velocity_threshold: float = 0.1,
    use_body_velocity_gate: bool = True,
) -> torch.Tensor:
    """Penalize if none of the desired contacts are present."""
    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    command = env.command_manager.get_command("base_velocity")
    lin_command = torch.linalg.norm(command[:, :2], dim=1)
    ang_command = torch.abs(command[:, 2])
    forces_z = torch.abs(contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, 2])
    num_contacts = torch.sum(forces_z > 5.0, dim=1)
    required_contacts = forces_z.shape[1]
    not_all_contacts = num_contacts < required_contacts
    if command_threshold is not None:
        lin_command_threshold = command_threshold
        ang_command_threshold = command_threshold
    is_stand_still = (lin_command < lin_command_threshold) & (ang_command < ang_command_threshold)
    if use_body_velocity_gate:
        asset: Articulation = env.scene[asset_cfg.name]
        body_vel = torch.linalg.norm(asset.data.root_lin_vel_b[:, :2], dim=1)
        is_stand_still = torch.logical_or(is_stand_still, body_vel < body_velocity_threshold)
    return 1.0 * not_all_contacts * is_stand_still


def feet_air_time_variance(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize variance in the amount of time each foot spends in the air/on the ground relative to each other"""
    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    if contact_sensor.cfg.track_air_time is False:
        raise RuntimeError("Activate ContactSensor's track_air_time!")
    # compute the reward
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    last_contact_time = contact_sensor.data.last_contact_time[:, sensor_cfg.body_ids]
    return torch.var(torch.clip(last_air_time, max=0.5), dim=1) + torch.var(
        torch.clip(last_contact_time, max=0.5), dim=1
    )


def feet_stumble(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize feet hitting vertical surfaces"""
    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    forces_z = torch.abs(contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, 2])
    forces_xy = torch.linalg.norm(contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, :2], dim=2)
    # Penalize feet hitting vertical surfaces
    reward = torch.any(forces_xy > 4 * forces_z, dim=1).float()
    #  reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 1.0)  # don't penalize when upside down
    return reward


def feet_air_time(
    env: ManagerBasedRLEnv, command_name: str, sensor_cfg: SceneEntityCfg, threshold: float
) -> torch.Tensor:
    """Reward long steps taken by the feet using L2-kernel while respecting command magnitude."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    first_contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    reward = torch.sum((last_air_time - threshold) * first_contact, dim=1)
    reward *= (torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1) | (torch.abs(env.command_manager.get_command(command_name)[:, 2]) > 0.1)
    #  reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 1.0)  # don't penalize when upside down
    return reward


def speed_gated_support_center(
    env: ManagerBasedRLEnv,
    std: float,
    command_name: str,
    body_cfg: SceneEntityCfg,
    feet_body_cfg: SceneEntityCfg,
    forward_speed_scale: float = 2.0,
    lateral_speed_scale: float = 0.8,
    max_forward_offset: float = 0.16,
    max_lateral_offset: float = 0.08,
) -> torch.Tensor:
    """Reward a command-dependent body/support-center relationship."""
    asset: Articulation = env.scene[body_cfg.name]
    command = env.command_manager.get_command(command_name)

    body_id = body_cfg.body_ids[0]
    body_pos_w = asset.data.body_pos_w[:, body_id]
    body_quat_w = asset.data.body_quat_w[:, body_id]
    feet_pos_w = asset.data.body_pos_w[:, feet_body_cfg.body_ids]
    support_center_w = torch.mean(feet_pos_w, dim=1)

    support_rel_h = quat_apply_inverse(yaw_quat(body_quat_w), support_center_w - body_pos_w)
    desired_rel = torch.zeros_like(support_rel_h[:, :2])
    desired_rel[:, 0] = -max_forward_offset * torch.tanh(command[:, 0] / forward_speed_scale)
    desired_rel[:, 1] = -max_lateral_offset * torch.tanh(command[:, 1] / lateral_speed_scale)

    support_error = torch.sum(torch.square(support_rel_h[:, :2] - desired_rel), dim=-1)
    return torch.exp(-support_error / std**2)


def speed_gated_knee_alignment(
    env: ManagerBasedRLEnv,
    std: float,
    command_name: str,
    body_cfg: SceneEntityCfg,
    hip_body_cfg: SceneEntityCfg,
    knee_body_cfg: SceneEntityCfg,
    ankle_body_cfg: SceneEntityCfg,
    command_threshold: float = 0.2,
    forward_speed_scale: float = 1.0,
    side_speed_scale: float = 0.5,
    turn_speed_scale: float = 0.7,
) -> torch.Tensor:
    """Reward knee alignment with the hip-ankle leg plane during mostly-forward motion."""
    asset: Articulation = env.scene[body_cfg.name]
    command = env.command_manager.get_command(command_name)

    body_id = body_cfg.body_ids[0]
    body_pos_w = asset.data.body_pos_w[:, body_id]
    body_quat_w = asset.data.body_quat_w[:, body_id]
    hip_pos_w = asset.data.body_pos_w[:, hip_body_cfg.body_ids]
    knee_pos_w = asset.data.body_pos_w[:, knee_body_cfg.body_ids]
    ankle_pos_w = asset.data.body_pos_w[:, ankle_body_cfg.body_ids]
    num_legs = hip_pos_w.shape[1]

    heading_quat = yaw_quat(body_quat_w)[:, None, :].expand(-1, num_legs, -1).reshape(-1, 4)
    body_pos = body_pos_w[:, None, :].expand(-1, num_legs, -1)
    hip_h = quat_apply_inverse(heading_quat, (hip_pos_w - body_pos).reshape(-1, 3)).reshape(-1, num_legs, 3)
    knee_h = quat_apply_inverse(heading_quat, (knee_pos_w - body_pos).reshape(-1, 3)).reshape(-1, num_legs, 3)
    ankle_h = quat_apply_inverse(heading_quat, (ankle_pos_w - body_pos).reshape(-1, 3)).reshape(-1, num_legs, 3)

    hip_xy = hip_h[..., :2]
    knee_xy = knee_h[..., :2]
    ankle_xy = ankle_h[..., :2]
    leg_xy = ankle_xy - hip_xy
    knee_xy_rel = knee_xy - hip_xy
    leg_len_sq = torch.sum(torch.square(leg_xy), dim=-1, keepdim=True).clamp(min=1e-5)
    proj = torch.sum(knee_xy_rel * leg_xy, dim=-1, keepdim=True) / leg_len_sq
    knee_xy_proj = hip_xy + torch.clamp(proj, 0.0, 1.0) * leg_xy
    knee_plane_error = torch.sum(torch.square(knee_xy - knee_xy_proj), dim=-1)

    forward_gate = torch.clamp((torch.abs(command[:, 0]) - command_threshold) / forward_speed_scale, 0.0, 1.0)
    side_gate = torch.clamp(torch.abs(command[:, 1]) / side_speed_scale, 0.0, 1.0)
    turn_gate = torch.clamp(torch.abs(command[:, 2]) / turn_speed_scale, 0.0, 1.0)
    active_gate = forward_gate * (1.0 - side_gate) * (1.0 - turn_gate)

    return active_gate * torch.exp(-torch.mean(knee_plane_error, dim=-1) / std**2)


def speed_gated_foot_heading(
    env: ManagerBasedRLEnv,
    std: float,
    command_name: str,
    body_cfg: SceneEntityCfg,
    feet_body_cfg: SceneEntityCfg,
    sensor_cfg: SceneEntityCfg | None = None,
    command_threshold: float = 0.2,
    forward_speed_scale: float = 1.0,
    side_speed_scale: float = 0.5,
    turn_speed_scale: float = 0.7,
    use_command_gate: bool = True,
    left_toe_out_yaw: float = 0.0,
    right_toe_out_yaw: float = 0.0,
) -> torch.Tensor:
    """Reward foot heading during mostly-forward locomotion to reduce toe-in."""
    asset: Articulation = env.scene[body_cfg.name]
    command = env.command_manager.get_command(command_name)

    body_quat_w = asset.data.body_quat_w[:, body_cfg.body_ids[0]]
    feet_quat_w = asset.data.body_quat_w[:, feet_body_cfg.body_ids]
    batch_size, num_feet = feet_quat_w.shape[:2]

    foot_axis = torch.zeros((batch_size, num_feet, 3), device=feet_quat_w.device, dtype=feet_quat_w.dtype)
    foot_axis[..., 0] = 1.0
    foot_forward_w = quat_apply(feet_quat_w.reshape(-1, 4), foot_axis.reshape(-1, 3)).reshape(batch_size, num_feet, 3)
    foot_forward_h = quat_apply_inverse(
        yaw_quat(body_quat_w)[:, None, :].expand(-1, num_feet, -1).reshape(-1, 4),
        foot_forward_w.reshape(-1, 3),
    ).reshape(batch_size, num_feet, 3)

    foot_yaw = torch.atan2(foot_forward_h[..., 1], foot_forward_h[..., 0])
    target_yaw = torch.zeros_like(foot_yaw)
    if num_feet > 0:
        target_yaw[:, 0] = left_toe_out_yaw
    if num_feet > 1:
        target_yaw[:, 1] = right_toe_out_yaw
    yaw_error = torch.atan2(torch.sin(foot_yaw - target_yaw), torch.cos(foot_yaw - target_yaw))

    if use_command_gate:
        forward_gate = torch.clamp((torch.abs(command[:, 0]) - command_threshold) / forward_speed_scale, 0.0, 1.0)
        side_gate = torch.clamp(torch.abs(command[:, 1]) / side_speed_scale, 0.0, 1.0)
        turn_gate = torch.clamp(torch.abs(command[:, 2]) / turn_speed_scale, 0.0, 1.0)
        active_gate = forward_gate * (1.0 - side_gate) * (1.0 - turn_gate)
    else:
        active_gate = torch.ones_like(command[:, 0])

    foot_weights = torch.ones_like(foot_yaw)
    if sensor_cfg is not None:
        contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
        contact = (contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids] > 0.0).float()
        if contact.shape[-1] == num_feet:
            foot_weights = contact

    weighted_error = torch.sum(torch.square(yaw_error) * foot_weights, dim=-1)
    weight_sum = torch.sum(foot_weights, dim=-1)
    mean_error = weighted_error / torch.clamp(weight_sum, min=1.0)
    heading_reward = torch.where(
        weight_sum > 0.0,
        torch.exp(-mean_error / std**2),
        torch.zeros_like(mean_error),
    )
    return active_gate * heading_reward


def speed_accel_gated_foot_contact_force(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    body_cfg: SceneEntityCfg,
    base_force: float = 300.0,
    speed_force_scale: float = 220.0,
    robot_acc_force_scale: float = 55.0,
    cmd_acc_force_scale: float = 80.0,
    force_normalizer: float = 450.0,
    robot_acc_scale: float = 8.0,
    cmd_acc_scale: float = 3.0,
) -> torch.Tensor:
    """Penalize foot contact forces above a speed/acceleration-dependent allowance."""
    asset: Articulation = env.scene[body_cfg.name]
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    command = env.command_manager.get_command(command_name)

    force_mag = torch.norm(contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids], dim=-1)
    contact = (force_mag > 1.0).float()
    cmd_speed = torch.norm(command[:, :2], dim=-1)
    body_vel = asset.data.body_lin_vel_w[:, body_cfg.body_ids[0], :2]

    prev_body_vel = getattr(env, "_amp_prev_foot_force_body_vel", None)
    robot_acc = torch.zeros_like(cmd_speed) if not isinstance(prev_body_vel, torch.Tensor) else torch.norm((body_vel - prev_body_vel) / env.step_dt, dim=-1)
    setattr(env, "_amp_prev_foot_force_body_vel", body_vel.detach().clone())

    prev_command = getattr(env, "_amp_prev_foot_force_command", None)
    cmd_acc = torch.zeros_like(cmd_speed) if not isinstance(prev_command, torch.Tensor) else torch.norm((command[:, :2] - prev_command[:, :2]) / env.step_dt, dim=-1)
    setattr(env, "_amp_prev_foot_force_command", command.detach().clone())

    robot_acc_gate = torch.clamp(robot_acc / robot_acc_scale, 0.0, 1.0)
    cmd_acc_gate = torch.clamp(cmd_acc / cmd_acc_scale, 0.0, 1.0)
    allowed_force = (
        base_force
        + speed_force_scale * cmd_speed
        + robot_acc_force_scale * robot_acc_gate
        + cmd_acc_force_scale * cmd_acc_gate
    )
    excess_force = torch.relu(force_mag - allowed_force[:, None])
    contact_count = torch.clamp(torch.sum(contact, dim=-1), min=1.0)
    return torch.sum(torch.square(excess_force / force_normalizer) * contact, dim=-1) / contact_count


def foot_contact_force_spike(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    body_cfg: SceneEntityCfg,
    base_peak_force: float = 1100.0,
    speed_peak_force_scale: float = 260.0,
    robot_acc_peak_force_scale: float = 80.0,
    cmd_acc_peak_force_scale: float = 120.0,
    base_delta_force: float = 500.0,
    robot_acc_delta_force_scale: float = 40.0,
    cmd_acc_delta_force_scale: float = 60.0,
    peak_normalizer: float = 1000.0,
    delta_normalizer: float = 800.0,
    peak_weight: float = 0.5,
    delta_weight: float = 1.0,
) -> torch.Tensor:
    """Penalize short contact-force spikes without penalizing normal support force."""
    asset: Articulation = env.scene[body_cfg.name]
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    command = env.command_manager.get_command(command_name)

    force_mag = torch.norm(contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids], dim=-1)
    contact = (force_mag > 1.0).float()
    active_force = force_mag * contact
    peak_force = torch.max(active_force, dim=-1).values

    cmd_speed = torch.norm(command[:, :2], dim=-1)
    body_vel = asset.data.body_lin_vel_w[:, body_cfg.body_ids[0], :2]
    prev_body_vel = getattr(env, "_amp_prev_spike_body_vel", None)
    robot_acc = torch.zeros_like(cmd_speed) if not isinstance(prev_body_vel, torch.Tensor) else torch.norm((body_vel - prev_body_vel) / env.step_dt, dim=-1)
    setattr(env, "_amp_prev_spike_body_vel", body_vel.detach().clone())

    prev_command = getattr(env, "_amp_prev_spike_command", None)
    cmd_acc = torch.zeros_like(cmd_speed) if not isinstance(prev_command, torch.Tensor) else torch.norm((command[:, :2] - prev_command[:, :2]) / env.step_dt, dim=-1)
    setattr(env, "_amp_prev_spike_command", command.detach().clone())

    prev_force = getattr(env, "_amp_prev_spike_force", None)
    force_delta = torch.zeros_like(force_mag) if not isinstance(prev_force, torch.Tensor) else torch.relu(active_force - prev_force)
    setattr(env, "_amp_prev_spike_force", active_force.detach().clone())
    peak_delta = torch.max(force_delta, dim=-1).values

    peak_allowance = (
        base_peak_force
        + speed_peak_force_scale * cmd_speed
        + robot_acc_peak_force_scale * robot_acc
        + cmd_acc_peak_force_scale * cmd_acc
    )
    delta_allowance = base_delta_force + robot_acc_delta_force_scale * robot_acc + cmd_acc_delta_force_scale * cmd_acc
    peak_excess = torch.relu(peak_force - peak_allowance)
    delta_excess = torch.relu(peak_delta - delta_allowance)
    return peak_weight * torch.square(peak_excess / peak_normalizer) + delta_weight * torch.square(delta_excess / delta_normalizer)


def speed_gated_arm_leg_sagittal_antiphase_vel(
    env: ManagerBasedRLEnv,
    std: float,
    command_name: str,
    arm_body_cfg: SceneEntityCfg,
    leg_body_cfg: SceneEntityCfg,
    anchor_cfg: SceneEntityCfg,
    command_threshold: float = 0.2,
    forward_speed_scale: float = 1.0,
    side_speed_scale: float = 0.5,
    turn_speed_scale: float = 0.7,
    arm_lateral_vel_std: float = 0.25,
    arm_vertical_vel_std: float = 0.5,
    arm_lateral_vel_cost_scale: float = 0.6,
    arm_vertical_vel_cost_scale: float = 0.15,
) -> torch.Tensor:
    """Reward arm and same-side leg sagittal velocities moving in opposite phase."""
    asset: Articulation = env.scene[arm_body_cfg.name]
    if len(arm_body_cfg.body_ids) != 2 or len(leg_body_cfg.body_ids) != 2:
        raise ValueError("speed_gated_arm_leg_sagittal_antiphase_vel expects left/right arm and leg body links.")

    command = env.command_manager.get_command(command_name)
    forward_gate = torch.clamp((torch.abs(command[:, 0]) - command_threshold) / forward_speed_scale, 0.0, 1.0)
    side_gate = torch.clamp(torch.abs(command[:, 1]) / side_speed_scale, 0.0, 1.0)
    turn_gate = torch.clamp(torch.abs(command[:, 2]) / turn_speed_scale, 0.0, 1.0)
    active_gate = forward_gate * (1.0 - side_gate) * (1.0 - turn_gate)

    anchor_id = anchor_cfg.body_ids[0]
    anchor_yaw_quat = yaw_quat(asset.data.body_quat_w[:, anchor_id])
    anchor_vel_w = asset.data.body_lin_vel_w[:, anchor_id, :]

    arm_vel_w = asset.data.body_lin_vel_w[:, arm_body_cfg.body_ids] - anchor_vel_w[:, None, :]
    leg_vel_w = asset.data.body_lin_vel_w[:, leg_body_cfg.body_ids] - anchor_vel_w[:, None, :]
    heading_quat = anchor_yaw_quat[:, None, :].expand(-1, 2, -1).reshape(-1, 4)
    arm_vel_h = quat_apply_inverse(heading_quat, arm_vel_w.reshape(-1, 3)).reshape(-1, 2, 3)
    leg_vel_h = quat_apply_inverse(heading_quat, leg_vel_w.reshape(-1, 3)).reshape(-1, 2, 3)

    sagittal_error = torch.mean(torch.square(arm_vel_h[..., 0] + leg_vel_h[..., 0]), dim=-1)
    lateral_cost = torch.mean(torch.square(arm_vel_h[..., 1] / arm_lateral_vel_std), dim=-1)
    vertical_cost = torch.mean(torch.square(arm_vel_h[..., 2] / arm_vertical_vel_std), dim=-1)
    shaped_error = (
        sagittal_error / std**2
        + arm_lateral_vel_cost_scale * lateral_cost
        + arm_vertical_vel_cost_scale * vertical_cost
    )
    return active_gate * torch.exp(-shaped_error)


def feet_slide(
    env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Penalize feet sliding when contact forces exceed threshold."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    contacts = contact_sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :].norm(dim=-1).max(dim=1)[0] > 1.0
    asset: Articulation = env.scene[asset_cfg.name]
    body_vel = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2]
    reward = torch.sum(body_vel.norm(dim=-1) * contacts, dim=1)
    #  reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 1.0)  # don't penalize when upside down
    return reward


def feet_flat_contact_humanoid(
    env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward feet being oriented vertically when in contact with the ground."""
    # extract the used quantities (to enable type-hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
    left_quat = asset.data.body_quat_w[:, asset_cfg.body_ids[0], :]
    left_projected_gravity = quat_apply_inverse(left_quat, asset.data.GRAVITY_VEC_W)
    right_quat = asset.data.body_quat_w[:, asset_cfg.body_ids[1], :]
    right_projected_gravity = quat_apply_inverse(right_quat, asset.data.GRAVITY_VEC_W)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    net_contact_forces = contact_sensor.data.net_forces_w_history
    is_contact = torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] > 1

    return (
        torch.sum(torch.square(left_projected_gravity[:, :2]), dim=-1) ** 0.5 * is_contact[:, 0]
        + torch.sum(torch.square(right_projected_gravity[:, :2]), dim=-1) ** 0.5 * is_contact[:, 1]
    )


def feet_gait(
    env: ManagerBasedRLEnv,
    period: float,
    offset: list[float],
    sensor_cfg: SceneEntityCfg,
    threshold: float = 0.5,
    velocity_threshold: float = 0.2,
) -> torch.Tensor:
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    is_contact = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids] > 0

    global_phase = ((env.episode_length_buf * env.step_dt) % period / period).unsqueeze(1)
    phases = []
    for offset_ in offset:
        phase = (global_phase + offset_) % 1.0
        phases.append(phase)
    leg_phase = torch.cat(phases, dim=-1)

    reward = torch.zeros(env.num_envs, dtype=torch.float, device=env.device)
    for i in range(len(sensor_cfg.body_ids)):
        is_stance = leg_phase[:, i] < threshold
        reward += ~(is_stance ^ is_contact[:, i])

    reward *= torch.norm(env.command_manager.get_command("base_velocity")[:, :2], dim=1) > velocity_threshold

    return reward


def feet_clearance_flat(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, target_height: float, std: float, tanh_mult: float
) -> torch.Tensor:
    """Reward the swinging feet for clearing a specified height off the flat ground"""
    asset: RigidObject = env.scene[asset_cfg.name]
    feet_z_target_error = torch.square(asset.data.body_pos_w[:, asset_cfg.body_ids, 2] - target_height)
    feet_velocity_tanh = torch.tanh(tanh_mult * torch.norm(asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2], dim=2))
    reward = feet_z_target_error * feet_velocity_tanh
    return torch.exp(-torch.sum(reward, dim=1) / std)


def feet_clearance_rough(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    target_height: float = 0.1,
    std: float = 0.05,
    tanh_mult: float = 2.0,
) -> torch.Tensor:
    """Reward swinging feet for reaching a target height above local rough terrain."""
    asset: RigidObject = env.scene[asset_cfg.name]
    feet_names = [asset.body_names[body_id] for body_id in asset_cfg.body_ids]
    pos_z = torch.stack([env.scene.sensors[f"{name}_height_scanner"].data.pos_w[:, 2] for name in feet_names], dim=1)
    ray_hits_z = torch.stack(
        [env.scene.sensors[f"{name}_height_scanner"].data.ray_hits_w[..., 2] for name in feet_names], dim=1
    )
    feet_height = (pos_z.unsqueeze(-1) - ray_hits_z).mean(dim=-1)

    feet_height_target_error = torch.square(feet_height - target_height)
    feet_velocity_tanh = torch.tanh(tanh_mult * torch.norm(asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2], dim=2))
    reward = feet_height_target_error * feet_velocity_tanh
    return torch.exp(-torch.sum(reward, dim=1) / std)


def feet_clearance(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    target_height_in_base_frame: float = -0.18,
) -> torch.Tensor:
    """Reward swinging feet for reaching a target height in the base frame."""
    asset: RigidObject = env.scene[asset_cfg.name]

    feet_pos_rel_w = asset.data.body_pos_w[:, asset_cfg.body_ids, :] - asset.data.root_pos_w.unsqueeze(1)
    feet_vel_rel_w = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :] - asset.data.root_lin_vel_w.unsqueeze(1)

    num_feet = feet_pos_rel_w.shape[1]
    root_quat_w = asset.data.root_quat_w.unsqueeze(1).expand(-1, num_feet, -1).reshape(-1, 4)
    feet_pos_b = quat_apply_inverse(root_quat_w, feet_pos_rel_w.reshape(-1, 3)).reshape(
        env.num_envs, num_feet, 3
    )
    feet_vel_b = quat_apply_inverse(root_quat_w, feet_vel_rel_w.reshape(-1, 3)).reshape(
        env.num_envs, num_feet, 3
    )

    height_error = torch.square(feet_pos_b[:, :, 2] - target_height_in_base_frame)
    feet_vel_b_xy = torch.linalg.norm(feet_vel_b[:, :, :2], dim=2)
    return torch.sum(height_error * feet_vel_b_xy, dim=1)


def action_rate_l2(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize the rate of change of the actions using L2 squared kernel."""
    return torch.sum(torch.square(env.action_manager.action - env.action_manager.prev_action), dim=1)


def stuck_penalty(
    env: ManagerBasedRLEnv,
    command_name: str,
    velocity_threshold: float = 0.1,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize when robot is stuck: command velocity is above threshold but actual velocity is very small.

    Args:
        env: The RL environment instance.
        command_name: Name of the velocity command in the command manager.
        actual_velocity_threshold: Maximum actual velocity magnitude to consider as stuck (default: 0.1 m/s).
        asset_cfg: Scene entity configuration for the robot asset.

    Returns:
        Penalty value (1.0 when stuck, 0.0 otherwise).
    """
    asset: RigidObject = env.scene[asset_cfg.name]
    # Get velocity_threshold from command term
    # Get command velocity magnitude (xy plane)
    command_vel_xy = torch.linalg.norm(env.command_manager.get_command(command_name)[:, :2], dim=1)
    # Get actual velocity magnitude (xy plane)
    actual_vel_xy = torch.linalg.norm(asset.data.root_lin_vel_b[:, :2], dim=1)
    # Robot is stuck when: command is above threshold but actual velocity is very small
    is_stuck = torch.logical_and(
        command_vel_xy > velocity_threshold,
        actual_vel_xy < velocity_threshold,
    )
    return is_stuck.float()


def undesired_contacts(env: ManagerBasedRLEnv, threshold: float, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize undesired contacts as the number of violations that are above a threshold."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    net_contact_forces = contact_sensor.data.net_forces_w_history
    is_contact = torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] > threshold
    return torch.sum(is_contact, dim=1)


def self_collisions(env: ManagerBasedRLEnv, force_threshold: float, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize self-collision-like contacts above a force threshold."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    net_contact_forces = contact_sensor.data.net_forces_w_history
    force_mag = torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1)
    is_collision = torch.max(force_mag, dim=1)[0] > force_threshold
    return torch.sum(is_collision, dim=1)


def is_alive(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Reward for being alive."""
    return (~env.termination_manager.terminated).float()


# This term seems to lead to slower speed ?
class action_smoothness_l2(ManagerTermBase):
    """
    Reward term for penalizing large instantaneous changes in the network action output (L2 norm).
    This penalty encourages smoother actions over time.
    """

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        """Initialize the term.

        Args:
            cfg: The configuration of the reward term.
            env: The RL environment instance.
        """
        super().__init__(cfg, env)
        self.prev_prev_action = None
        self.prev_action = None

    def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        """Compute the action smoothness penalty.

        Args:
            env: The RL environment instance.

        Returns:
            The penalty value based on the action smoothness.
        """
        # Get the current action from the environment's action manager
        current_action = env.action_manager.action.clone()

        # If this is the first call, initialize the previous actions
        if self.prev_action is None:
            self.prev_action = current_action
            return torch.zeros(current_action.shape[0], device=current_action.device)

        if self.prev_prev_action is None:
            self.prev_prev_action = self.prev_action
            self.prev_action = current_action
            return torch.zeros(current_action.shape[0], device=current_action.device)

        # Compute the smoothness penalty
        action_smoothness_penalty = torch.sum(
            torch.square(current_action - 2 * self.prev_action + self.prev_prev_action), dim=1
        )

        # Update the previous actions for the next call
        self.prev_prev_action = self.prev_action
        self.prev_action = current_action

        # Apply a condition to ignore penalty during the first few episodes
        startup_env_mask = env.episode_length_buf < 3
        action_smoothness_penalty[startup_env_mask] = 0

        # Return the penalty scaled by the configured weight
        return action_smoothness_penalty
