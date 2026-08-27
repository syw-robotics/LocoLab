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
from isaaclab.utils.math import quat_apply_inverse, yaw_quat

if TYPE_CHECKING:
    from isaaclab.assets import Articulation, RigidObject
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.sensors import ContactSensor, RayCaster


# -------------------- Velocity Tracking Rewards -------------------- #

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


# -------------------- Stability Rewards -------------------- #

def lin_vel_z_l2(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    selected_bodies: bool = False,
) -> torch.Tensor:
    """Penalize z-axis root or selected-body linear velocity using an L2 squared kernel."""
    asset: RigidObject = env.scene[asset_cfg.name]
    if selected_bodies:
        return torch.sum(torch.square(asset.data.body_lin_vel_w[:, asset_cfg.body_ids, 2]), dim=1)
    else:
        return torch.square(asset.data.root_lin_vel_b[:, 2])


def ang_vel_xy_l2(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    selected_bodies: bool = False,
) -> torch.Tensor:
    """Penalize xy-axis root or selected-body angular velocity using an L2 squared kernel."""
    asset: RigidObject = env.scene[asset_cfg.name]
    if selected_bodies:
        return torch.sum(torch.square(asset.data.body_ang_vel_w[:, asset_cfg.body_ids, :2]), dim=(1, 2))
    else:
        return torch.sum(torch.square(asset.data.root_ang_vel_b[:, :2]), dim=1)


def flat_orientation_l2(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    selected_bodies: bool = False,
) -> torch.Tensor:
    """Penalize non-flat base orientation using L2 squared kernel by penalizing projected gravity xy-components."""
    asset: RigidObject = env.scene[asset_cfg.name]
    if selected_bodies:
        body_quat_w = asset.data.body_quat_w[:, asset_cfg.body_ids]
        gravity_w = asset.data.GRAVITY_VEC_W[:, None, :].expand(*body_quat_w.shape[:-1], -1)
        projected_gravity_b = quat_apply_inverse(body_quat_w, gravity_w)
        return torch.sum(torch.square(projected_gravity_b[..., :2]), dim=(1, 2))
    else:
        return torch.sum(torch.square(asset.data.projected_gravity_b[:, :2]), dim=1)


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


# -------------------- Joint Regularization Rewards -------------------- #

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


def joint_paired_direction_coordination(
    env: ManagerBasedRLEnv,
    direction_scale: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize opposing relative-position directions between two joint pairs using a smooth kernel.

    The asset configuration must select four joints in the order
    ``[pair_0_a, pair_0_b, pair_1_a, pair_1_b]``.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    joint_pos_rel = (
        asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    )
    joint_direction = torch.tanh(joint_pos_rel / direction_scale)
    first_pair_penalty = torch.relu(-joint_direction[:, 0] * joint_direction[:, 1])
    second_pair_penalty = torch.relu(-joint_direction[:, 2] * joint_direction[:, 3])
    return first_pair_penalty + second_pair_penalty


def joint_paired_position_coordination(
    env: ManagerBasedRLEnv,
    target_scale: float,
    error_scale: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalize paired joints that deviate from a scaled relative-position target.

    The asset configuration must select joints in the order
    ``[source_0, target_0, source_1, target_1, ...]``. For each pair, the
    target joint is expected to follow ``target_scale * source_position``.
    Use a negative ``target_scale`` when the paired joints should move in
    opposite directions.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    joint_pos_rel = (
        asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    )
    # [num_envs, num_pairs, 2]
    joint_pairs = joint_pos_rel.reshape(joint_pos_rel.shape[0], -1, 2)
    source_position = joint_pairs[..., 0]
    target_position = joint_pairs[..., 1]

    normalized_error = (target_position - target_scale * source_position) / error_scale

    # Pseudo-Huber: quadratic near zero and linear for large errors.
    pair_penalty = torch.sqrt(1.0 + normalized_error.square()) - 1.0

    return torch.mean(pair_penalty, dim=1)


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


# -------------------- Gait Rewards -------------------- #

def command_is_moving(
    env: ManagerBasedRLEnv,
    command_name: str,
    threshold: float = 0.1,
) -> torch.Tensor:
    """Return whether the planar or yaw velocity command exceeds a threshold."""
    command = env.command_manager.get_command(command_name)
    planar_is_moving = torch.sum(command[:, :2].square(), dim=1) > threshold**2
    yaw_is_moving = command[:, 2].abs() > threshold
    return planar_is_moving | yaw_is_moving


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
) -> torch.Tensor:
    """Penalize if none of the desired contacts are present."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    command_vel = torch.linalg.norm(env.command_manager.get_command("base_velocity"), dim=1)
    body_vel = torch.linalg.norm(asset.data.root_lin_vel_b[:, :2], dim=1)
    forces_z = torch.abs(contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, 2])
    num_contacts = torch.sum(forces_z > 5.0, dim=1)
    not_all_contacts = num_contacts != 4
    is_stand_still = torch.logical_or(command_vel < 0.1, body_vel < 0.1)
    return 1.0 * not_all_contacts * is_stand_still


def feet_air_time(
    env: ManagerBasedRLEnv, command_name: str, sensor_cfg: SceneEntityCfg, threshold: float
) -> torch.Tensor:
    """Reward long steps taken by the feet using L2-kernel while respecting command magnitude."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    first_contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    reward = torch.sum((last_air_time - threshold) * first_contact, dim=1)
    reward *= command_is_moving(env, command_name)
    #  reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 1.0)  # don't penalize when upside down
    return reward


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


def feet_too_near_bipedal(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"), threshold: float = 0.15
) -> torch.Tensor:
    """Penalize the distance deficit when the two selected feet are too close."""
    asset: RigidObject = env.scene[asset_cfg.name]
    body_pos_w = asset.data.body_pos_w
    feet_delta = body_pos_w[:, asset_cfg.body_ids[0]] - body_pos_w[:, asset_cfg.body_ids[1]]
    distance = torch.linalg.vector_norm(feet_delta, dim=-1)
    return torch.clamp_min(threshold - distance, 0.0)


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


class feet_gait(ManagerTermBase):
    """Reward feet whose contact states match the configured periodic gait.

    Each foot follows the same normalized gait cycle with an individual phase
    offset. A phase below ``threshold`` represents stance; otherwise it
    represents swing. The reward is the number of feet whose actual contact
    state matches the expected state.

    The phase offsets are static configuration, so they are transferred to the
    simulation device once when the reward term is initialized.
    """

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        period = cfg.params["period"]
        offset = cfg.params.get("offset", [0.0, 0.5])
        sensor_cfg = cfg.params["sensor_cfg"]

        if period <= 0.0:
            raise ValueError(f"Expected period to be positive, got {period}.")
        if len(offset) != len(sensor_cfg.body_ids):
            raise ValueError(
                f"Expected one phase offset per selected foot, got {len(offset)} offsets "
                f"and {len(sensor_cfg.body_ids)} feet."
            )

        self._phase_offsets = torch.tensor(offset, device=env.device).unsqueeze(0)

    def __call__(
        self,
        env: ManagerBasedRLEnv,
        period: float,
        sensor_cfg: SceneEntityCfg,
        offset: list[float] = [0.0, 0.5],  # this is a bipedal setup
        threshold: float = 0.5,
        command_name: str | None = None,
    ) -> torch.Tensor:
        contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
        is_contact = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids] > 0

        global_phase = ((env.episode_length_buf * env.step_dt) % period / period).unsqueeze(1)
        leg_phase = torch.remainder(global_phase + self._phase_offsets, 1.0)

        # A threshold above 0.5 introduces a double-stance portion of the cycle.
        is_stance = leg_phase < threshold
        reward = torch.sum(is_stance == is_contact, dim=1, dtype=torch.float)

        if command_name is not None:
            reward *= command_is_moving(env, command_name)
        return reward


# -------------------- MISC Rewards -------------------- #

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


def is_alive(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Reward for being alive."""
    return (~env.termination_manager.terminated).float()


# This term seems to lead to slower speed ?
#  class action_smoothness_l2(ManagerTermBase):
#      """
#      Reward term for penalizing large instantaneous changes in the network action output (L2 norm).
#      This penalty encourages smoother actions over time.
#      """
#
#      def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
#          """Initialize the term.
#
#          Args:
#              cfg: The configuration of the reward term.
#              env: The RL environment instance.
#          """
#          super().__init__(cfg, env)
#          self.prev_prev_action = None
#          self.prev_action = None
#
#      def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
#          """Compute the action smoothness penalty.
#
#          Args:
#              env: The RL environment instance.
#
#          Returns:
#              The penalty value based on the action smoothness.
#          """
#          # Get the current action from the environment's action manager
#          current_action = env.action_manager.action.clone()
#
#          # If this is the first call, initialize the previous actions
#          if self.prev_action is None:
#              self.prev_action = current_action
#              return torch.zeros(current_action.shape[0], device=current_action.device)
#
#          if self.prev_prev_action is None:
#              self.prev_prev_action = self.prev_action
#              self.prev_action = current_action
#              return torch.zeros(current_action.shape[0], device=current_action.device)
#
#          # Compute the smoothness penalty
#          action_smoothness_penalty = torch.sum(
#              torch.square(current_action - 2 * self.prev_action + self.prev_prev_action), dim=1
#          )
#
#          # Update the previous actions for the next call
#          self.prev_prev_action = self.prev_action
#          self.prev_action = current_action
#
#          # Apply a condition to ignore penalty during the first few episodes
#          startup_env_mask = env.episode_length_buf < 3
#          action_smoothness_penalty[startup_env_mask] = 0
#
#          # Return the penalty scaled by the configured weight
#          return action_smoothness_penalty


# Reference implementation with fewer allocations. To use it, uncomment the class and
# point the reward configuration to ``mdp.action_smoothness_l2_optimized``.
class action_smoothness_l2(ManagerTermBase):
   """Penalize the second finite difference of actions using preallocated buffers."""

   def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
       super().__init__(cfg, env)
       action = env.action_manager.action
       # ActionManager already stores a_t and a_{t-1}, so only a_{t-2} is needed here.
       self._prev_prev_action = torch.zeros_like(action)
       self._second_difference = torch.empty_like(action)

   def reset(self, env_ids=None) -> None:
       """Clear history for environments that begin a new episode."""
       if env_ids is None:
           env_ids = slice(None)
       self._prev_prev_action[env_ids] = 0.0

   def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
       action = env.action_manager.action
       prev_action = env.action_manager.prev_action

       # second_difference = a_t - 2 * a_{t-1} + a_{t-2}
       torch.add(action, self._prev_prev_action, out=self._second_difference)
       self._second_difference.add_(prev_action, alpha=-2.0)

       # Preserve a_{t-1} before the action manager advances on the next step.
       self._prev_prev_action.copy_(prev_action)

       self._second_difference.square_()
       penalty = torch.sum(self._second_difference, dim=1)
       penalty.masked_fill_(env.episode_length_buf < 3, 0.0)
       return penalty
