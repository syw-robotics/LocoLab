"""Joint and observation settings shared by deployment exporters."""

import re

import numpy as np


OBSERVATION_TYPES = {
    "base_lin_vel": "BaseLinearVelocityObservation",
    "base_ang_vel": "BaseAngularVelocityObservation",
    "projected_gravity": "ProjectedGravityObservation",
    "command": "CommandObservation",
    "joint_pos_rel": "JointPositionObservation",
    "joint_vel": "JointVelocityObservation",
    "prev_action": "PreviousActionObservation",
}


TERM_TYPES = {
    "base_lin_vel": "base_lin_vel",
    "base_ang_vel": "base_ang_vel",
    "projected_gravity": "projected_gravity",
    "generated_commands": "command",
    "joint_pos_rel": "joint_pos_rel",
    "joint_pos_rel_biased": "joint_pos_rel",
    "joint_vel_rel": "joint_vel",
    "last_action": "prev_action",
}


def joint_values(value, names, default: float) -> list[float]:
    if not isinstance(value, dict):
        return [float(value)] * len(names)
    return [next((float(v) for pattern, v in value.items() if re.fullmatch(pattern, name)), default)
            for name in names]


def joint_gains(robot, names):
    kp, kd = [0.0] * len(names), [0.0] * len(names)
    for actuator in robot.cfg.actuators.values():
        stiffness = joint_values(actuator.stiffness, names, 0.0)
        damping = joint_values(actuator.damping, names, 0.0)
        for i, name in enumerate(names):
            if any(re.fullmatch(pattern, name) for pattern in actuator.joint_names_expr):
                kp[i], kd[i] = stiffness[i], damping[i]
    return kp, kd


def observation_processing(term) -> dict:
    """Translate Isaac Lab's clip-then-scale to deployment's scale-then-clip."""
    if term.tensor_processors or term.delay_max_lag:
        raise ValueError("unitree-deploy export does not support observation processors or delays.")
    spec = {}
    scale = 1.0 if term.scale is None else term.scale
    if not np.isscalar(scale):
        raise ValueError("unitree-deploy export supports scalar observation scales only.")
    if term.scale is not None:
        spec["scale"] = float(scale)
    if term.clip is not None:
        spec["clip"] = sorted(float(v * scale) for v in term.clip)
    return spec


def tensor_list(value):
    return getattr(value, "torch", value).detach().cpu().tolist()
