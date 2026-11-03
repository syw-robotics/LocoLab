"""Bundle a single LocoLab tracking clip and its deployment contract into ONNX."""

from __future__ import annotations

import json
import re
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper
import yaml


REFERENCE_NAMES = (
    "joint_pos", "joint_vel", "body_pos_w", "body_quat_w",
    "body_lin_vel_w", "body_ang_vel_w",
)


def bundle_tracking_motion(path: str | Path, motion, config: dict) -> None:
    """Append frame lookup to an already normalized actor; preserve its computation.

    time_step is int64[1], in control steps. Out-of-range indices hold the
    nearest endpoint. Deployment-facing body quaternions use wxyz.
    """
    path = Path(path)
    model = onnx.load(path)
    if len(model.graph.input) != 1 or len(model.graph.output) != 1:
        raise ValueError("Tracking deployment requires a feed-forward actor with one input/output.")
    config = dict(config, policy_input_name=model.graph.input[0].name,
                  policy_output_name=model.graph.output[0].name)
    model.graph.input.append(helper.make_tensor_value_info("time_step", TensorProto.INT64, [1]))
    for name, value in (("motion_min", 0), ("motion_max", motion.time_step_total - 1)):
        model.graph.initializer.append(numpy_helper.from_array(np.array([value], dtype=np.int64), name))
    model.graph.node.extend([
        helper.make_node("Max", ["time_step", "motion_min"], ["motion_nonnegative"]),
        helper.make_node("Min", ["motion_nonnegative", "motion_max"], ["motion_index"]),
    ])
    for name in REFERENCE_NAMES:
        tensor = getattr(motion, name)
        values = tensor.detach().cpu().numpy().astype(np.float32)
        if name == "body_quat_w":
            values = values[..., [3, 0, 1, 2]]
        model.graph.initializer.append(numpy_helper.from_array(values, "motion_" + name))
        model.graph.node.append(helper.make_node("Gather", ["motion_" + name, "motion_index"], [name], axis=0))
        model.graph.output.append(helper.make_tensor_value_info(name, TensorProto.FLOAT, [1, *values.shape[1:]]))
    metadata = {item.key: item.value for item in model.metadata_props}
    metadata.update({
        "locolab_tracking_version": "1",
        "deployment_config": json.dumps(config),
        "motion_num_frames": str(motion.time_step_total),
        "motion_dt": str(motion.output_dt),
        "motion_source": Path(motion.motion_file).name,
        "quaternion_order": "wxyz",
    })
    helper.set_model_props(model, metadata)
    onnx.checker.check_model(model)
    onnx.save_model(model, path, save_as_external_data=False)
    path.with_name("policy.yaml").write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")


def _joint_values(value, names, default: float) -> list[float]:
    if not isinstance(value, dict):
        return [float(value)] * len(names)
    return [next((float(v) for pattern, v in value.items() if re.fullmatch(pattern, name)), default)
            for name in names]


def export_tracking_deployment(env, path: str | Path, clip_actions: float | None = None) -> None:
    """Called after the runner's regular ONNX export; velocity exports are untouched."""
    if not hasattr(env.cfg.commands, "motion"):
        return
    command = env.command_manager.get_term("motion")
    names = list(command.cfg.joint_names)
    action = env.action_manager.get_term("joint_pos")
    if list(action._joint_names) != names:
        raise ValueError("Tracking action and motion joint orders must match.")
    if not action.cfg.use_default_offset:
        raise ValueError("Tracking deployment expects actions offset by the default joint pose.")
    if action.cfg.clip is not None:
        raise ValueError("Tracking deployment does not support per-joint target clipping.")
    data = command.robot.data.default_joint_pos
    defaults = getattr(data, "torch", data)[0, command._joint_ids].detach().cpu().tolist()
    kp, kd = [0.0] * len(names), [0.0] * len(names)
    for actuator in command.robot.cfg.actuators.values():
        stiffness = _joint_values(actuator.stiffness, names, 0.0)
        damping = _joint_values(actuator.damping, names, 0.0)
        for i, name in enumerate(names):
            if any(re.fullmatch(pattern, name) for pattern in actuator.joint_names_expr):
                kp[i], kd[i] = stiffness[i], damping[i]
    type_names = {"command": "motion_command", "joint_pos": "joint_pos_rel", "actions": "prev_action"}
    observations = []
    for name in env.observation_manager.active_terms["policy"]:
        term = getattr(env.cfg.observations.policy, name)
        if term.history_length or term.tensor_processors or term.delay_max_lag:
            raise ValueError("Tracking deployment expects unstacked observations without modifiers.")
        spec = {"type": type_names.get(name, name), "history_len": 1}
        if term.scale is not None and term.clip is not None:
            raise ValueError("Combined observation scale and clip have different ordering in deployment.")
        if term.scale is not None:
            spec["scale"] = list(term.scale) if isinstance(term.scale, tuple) else term.scale
        if term.clip is not None:
            spec["clip"] = list(term.clip)
        observations.append(spec)
    config = {
        "robot": "g1", "imu_source": "pelvis", "policy_path": Path(path).name,
        "policy_class": "unitree_deploy.policy.motion_tracking:MotionTrackingPolicy",
        "policy_step_dt": float(env.step_dt), "physics_dt": 0.001,
        "decimation": round(env.step_dt / 0.001), "runtime_command_dim": 3,
        "obs_joint_order": names, "action_joint_order": names, "sdk_joint_order": names,
        "default_qpos": defaults, "action_dim": len(names), "action_clip": clip_actions,
        "motion_start_q": command.motion.joint_pos[0].detach().cpu().tolist(),
        "action_scale": _joint_values(action.cfg.scale, names, 1.0),
        "kp_policy": kp, "kd_policy": kd, "kp_fixed_stand": kp, "kd_fixed_stand": kd,
        "kd_damping": kd, "obs_use_scaled_prev_action": False, "obs_prime_on_reset": True,
        "obs_group_concat_mode": "term_major",
        "observation_modules": ["unitree_deploy.obs.motion_tracking_obs"],
        "observation_types": {
            "base_ang_vel": "unitree_deploy.obs.base_obs:BaseAngularVelocityObservation",
            "joint_pos_rel": "unitree_deploy.obs.base_obs:JointPositionObservation",
            "joint_vel": "unitree_deploy.obs.base_obs:JointVelocityObservation",
            "prev_action": "unitree_deploy.obs.base_obs:PreviousActionObservation",
        },
        "observations": observations, "anchor_body_name": command.cfg.anchor_body_name,
        "body_names": list(command.cfg.body_names),
    }
    bundle_tracking_motion(path, command.motion, config)
    write_tracking_state_machine(Path(path).parent)


def write_tracking_state_machine(directory: str | Path) -> None:
    """Deploy beside a sibling vanilla_ppo_flat velocity policy."""
    directory = Path(directory)
    multi_ckpt = """default: locomotion
ckpts:
  locomotion: ../vanilla_ppo_flat
  motion_tracking: .
# Manual round-robin profile switching, separate from state-machine transitions.
switch:
  # Keep false here: B must reach locomotion -> prepare_motion for the arm handoff.
  enabled: false
  # Only used when enabled above; the workflow B key is in state_machine.yaml.
  button: B
  # Names from ckpts above, not filesystem paths.
  order: [locomotion, motion_tracking]
  # Controller STATE names where manual cycling is allowed.
  only_when: [locomotion]
  # null keeps the current state; a state name triggers it after manual switching.
  on_switch: null
# Sim2sim reference ghost; these settings do not affect real deployment.
motion_ghost:
  enabled: true  # initial visibility; the toggle key overrides it at runtime
  toggle_key: g  # choose an unused simulator key, separate from remote buttons
"""
    state_machine = {
        "initial": "damping",
        "states": {
            "damping": {"type": "damping", "transitions": {"A": "move_to_default_qpos"}},
            "move_to_default_qpos": {
                "type": "move_to_target_pos", "policy": "locomotion", "target": "default_q", "duration": 2.0,
                "kp": "fixed_stand", "kd": "fixed_stand",
                "transitions": {"Start": "locomotion", "X": "damping"},
            },
            "locomotion": {
                "type": "policy", "policy": "locomotion", "reset_policy": False,
                "transitions": {"B": "prepare_motion", "A": "move_to_default_qpos", "X": "damping"},
            },
            "prepare_motion": {
                "type": "arm_transition", "direction": "to_motion", "enabled": True,
                "locomotion_policy": "locomotion", "motion_policy": "motion_tracking", "duration": 2.0, "max_joint_speed": 1.0,
                "transitions": {"done": "motion_tracking", "X": "damping"},
            },
            "motion_tracking": {
                "type": "policy", "policy": "motion_tracking",
                "transitions": {"done": "return_locomotion", "X": "damping"},
            },
            "return_locomotion": {
                "type": "arm_transition", "direction": "to_locomotion",
                "locomotion_policy": "locomotion", "motion_policy": "motion_tracking", "duration": 3.0, "max_joint_speed": 1.0,
                "transitions": {"done": "locomotion", "X": "damping"},
            },
        },
    }
    (directory / "multi_ckpt.yaml").write_text(multi_ckpt, encoding="utf-8")
    note = (
        "# Arm interpolation: duration is nominal seconds (smaller = faster).\n"
        "# max_joint_speed caps each arm joint target in rad/s; null disables the cap.\n"
        "# Handoff waits until the capped targets reach their goals.\n"
    )
    (directory / "state_machine.yaml").write_text(
        note + yaml.safe_dump(state_machine, sort_keys=False), encoding="utf-8")
