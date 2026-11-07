"""Bundle a tracking actor and its configured references for deployment."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

from .common import OBSERVATION_TYPES, TERM_TYPES, joint_gains, joint_values, observation_processing, tensor_list
from .writers.policy_yaml import write_policy_yaml
from .writers.readme import write_readme
from .writers.state_machine import write_tracking_state_machine


TRACKING_TERM_TYPES = {
    "motion_anchor_pos_b": "motion_anchor_pos_b",
    "motion_anchor_ori_b": "motion_anchor_ori_b",
}


def bundle_tracking_motion(path: str | Path, motion, config: dict) -> None:
    """Append frame lookup to an already normalized actor; preserve its computation.

    time_step is int64[1], in control steps. Out-of-range indices hold the
    nearest endpoint. Deployment-facing body quaternions use wxyz.
    """
    from locolab.utils.motion.command import command_dimension, command_sources

    path = Path(path)
    terms = tuple(config["motion_command_terms"])
    command_dimension(terms, len(config["action_joint_order"]))
    references = dict(zip(terms, command_sources(motion, terms)))
    # Reference trajectories support FK alignment, residual targets, and the deployment ghost.
    for name in ("joint_pos", "joint_vel", "body_pos_w", "body_quat_w", "body_lin_vel_w", "body_ang_vel_w"):
        references.setdefault(name, getattr(motion, name))
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
    for name, tensor in references.items():
        values = tensor.detach().cpu().numpy().astype(np.float32)
        if name == "body_quat_w":
            values = values[..., [3, 0, 1, 2]]
        model.graph.initializer.append(numpy_helper.from_array(values, "motion_" + name))
        model.graph.node.append(helper.make_node("Gather", ["motion_" + name, "motion_index"], [name], axis=0))
        model.graph.output.append(helper.make_tensor_value_info(name, TensorProto.FLOAT, [1, *values.shape[1:]]))
    metadata = {
        "motion_reference_names": json.dumps(list(references)),
        "motion_joint_names": json.dumps(config["action_joint_order"]),
        "motion_body_names": json.dumps(config["body_names"]),
        "motion_num_frames": str(motion.time_step_total),
        "motion_dt": str(motion.output_dt),
        "motion_source": Path(motion.motion_file).name,
        "quaternion_order": "wxyz",
    }
    helper.set_model_props(model, metadata)
    onnx.checker.check_model(model)
    onnx.save_model(model, path, save_as_external_data=False)
    write_policy_yaml(path.with_name("policy.yaml"), config)


def tracking_deployment_config(env, path: str | Path, clip_actions: float | None = None) -> tuple[dict, int]:
    """Describe the trained robot, actions, and policy observations without bundling a motion."""
    command = env.command_manager.get_term("motion")
    if tuple(command.cfg.command_frame_offsets) != (0,):
        raise ValueError("Tracking deployment currently supports only command_frame_offsets=(0,).")
    from locolab.utils.motion.command import command_dimension

    names = list(command.cfg.joint_names)
    terms = tuple(command.cfg.command_terms)
    command_dim = command_dimension(terms, len(names))
    if len(env.action_manager.active_terms) != 1:
        raise ValueError("Tracking deployment requires a single joint-position action term.")
    action = env.action_manager.get_term(env.action_manager.active_terms[0])
    action = getattr(action, "_term", action)
    if getattr(env.cfg, "_locolab_warp", False):
        from isaaclab_experimental.envs.mdp.actions.joint_actions import JointPositionAction
    else:
        from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction

    if not issubclass(action.cfg.class_type, JointPositionAction):
        raise ValueError("Tracking deployment requires joint-position actions.")
    if action.cfg.asset_name != command.cfg.asset_name:
        raise ValueError("Tracking actions and motion must use the same robot asset.")
    residual = getattr(action.cfg, "use_reference_residual", False)
    if list(action._joint_names) != names:
        raise ValueError("Tracking action and motion joint orders must match.")
    if not residual and not action.cfg.use_default_offset:
        raise ValueError("Tracking deployment expects actions offset by the default joint pose.")
    if action.cfg.clip is not None:
        raise ValueError("Tracking deployment does not support per-joint target clipping.")
    robot = command.robot
    robot_name = getattr(robot.cfg, "deployment_robot", None)
    if robot_name is None:
        raise ValueError("The task robot must define deployment_robot.")
    sdk_names = list(robot.cfg.joint_sdk_names)
    if set(names) != set(sdk_names):
        raise ValueError("Tracking deployment requires all SDK joints to be controlled.")
    sdk_ids = [robot.joint_names.index(name) for name in sdk_names]
    defaults = tensor_list(robot.data.default_joint_pos)[0]
    default_vel = tensor_list(robot.data.default_joint_vel)[0]
    if any(default_vel[i] != 0 for i in sdk_ids):
        raise ValueError("Tracking deployment requires zero default joint velocities.")
    kp, kd = joint_gains(robot, sdk_names)
    group = env.cfg.observations.policy
    if not group.concatenate_terms or group.concatenate_dim != -1:
        raise ValueError("Tracking deployment requires flat, concatenated policy observations.")
    observations = []
    obs_dim = 0
    type_names = dict(TERM_TYPES, **TRACKING_TERM_TYPES, generated_commands="motion_command")
    for name, term in zip(env.observation_manager.active_terms["policy"],
                          env.observation_manager._group_obs_term_cfgs["policy"]):
        if term.history_length and not term.flatten_history_dim:
            raise ValueError("Tracking deployment requires flattened observation history.")
        func_name = getattr(term.func, "__name__", type(term.func).__name__)
        if func_name not in type_names:
            raise ValueError(f"Unsupported tracking observation {name!r} ({func_name}); no deployment equivalent.")
        kind = type_names[func_name]
        spec = {"type": kind, "history_len": max(1, term.history_length)}
        if term.history_length:
            spec["history_order"] = "newest_first" if term.history_newest_first else "oldest_first"
        spec.update(observation_processing(term))
        if kind in ("motion_command", "motion_anchor_pos_b", "motion_anchor_ori_b"):
            if term.params.get("command_name") != "motion":
                raise ValueError(f"Observation {name!r} must use the exported motion command.")
        if kind in ("joint_pos_rel", "joint_vel"):
            asset = term.params["asset_cfg"]
            ids = getattr(asset.joint_ids, "torch", asset.joint_ids)
            selected = robot.joint_names[ids] if isinstance(ids, slice) else [robot.joint_names[int(i)] for i in ids]
            if asset.name != action.cfg.asset_name or list(selected) != names:
                raise ValueError(f"Observation {name!r} must use the action joint order.")
        if kind == "prev_action" and term.params.get("action_name") is not None:
            raise ValueError("Tracking deployment expects the complete previous action.")
        obs_dim += spec["history_len"] * (command_dim if kind == "motion_command" else
                    len(names) if kind in ("joint_pos_rel", "joint_vel", "prev_action") else
                    6 if kind == "motion_anchor_ori_b" else 3)
        observations.append(spec)
    config = {
        "robot": robot_name, "policy_path": Path(path).name,
        "policy_class": "unitree_deploy.policy.motion_tracking.motion_tracking_policy:MotionTrackingPolicy",
        "policy_step_dt": float(env.step_dt), "physics_dt": float(env.cfg.sim.dt),
        "decimation": env.cfg.decimation,
        "obs_joint_order": names, "action_joint_order": names, "sdk_joint_order": sdk_names,
        "default_qpos": [defaults[i] for i in sdk_ids], "action_dim": len(names), "action_clip": clip_actions,
        "action_scale": joint_values(action.cfg.scale, sdk_names, 1.0),
        "kp_policy": kp, "kd_policy": kd, "kp_fixed_stand": kp, "kd_fixed_stand": kd,
        "kd_damping": kd, "obs_use_scaled_prev_action": False, "obs_prime_on_reset": True,
        "obs_group_concat_mode": "history_major" if group.history_order == "time" and group.history_length else "term_major",
        "observation_modules": ["unitree_deploy.obs.motion_tracking_obs"],
        "observation_types": {kind: f"unitree_deploy.obs.base_obs:{cls}" for kind, cls in OBSERVATION_TYPES.items()
                              if any(spec["type"] == kind for spec in observations)},
        "observations": observations, "anchor_body_name": command.cfg.anchor_body_name,
        "body_names": list(command.cfg.body_names),
        "motion_command_terms": list(terms), "use_reference_residual": residual, "reference_source": "embedded",
    }
    if getattr(robot.cfg, "deployment_imu_source", None) is not None:
        config["imu_source"] = robot.cfg.deployment_imu_source
    return config, obs_dim


def export_tracking_deployment(env, path: str | Path, clip_actions: float | None = None) -> None:
    """Bundle a single-input tracking actor with its configured reference motion."""
    if not hasattr(env.cfg.commands, "motion"):
        return
    config, obs_dim = tracking_deployment_config(env, path, clip_actions)
    model = onnx.load(path, load_external_data=False)
    if len(model.graph.input) != 1 or len(model.graph.output) != 1:
        raise ValueError("Tracking deployment requires a feed-forward actor with one input/output.")
    input_shape = model.graph.input[0].type.tensor_type.shape.dim
    output_shape = model.graph.output[0].type.tensor_type.shape.dim
    if len(input_shape) != 2 or input_shape[-1].dim_value != obs_dim:
        raise ValueError(f"Actor input does not match the exported policy observation dimension {obs_dim}.")
    if len(output_shape) != 2 or output_shape[-1].dim_value != config["action_dim"]:
        raise ValueError("Actor output does not match the joint-position action dimension.")
    motion = env.command_manager.get_term("motion").motion
    start_q = tensor_list(motion.joint_pos[0])
    names = config["action_joint_order"]
    config["motion_start_q"] = [start_q[names.index(name)] for name in config["sdk_joint_order"]]
    bundle_tracking_motion(path, motion, config)
    write_tracking_state_machine(Path(path).parent)
    write_readme(Path(path).parent, config["robot"], tracking=True)
