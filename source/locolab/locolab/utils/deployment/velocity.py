"""Export proprioceptive G1, Go2, and B2 velocity policies for unitree-deploy."""

from pathlib import Path
import copy
import re

import onnx

from .common import OBSERVATION_TYPES, TERM_TYPES, joint_gains, joint_values, observation_processing, tensor_list
from .writers.readme import write_readme
from .writers.policy_yaml import write_policy_yaml


def export_velocity_deployment(env, path: str | Path, clip_actions: float | None = None) -> None:
    if getattr(env.cfg, "_locolab_warp", False):
        from isaaclab_experimental.envs.mdp.actions.joint_actions import JointPositionAction
    else:
        from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction

    path = Path(path)
    if list(env.action_manager.active_terms) != ["joint_pos"]:
        raise ValueError("Velocity deployment requires a single joint_pos action term.")
    action = env.action_manager.get_term("joint_pos")
    action = getattr(action, "_term", action)  # unwrap training-time actuator delay
    if not issubclass(action.cfg.class_type, JointPositionAction):
        raise ValueError("Velocity deployment requires absolute joint-position actions.")
    if not getattr(action.cfg, "use_default_offset", False):
        raise ValueError("Velocity deployment requires joint-position actions with default-pose offsets.")
    if getattr(action.cfg, "use_reference_residual", False):
        raise ValueError("Reference-residual actions require the tracking exporter.")
    robot = env.scene[action.cfg.asset_name]
    robot_name = getattr(robot.cfg, "deployment_robot", None)
    if robot_name is None:
        raise ValueError("The task robot must define deployment_robot.")
    names = list(action._joint_names)
    sdk_names = list(robot.cfg.joint_sdk_names)
    if set(names) != set(sdk_names):
        raise ValueError("Velocity deployment requires all SDK joints to be controlled.")
    sdk_ids = [robot.joint_names.index(name) for name in sdk_names]
    defaults = tensor_list(robot.data.default_joint_pos)[0]
    default_vel = tensor_list(robot.data.default_joint_vel)[0]
    if any(default_vel[i] != 0 for i in sdk_ids):
        raise ValueError("Velocity deployment requires zero default joint velocities.")
    kp, kd = joint_gains(robot, sdk_names)
    group = env.cfg.observations.policy
    if not group.concatenate_terms or group.concatenate_dim != -1:
        raise ValueError("Velocity deployment requires flat, concatenated policy observations.")
    observations = []
    obs_dim = 0
    for name, term in zip(env.observation_manager.active_terms["policy"],
                          env.observation_manager._group_obs_term_cfgs["policy"]):
        if hasattr(term, "_warp_history_length"):
            # LocoLab Warp folds history, clip, and scale into ObservationHistory.
            term = copy.copy(term)
            term.func = term._warp_source
            term.history_length = term._warp_history_length
            term.history_newest_first = False  # the Warp history kernel is chronological
            term.clip = None if term._warp_clip == (-float("inf"), float("inf")) else term._warp_clip
            term.scale = None if term._warp_scale == 1.0 else term._warp_scale
        func_name = getattr(term.func, "__name__", type(term.func).__name__)
        if func_name not in TERM_TYPES:
            raise ValueError(f"Unsupported velocity observation {name!r} ({func_name}); no deployment equivalent.")
        kind = TERM_TYPES[func_name]
        if term.history_length and not term.flatten_history_dim:
            raise ValueError("Velocity deployment requires flattened observation history.")
        spec = {"type": kind, "history_len": max(1, term.history_length),
                "history_order": "newest_first" if term.history_newest_first else "oldest_first"}
        spec.update(observation_processing(term))
        if kind in ("joint_pos_rel", "joint_vel"):
            asset = term.params["asset_cfg"]
            ids = getattr(asset.joint_ids, "torch", asset.joint_ids)
            selected = robot.joint_names[ids] if isinstance(ids, slice) else [robot.joint_names[int(i)] for i in ids]
            if asset.name != action.cfg.asset_name or list(selected) != names:
                raise ValueError(f"Observation {name!r} must use the action joint order.")
        if kind == "command":
            if term.params.get("command_name") != "base_velocity":
                raise ValueError("Velocity deployment requires base_velocity commands.")
            ranges = env.command_manager.get_term("base_velocity").cfg.ranges
            spec["command_range"] = [list(ranges.lin_vel_x), list(ranges.lin_vel_y), list(ranges.ang_vel_z)]
        if kind == "prev_action" and term.params.get("action_name") is not None:
            raise ValueError("Velocity deployment expects the complete previous action.")
        obs_dim += spec["history_len"] * (len(names) if kind in ("joint_pos_rel", "joint_vel", "prev_action") else 3)
        observations.append(spec)
    if not any(spec["type"] == "prev_action" for spec in observations):
        raise ValueError("Velocity deployment requires a previous-action observation.")
    if not any(spec["type"] == "command" for spec in observations):
        raise ValueError("Velocity deployment requires a base_velocity command observation.")
    model = onnx.load(path, load_external_data=False)
    if len(model.graph.input) != 1 or len(model.graph.output) != 1:
        raise ValueError("Velocity deployment requires a feed-forward actor with one input/output.")
    input_shape = model.graph.input[0].type.tensor_type.shape.dim
    output_shape = model.graph.output[0].type.tensor_type.shape.dim
    if len(input_shape) != 2 or input_shape[-1].dim_value != obs_dim:
        raise ValueError(f"Actor input does not match the exported policy observation dimension {obs_dim}.")
    if len(output_shape) != 2 or output_shape[-1].dim_value != len(names):
        raise ValueError("Actor output does not match the joint-position action dimension.")
    config = {
        "robot": robot_name,
        "policy_path": path.name, "policy_class": "unitree_deploy.policy.velocity:VelocityPolicy",
        "policy_step_dt": float(env.step_dt), "physics_dt": float(env.cfg.sim.dt),
        "decimation": env.cfg.decimation,
        "runtime_command_dim": env.command_manager.get_term("base_velocity").command.shape[-1],
        "policy_input_name": model.graph.input[0].name, "policy_output_name": model.graph.output[0].name,
        "obs_joint_order": names, "action_joint_order": names, "sdk_joint_order": sdk_names,
        "default_qpos": [defaults[i] for i in sdk_ids], "action_dim": len(names), "action_clip": clip_actions,
        "action_scale": joint_values(action.cfg.scale, sdk_names, 1.0),
        "kp_policy": kp, "kd_policy": kd, "kp_fixed_stand": kp, "kd_fixed_stand": kd, "kd_damping": kd,
        "obs_use_scaled_prev_action": False, "obs_prime_on_reset": True,
        "obs_group_concat_mode": "history_major" if group.history_order == "time" and group.history_length else "term_major",
        "observation_types": {kind: f"unitree_deploy.obs.base_obs:{cls}" for kind, cls in OBSERVATION_TYPES.items()},
        "observations": observations,
    }
    if getattr(robot.cfg, "deployment_imu_source", None) is not None:
        config["imu_source"] = robot.cfg.deployment_imu_source
    if action.cfg.clip is not None:
        limits = []
        for name in names:
            bounds = next((v for pattern, v in action.cfg.clip.items() if re.fullmatch(pattern, name)), None)
            # YAML null bounds become NaN in NumPy; explicit infinities mean no clip.
            limits.append(list(bounds) if bounds is not None else [float("-inf"), float("inf")])
        config["action_target_clip"] = limits
    write_policy_yaml(path.with_name("policy.yaml"), config)
    write_readme(path.parent, robot_name, tracking=False)
