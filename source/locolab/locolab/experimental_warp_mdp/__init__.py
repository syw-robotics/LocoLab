"""LocoLab Warp terms, reusing the experimental runtime and standard task IDs."""

import os

from isaaclab_experimental.envs.mdp import *  # noqa: F401, F403
from isaaclab_tasks_experimental.core.velocity.mdp import *  # noqa: F401, F403

from .actions import DelayedAction, EncoderBiasJointPositionAction
from .events import randomize_actuator_gains, randomize_encoder_bias, randomize_joint_parameters
from .observations import (
    feet_contact_flag,
    generated_commands,
    height_scan,
    joint_acc,
    joint_effort,
    joint_pos_rel_biased,
)
from .rewards import (
    action_smoothness_l2,
    ang_vel_xy_l2,
    base_height_l2,
    feet_air_time,
    feet_clearance_flat,
    feet_flight,
    feet_landing_vel_z_l2,
    feet_single_support,
    feet_short_swing,
    feet_slide,
    feet_standing_contact,
    feet_stumble,
    feet_too_near_bipedal,
    flat_orientation_l2,
    joint_power_l1,
    stand_still,
    track_ang_vel_z_exp,
    track_ang_vel_z_world_exp,
    track_lin_vel_xy_exp,
    track_lin_vel_xy_yaw_frame_exp,
    undesired_contacts,
)
from .terminations import bad_orientation, illegal_contact


def prepare_cfg(cfg):
    """Adapt supported tasks; reject missing terms instead of mixing manager interfaces."""
    from isaaclab_experimental.envs.frontend import WarpFrontend

    from locolab.tasks.manager_based.locomotion.velocity import mdp as velocity_mdp
    from locolab.tasks.manager_based.locomotion.velocity.config.unitree_b2.flat_env_cfg import (
        B2FlatEnvCfg,
        B2FlatEnvCfg_PLAY,
    )
    from locolab.tasks.manager_based.locomotion.velocity.config.unitree_g1.flat_env_cfg import (
        G1FlatEnvCfg,
        G1FlatEnvCfg_PLAY,
    )
    from locolab.tasks.manager_based.locomotion.velocity.config.unitree_go2.flat_env_cfg import (
        Go2FlatEnvCfg,
        Go2FlatEnvCfg_PLAY,
    )
    from locolab.tasks.manager_based.locomotion.velocity.config.unitree_go2.rough_env_cfg import (
        Go2RoughEnvCfg,
        Go2RoughEnvCfg_PLAY,
    )
    from locolab.tasks.manager_based.locomotion.tracking.config.unitree_g1.tracking_flat_env_cfg import (
        G1TrackingEnvCfg,
        G1TrackingEnvCfg_PLAY,
    )

    from .history import configure_history

    velocity_types = (
        Go2FlatEnvCfg, Go2FlatEnvCfg_PLAY, Go2RoughEnvCfg, Go2RoughEnvCfg_PLAY,
        B2FlatEnvCfg, B2FlatEnvCfg_PLAY, G1FlatEnvCfg, G1FlatEnvCfg_PLAY,
    )
    tracking_types = (G1TrackingEnvCfg, G1TrackingEnvCfg_PLAY)
    if type(cfg) not in (*velocity_types, *tracking_types):
        raise ValueError(
            "--mdp_backend warp currently supports Velocity-{Flat,Rough}-Go2, Velocity-Flat-{B2,G1},"
            " Tracking-Flat-G1 and their Play configs."
        )
    if getattr(cfg, "_locolab_warp", False):
        return
    WarpFrontend._require_newton_physics(cfg, type(cfg).__name__)
    # The runtime calls adapt_cfg again during construction; local twins must be recognized as Warp-native.
    if __name__ not in WarpFrontend.WARP_ROOT_PREFIXES:
        WarpFrontend.WARP_ROOT_PREFIXES += (__name__,)
    namespace = dict(globals())
    mdp = velocity_mdp
    if type(cfg) in tracking_types:
        from locolab.tasks.manager_based.locomotion.tracking import mdp
        from . import tracking
        from .motion_command import WarpMotionCommand
        from .actions import TrackingJointPositionAction

        namespace.update(vars(tracking))
        namespace["EncoderBiasJointPositionAction"] = TrackingJointPositionAction
        cfg.commands.motion.class_type = WarpMotionCommand
        # LocoLabWarpEnv refreshes live link/sensor caches before tracking graph replay.
    replacements = {
        value: namespace[name]
        for name, value in vars(mdp).items()
        if callable(value) and name in namespace and callable(namespace[name])
    }
    replacements[velocity_mdp.actions.delayed_actions.DelayedAction] = DelayedAction
    replacements[velocity_mdp.actions.delayed_actions.EncoderBiasJointPositionAction] = namespace[
        "EncoderBiasJointPositionAction"
    ]
    plan, missing = [], []
    for path, term in WarpFrontend._walk_terms(cfg):
        if path[0] not in WarpFrontend.WARP_MANAGED_GROUPS:
            continue
        for attr in ("func", "class_type"):
            source = getattr(term, attr, None)
            if source is None:
                continue
            twin = replacements.get(source)
            if twin is None:
                missing.append(".".join(path) + ": " + source.__name__)
            else:
                plan.append((term, attr, twin))
    if missing:
        raise ValueError("Missing LocoLab Warp terms: " + ", ".join(missing))
    for term, attr, twin in plan:
        setattr(term, attr, twin)
    # The frontend intentionally stops at action terms, so adapt the delayed action's nested term explicitly.
    if hasattr(cfg.actions.joint_pos, "action"):
        cfg.actions.joint_pos.action.class_type = EncoderBiasJointPositionAction
    WarpFrontend.adapt_cfg(cfg)
    configure_history(cfg)
    # Keep lazy sensor refresh outside manager graphs until its replay path is fixed.
    if getattr(cfg, "manager_call_config", None) is None:
        cfg.manager_call_config = os.environ.get("MANAGER_CALL_CONFIG") or {"default": 1}
    cfg._locolab_warp = True
    print(
        "[INFO] LocoLab Warp runtime: observations/history, actions, rewards, terminations, events;"
        f" manager_call_config={cfg.manager_call_config!r}."
    )
