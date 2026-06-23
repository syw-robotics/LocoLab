# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Symmetry augmentation for Unitree G1 velocity tasks."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch
from tensordict import TensorDict

from z_rl.extensions import (
    TensorMirrorSpec,
    augment_mirrored_tensor,
    build_name_mirror_spec,
    build_obs_group_mirror_spec,
)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

__all__ = ["compute_symmetric_states", "compute_g1_symmetric_states"]

_G1_SYMMETRY_CACHE_ATTR = "_locolab_g1_symmetry_cache"


@torch.no_grad()
def compute_symmetric_states(
    env,
    obs: TensorDict | None = None,
    actions: torch.Tensor | None = None,
) -> tuple[TensorDict | None, torch.Tensor | None]:
    """Append left-right mirrored G1 observations/actions for z_rl symmetry training."""
    cache = _get_symmetry_cache(env)

    obs_aug = None
    if obs is not None:
        obs_aug_data = {}
        for group_name, group_obs in obs.items():
            if group_name not in cache["obs_specs"]:
                obs_aug_data[group_name] = torch.cat((group_obs, group_obs.clone()), dim=0)
                continue
            obs_aug_data[group_name] = augment_mirrored_tensor(group_obs, cache["obs_specs"][group_name])
        obs_aug = TensorDict(obs_aug_data, batch_size=[obs.batch_size[0] * 2], device=obs.device)

    actions_aug = None
    if actions is not None:
        actions_aug = augment_mirrored_tensor(actions, cache["action_spec"])

    return obs_aug, actions_aug


compute_g1_symmetric_states = compute_symmetric_states


def _get_symmetry_cache(env) -> dict:
    cache = getattr(env, _G1_SYMMETRY_CACHE_ATTR, None)
    if cache is not None:
        return cache

    base_env = env.unwrapped
    action_spec = _build_action_mirror_spec(base_env)
    obs_specs = _build_obs_mirror_specs(env, action_spec)
    cache = {"action_spec": action_spec, "obs_specs": obs_specs}
    setattr(env, _G1_SYMMETRY_CACHE_ATTR, cache)
    return cache


def _build_obs_mirror_specs(env, action_spec: TensorMirrorSpec) -> dict[str, TensorMirrorSpec]:
    if not hasattr(env, "obs_format"):
        raise ValueError("G1 symmetry requires a z_rl environment wrapper exposing `obs_format`.")

    obs_specs = {}
    layout_mode_map = getattr(env, "obs_group_layout_mode_map", {})
    for group_name, group_format in env.obs_format.items():
        term_specs = {}
        for term_name, term_format in group_format.items():
            frame_dim = int(torch.Size(term_format[1:]).numel())
            term_spec = _build_term_mirror_spec(term_name, frame_dim, action_spec)
            if term_spec is not None:
                term_specs[term_name] = term_spec
        obs_specs[group_name] = build_obs_group_mirror_spec(
            env.obs_format,
            group_name,
            term_specs,
            layout_mode=layout_mode_map.get(group_name, "term_major"),
            strict=False,
        )
    return obs_specs


def _build_term_mirror_spec(
    term_name: str,
    frame_dim: int,
    action_spec: TensorMirrorSpec,
) -> TensorMirrorSpec | None:
    if term_name == "base_lin_vel":
        return _sign_spec(term_name, frame_dim, [1.0, -1.0, 1.0])
    if term_name == "base_ang_vel":
        return _sign_spec(term_name, frame_dim, [-1.0, 1.0, -1.0])
    if term_name == "projected_gravity":
        return _sign_spec(term_name, frame_dim, [1.0, -1.0, 1.0])
    if term_name == "velocity_commands":
        return _sign_spec(term_name, frame_dim, [1.0, -1.0, -1.0])
    if term_name in {"joint_pos", "joint_vel", "joint_effort", "joint_acc", "actions"}:
        if frame_dim != len(action_spec.index):
            raise ValueError(
                f"G1 symmetry term '{term_name}' dim={frame_dim} does not match action dim={len(action_spec.index)}."
            )
        return action_spec
    if term_name == "feet_contact_flag":
        if frame_dim != 2:
            raise ValueError(f"G1 feet_contact_flag symmetry expects dim=2, got {frame_dim}.")
        return TensorMirrorSpec(index=[1, 0], name="feet_contact_flag")
    return None


def _build_action_mirror_spec(env: ManagerBasedRLEnv) -> TensorMirrorSpec:
    action_manager = env.action_manager
    if "joint_pos" not in action_manager.active_terms:
        raise KeyError(
            "G1 symmetry expects a 'joint_pos' action term. "
            f"Available action terms: {action_manager.active_terms}."
        )
    action_term = action_manager.get_term("joint_pos")
    joint_names = tuple(action_term._joint_names)
    return build_name_mirror_spec(
        joint_names,
        _mirror_joint_name,
        _mirror_joint_sign,
        name="g1_joint_action",
    )


def _sign_spec(term_name: str, frame_dim: int, sign: list[float]) -> TensorMirrorSpec:
    if frame_dim != len(sign):
        raise ValueError(f"G1 term '{term_name}' symmetry expects dim={len(sign)}, got {frame_dim}.")
    return TensorMirrorSpec(index=list(range(frame_dim)), sign=sign, name=term_name)


def _mirror_joint_name(joint_name: str) -> str:
    if joint_name.startswith("left_"):
        return "right_" + joint_name.removeprefix("left_")
    if joint_name.startswith("right_"):
        return "left_" + joint_name.removeprefix("right_")
    return joint_name


def _mirror_joint_sign(joint_name: str) -> float:
    if "roll" in joint_name or "yaw" in joint_name:
        return -1.0
    return 1.0
