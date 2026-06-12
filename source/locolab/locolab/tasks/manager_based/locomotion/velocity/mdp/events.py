# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from __future__ import annotations

from typing import TYPE_CHECKING, Literal

import torch

from isaaclab.envs.mdp.events import _randomize_prop_by_op
from isaaclab.managers import EventTermCfg, ManagerTermBase, SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.assets import Articulation
    from isaaclab.envs import ManagerBasedEnv


class randomize_joint_offsets(ManagerTermBase):
    """Randomize joint encoder offsets by perturbing the articulation default joint positions.

    Isaac Lab's relative joint-position observation is computed as ``joint_pos - default_joint_pos``.
    Perturbing ``default_joint_pos`` therefore simulates a per-environment encoder zero-offset while leaving
    the current simulated joint state untouched. This term is intended to run once in ``startup`` mode.
    """

    def __init__(self, cfg: EventTermCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        self.asset_cfg: SceneEntityCfg = cfg.params["asset_cfg"]
        self.asset: Articulation = env.scene[self.asset_cfg.name]

    def __call__(
        self,
        env: ManagerBasedEnv,
        env_ids: torch.Tensor | None,
        asset_cfg: SceneEntityCfg,
        offsets_distribution_params: tuple[float, float],
        operation: Literal["add", "scale", "abs"] = "add",
        distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
    ):
        if env_ids is None:
            env_ids = torch.arange(env.scene.num_envs, device=self.asset.device)

        if self.asset_cfg.joint_ids == slice(None):
            joint_ids = slice(None)
        else:
            joint_ids = torch.tensor(self.asset_cfg.joint_ids, dtype=torch.int, device=self.asset.device)

        if env_ids != slice(None) and joint_ids != slice(None):
            env_ids_for_slice = env_ids[:, None]
        else:
            env_ids_for_slice = env_ids

        default_joint_pos = self.asset.data.default_joint_pos[env_ids_for_slice, joint_ids].clone()
        default_joint_pos = _randomize_prop_by_op(
            default_joint_pos,
            offsets_distribution_params,
            None,
            slice(None),
            operation=operation,
            distribution=distribution,
        )

        #  joint_pos_limits = self.asset.data.soft_joint_pos_limits[env_ids_for_slice, joint_ids]
        #  default_joint_pos = torch.clamp(default_joint_pos, joint_pos_limits[..., 0], joint_pos_limits[..., 1])
        self.asset.data.default_joint_pos[env_ids_for_slice, joint_ids] = default_joint_pos
