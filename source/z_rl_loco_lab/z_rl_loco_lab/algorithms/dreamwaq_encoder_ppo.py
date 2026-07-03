# Copyright (c) 2021-2026, ETH Zurich and NVIDIA CORPORATION
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause


from __future__ import annotations

from dataclasses import dataclass, field

import torch
import torch.nn.functional as F
from tensordict import TensorDict

from z_rl.env import VecEnv
from z_rl.storage import RolloutStorage
from z_rl.utils import ObsSelector, resolve_target_obs_term_selector
from z_rl.algorithms.composition import ComposablePPO, PPOLossSpec

from z_rl_loco_lab.models import DreamWaQEncoderMLPModel


@dataclass(slots=True)
class DreamWaQLossSpec(PPOLossSpec):
    """PPO loss spec for DreamWaQ VAE reconstruction and linear velocity estimation."""

    obs_format: dict[str, dict[str, tuple[int, ...]]]
    obs_group_time_slice_map: dict[str, dict[str, ObsSelector]]
    target_obs_group_name: str = "critic"
    recon_target_obs_term_names: list[str] = field(
        default_factory=lambda: ["base_lin_vel", "base_ang_vel", "projected_gravity", "joint_pos", "joint_vel"]
    )
    lin_vel_target_obs_term_names: list[str] = field(default_factory=lambda: ["base_lin_vel"])
    recon_target_selector: ObsSelector = field(default_factory=lambda: ObsSelector(slice(0, 0)), init=False, repr=False)
    lin_vel_target_selector: ObsSelector = field(default_factory=lambda: ObsSelector(slice(0, 0)), init=False, repr=False)

    def validate(self, algo: object) -> None:
        """Validate the actor assumptions and cache resolved target selectors."""
        actor = getattr(algo, "actor", None)
        if not isinstance(actor, DreamWaQEncoderMLPModel):
            raise ValueError(
                f"`DreamWaQLossSpec` requires `algo.actor` to be `DreamWaQEncoderMLPModel`, got {type(actor)}."
            )

        self.recon_target_selector = resolve_target_obs_term_selector(
            target_obs_group_name=self.target_obs_group_name,
            target_obs_term_names=self.recon_target_obs_term_names,
            obs_group_time_slice_map=self.obs_group_time_slice_map,
            obs_format=self.obs_format,
        )
        self.lin_vel_target_selector = resolve_target_obs_term_selector(
            target_obs_group_name=self.target_obs_group_name,
            target_obs_term_names=self.lin_vel_target_obs_term_names,
            obs_group_time_slice_map=self.obs_group_time_slice_map,
            obs_format=self.obs_format,
        )

        actor_latent_adapter = actor.latent_adapter
        if actor_latent_adapter.reconstruction_dim != self.recon_target_selector.dim:
            raise ValueError(
                "`vae_decoder_output_dim` must match the selected reconstruction target dim, got "
                f"decoder_output_dim={actor_latent_adapter.reconstruction_dim} and "
                f"target_dim={self.recon_target_selector.dim}."
            )
        if actor_latent_adapter.lin_vel_dim != self.lin_vel_target_selector.dim:
            raise ValueError(
                "`lin_vel_dim` must match the selected linear velocity target dim, got "
                f"lin_vel_dim={actor_latent_adapter.lin_vel_dim} and target_dim={self.lin_vel_target_selector.dim}."
            )

    def compute(
        self,
        algo: object,
        minibatch: RolloutStorage.Batch,
    ) -> tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:
        """Compute auxiliary losses from the context returned by the update-time actor forward pass."""
        actor = getattr(algo, "_raw_actor", algo.actor)  # type: ignore[attr-defined]
        actor_latent_adapter = actor.latent_adapter
        context = getattr(algo, "actor_forward_context", None)
        
        lin_vel_target = self.lin_vel_target_selector.select(minibatch.observations[self.target_obs_group_name])
        lin_vel_est = context["lin_vel_est"]
        lin_vel_est_loss = F.mse_loss(lin_vel_est, lin_vel_target)

        recon_target = self.recon_target_selector.select(minibatch.observations[self.target_obs_group_name])
        vae_recon_loss, vae_kl_loss = actor_latent_adapter.dreamwaq_vae.compute_vae_loss(
            x=context["reconstruction"],
            recon_target=recon_target,
            mu=context["reconstruction_mu"],
            log_var=context["reconstruction_log_var"],
            beta=actor_latent_adapter.vae_beta,
        )
        vae_loss = vae_recon_loss + actor_latent_adapter.vae_beta * vae_kl_loss

        non_opt_losses = {
            "vae_recon_loss": vae_recon_loss,
            "vae_kl_loss": vae_kl_loss,
        }

        return {"lin_vel_est_loss": lin_vel_est_loss, "vae_loss": vae_loss}, non_opt_losses


class DreamWaQPPO(ComposablePPO):
    """Composable PPO variant with ``DreamWaQLossSpec`` installed."""

    def __init__(
        self,
        *args,
        lin_vel_est_loss_coef: float = 1.0,
        vae_loss_coef: float = 1.0,
        **kwargs,
    ) -> None:
        """Initialize the variant and expose coefficients for both auxiliary losses."""
        self.lin_vel_est_loss_coef = lin_vel_est_loss_coef
        self.vae_loss_coef = vae_loss_coef
        super().__init__(*args, **kwargs)

    def forward_actor_for_update(self, minibatch: RolloutStorage.Batch) -> None:
        """Run the update actor forward and expose DreamWaQ auxiliary-loss context."""
        _, self.actor_forward_context = self.actor.forward_with_context(
            minibatch.observations,
            masks=minibatch.masks,
            hidden_state=minibatch.hidden_states[0],
            stochastic_output=True,
        )

    @classmethod
    def build_loss_spec(cls, env: VecEnv, algorithm_cfg: dict) -> DreamWaQLossSpec:
        """Build the DreamWaQ loss spec from environment metadata and algorithm config."""
        if not hasattr(env, "obs_format") or not hasattr(env, "obs_group_time_slice_map"):
            raise ValueError(f"`{cls.__name__}` requires `env` to expose `obs_format` and `obs_group_time_slice_map`.")

        lin_vel_target_obs_term_names = algorithm_cfg.pop(
            "lin_vel_target_obs_term_names",
            algorithm_cfg.pop("target_obs_term_names", ["base_lin_vel"]),
        )
        recon_target_obs_term_names = algorithm_cfg.pop(
            "recon_target_obs_term_names",
            ["base_ang_vel", "projected_gravity", "joint_pos", "joint_vel"],
        )

        return DreamWaQLossSpec(
            obs_format=env.obs_format,
            obs_group_time_slice_map=env.obs_group_time_slice_map,
            target_obs_group_name=algorithm_cfg.pop("target_obs_group_name", "critic"),
            recon_target_obs_term_names=recon_target_obs_term_names,
            lin_vel_target_obs_term_names=lin_vel_target_obs_term_names,
        )
