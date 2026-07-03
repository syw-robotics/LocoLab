# Copyright (c) 2021-2026, ETH Zurich and NVIDIA CORPORATION
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause


from __future__ import annotations

from dataclasses import dataclass

import torch
import torch.nn as nn
from tensordict import TensorDict

from z_rl.models.composition import ComposableModel, LatentSpec
from z_rl.modules import HiddenState
from z_rl_loco_lab.modules import DreamWaqVAE
from z_rl.utils import ObsSelector, resolve_obs_temporal_selector, unpad_trajectories


@dataclass(slots=True)
class DreamWaQEncoderLatentSpec(LatentSpec):
    """Latent spec for a DreamWaQ-style history encoder with VAE and velocity auxiliary losses."""

    vae_latent_dim: int = 32
    vae_encoder_hidden_dims: tuple[int, ...] | list[int] = (256,)
    vae_decoder_hidden_dims: tuple[int, ...] | list[int] = (256,)
    vae_decoder_output_dim: int | None = None
    vae_activation: str = "elu"
    lin_vel_dim: int = 3
    vae_beta: float = 1.0

    def validate(self, model: nn.Module) -> None:
        """Validate actor observation assumptions and cache temporal selectors."""
        if getattr(model, "obs_groups", None) != ["policy"]:
            raise ValueError(
                "`DreamWaQEncoderLatentSpec` requires exactly one active observation group named 'policy'. "
                f"Got {getattr(model, 'obs_groups', None)}."
            )
        if self.vae_latent_dim <= 0:
            raise ValueError(f"`vae_latent_dim` must be positive, got {self.vae_latent_dim}.")
        if self.lin_vel_dim <= 0:
            raise ValueError(f"`lin_vel_dim` must be positive, got {self.lin_vel_dim}.")
        if self.vae_beta < 0.0:
            raise ValueError(f"`vae_beta` must be non-negative, got {self.vae_beta}.")
        if self.vae_decoder_output_dim is not None and self.vae_decoder_output_dim <= 0:
            raise ValueError(
                "`vae_decoder_output_dim` must be positive when provided, "
                f"got {self.vae_decoder_output_dim}."
            )

        # The encoder and decoder use shifted history windows.
        self.exclude_first_obs_selector = resolve_obs_temporal_selector(
            "policy", "exclude_first", model.obs_group_time_slice_map
        )
        self.exclude_last_obs_selector = resolve_obs_temporal_selector(
            "policy", "exclude_last", model.obs_group_time_slice_map
        )
        self.last_obs_selector = resolve_obs_temporal_selector("policy", "last", model.obs_group_time_slice_map)

        if self.exclude_first_obs_selector.dim != self.exclude_last_obs_selector.dim:
            raise ValueError(
                "`exclude_first` and `exclude_last` temporal selectors must have the same dim, got "
                f"{self.exclude_first_obs_selector.dim} and {self.exclude_last_obs_selector.dim}."
            )

    def build_latent_adapter(self, model: nn.Module) -> nn.Module:
        """Build the DreamWaQ VAE adapter."""
        dreamwaq_vae = DreamWaqVAE(
            input_dim=self.exclude_first_obs_selector.dim,
            latent_dim=self.vae_latent_dim,
            extra_dim=self.lin_vel_dim,
            decoder_input_dim=self.vae_latent_dim,
            decoder_output_dim=self.vae_decoder_output_dim,
            encoder_hidden_dims=self.vae_encoder_hidden_dims,
            decoder_hidden_dims=self.vae_decoder_hidden_dims,
            activation=self.vae_activation,
        )

        return _DreamWaQEncoderLatentAdapter(
            obs_group="policy",
            obs_normalizer=model._build_obs_normalizer(model.obs_normalization),
            dreamwaq_vae=dreamwaq_vae,
            lin_vel_dim=self.lin_vel_dim,
            exclude_first_obs_selector=self.exclude_first_obs_selector,
            exclude_last_obs_selector=self.exclude_last_obs_selector,
            last_obs_selector=self.last_obs_selector,
            vae_beta=self.vae_beta,
        )

    def get_latent_dim(self, model: nn.Module) -> int:
        """Return the head input width: velocity estimate, sampled VAE latent, and last observation frame."""
        return self.lin_vel_dim + self.vae_latent_dim + self.last_obs_selector.dim


class _DreamWaQEncoderLatentAdapter(nn.Module):
    """Latent adapter that exposes VAE reconstruction and linear-velocity losses."""

    def __init__(
        self,
        obs_group: str,
        obs_normalizer: nn.Module,
        dreamwaq_vae: DreamWaqVAE,
        lin_vel_dim: int,
        exclude_first_obs_selector: ObsSelector,
        exclude_last_obs_selector: ObsSelector,
        last_obs_selector: ObsSelector,
        vae_beta: float,
    ) -> None:
        super().__init__()
        self.obs_group = obs_group
        self.obs_normalizer = obs_normalizer
        self.dreamwaq_vae = dreamwaq_vae
        self.lin_vel_dim = lin_vel_dim
        self.exclude_first_obs_selector = exclude_first_obs_selector
        self.exclude_last_obs_selector = exclude_last_obs_selector
        self.last_obs_selector = last_obs_selector
        self.vae_beta = vae_beta
        self.reconstruction_dim = dreamwaq_vae.decoder_output_dim

        self.lin_vel_est: torch.Tensor | None = None
        self.reconstruction: torch.Tensor | None = None
        self.reconstruction_mu: torch.Tensor | None = None
        self.reconstruction_log_var: torch.Tensor | None = None
        self.last_vae_recon_loss: torch.Tensor | None = None
        self.last_vae_kl_loss: torch.Tensor | None = None

    def forward_with_context(self, obs: TensorDict) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
        """Build the policy latent and return differentiable tensors required by auxiliary losses."""
        x = self.obs_normalizer(obs[self.obs_group])
        exclude_first_obs = self.exclude_first_obs_selector.select(x)
        exclude_last_obs = self.exclude_last_obs_selector.select(x)
        last_obs = self.last_obs_selector.select(x)

        # Compute the rollout actor latent.
        lin_vel_est, mu, log_var = self.dreamwaq_vae.encode(exclude_first_obs)
        vae_latent = self.dreamwaq_vae.reparameterize(mu, log_var)

        # Compute the shifted reconstruction branch used by the auxiliary VAE loss.
        _, reconstruction_mu, reconstruction_log_var = self.dreamwaq_vae.encode(exclude_last_obs)
        reconstruction_latent = self.dreamwaq_vae.reparameterize(reconstruction_mu, reconstruction_log_var)
        reconstruction = self.dreamwaq_vae.decode(reconstruction_latent)

        context = {
            "lin_vel_est": lin_vel_est,
            "reconstruction": reconstruction,
            "reconstruction_mu": reconstruction_mu,
            "reconstruction_log_var": reconstruction_log_var,
        }
        return torch.cat([lin_vel_est, vae_latent, last_obs], dim=-1), context

    def forward(self, obs: TensorDict) -> torch.Tensor:
        """Build the rollout latent without computing the reconstruction branch."""
        x = self.obs_normalizer(obs[self.obs_group])
        exclude_first_obs = self.exclude_first_obs_selector.select(x)
        last_obs = self.last_obs_selector.select(x)

        lin_vel_est, mu, log_var = self.dreamwaq_vae.encode(exclude_first_obs)
        vae_latent = self.dreamwaq_vae.reparameterize(mu, log_var)

        return torch.cat([lin_vel_est, vae_latent, last_obs], dim=-1)

    def update_normalization(self, obs: TensorDict) -> None:
        """Update running normalization statistics for the full policy observation group."""
        update = getattr(self.obs_normalizer, "update", None)
        if update is not None:
            update(obs[self.obs_group])

    def as_export_module(self) -> nn.Module:
        """Return a tensor-only adapter for ONNX export."""
        return _DreamWaQEncoderLatentAdapterExporter(
            obs_normalizer=self.obs_normalizer,
            dreamwaq_vae=self.dreamwaq_vae,
            exclude_first_obs_selector=self.exclude_first_obs_selector,
            last_obs_selector=self.last_obs_selector,
        )


class _DreamWaQEncoderLatentAdapterExporter(nn.Module):
    """Tensor-only export adapter for DreamWaQ encoder latents."""

    def __init__(
        self,
        obs_normalizer: nn.Module,
        dreamwaq_vae: DreamWaqVAE,
        exclude_first_obs_selector: ObsSelector,
        last_obs_selector: ObsSelector,
    ) -> None:
        super().__init__()
        self.obs_normalizer = obs_normalizer
        self.dreamwaq_vae = dreamwaq_vae
        self.exclude_first_obs_selector = exclude_first_obs_selector
        self.last_obs_selector = last_obs_selector

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Normalize a flat observation tensor and build a deterministic exported actor latent."""
        x = self.obs_normalizer(x)
        exclude_first_obs = _select_for_export(self.exclude_first_obs_selector, x)
        last_obs = _select_for_export(self.last_obs_selector, x)
        lin_vel_est, latent, _ = self.dreamwaq_vae.encode(exclude_first_obs)
        return torch.cat([lin_vel_est, latent, last_obs], dim=-1)


def _select_for_export(selector: ObsSelector, obs: torch.Tensor) -> torch.Tensor:
    """Select observation features while keeping tensor indices on the input device."""
    if isinstance(selector.meta, slice):
        return obs[:, selector.meta]
    return obs.index_select(dim=1, index=selector.meta.to(obs.device))


class DreamWaQEncoderMLPModel(ComposableModel):
    """MLP policy model whose latent is produced by a DreamWaQ-style VAE history encoder."""

    def __init__(
        self,
        obs: TensorDict,
        obs_groups: dict[str, list[str]],
        obs_set: str,
        output_dim: int,
        hidden_dims: tuple[int, ...] | list[int] = (256, 256, 256),
        activation: str = "elu",
        obs_normalization: bool = False,
        distribution_cfg: dict | None = None,
        obs_group_time_slice_map: dict[str, dict[str, ObsSelector]] | None = None,
        vae_latent_dim: int = 128,
        vae_encoder_hidden_dims: tuple[int, ...] | list[int] = (256,),
        vae_decoder_hidden_dims: tuple[int, ...] | list[int] = (256,),
        vae_decoder_output_dim: int = 16,
        vae_activation: str = "elu",
        lin_vel_dim: int = 3,
        vae_beta: float = 1.0,
    ) -> None:
        """Initialize the DreamWaQ encoder-based MLP model."""
        super().__init__(
            obs=obs,
            obs_groups=obs_groups,
            obs_set=obs_set,
            output_dim=output_dim,
            hidden_dims=hidden_dims,
            activation=activation,
            obs_normalization=obs_normalization,
            distribution_cfg=distribution_cfg,
            obs_group_time_slice_map=obs_group_time_slice_map,
            latent_spec=DreamWaQEncoderLatentSpec(
                vae_latent_dim=vae_latent_dim,
                vae_encoder_hidden_dims=vae_encoder_hidden_dims,
                vae_decoder_hidden_dims=vae_decoder_hidden_dims,
                vae_decoder_output_dim=vae_decoder_output_dim,
                vae_activation=vae_activation,
                lin_vel_dim=lin_vel_dim,
                vae_beta=vae_beta,
            ),
        )

    def forward_with_context(
        self,
        obs: TensorDict,
        masks: torch.Tensor | None = None,
        hidden_state: HiddenState = None,
        stochastic_output: bool = False,
    ) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
        """Forward pass used during update when auxiliary DreamWaQ losses need intermediate tensors."""
        del hidden_state
        obs = unpad_trajectories(obs, masks) if masks is not None and not self.is_recurrent else obs
        latent, context = self.latent_adapter.forward_with_context(obs)
        mlp_output = self.head(latent)
        if self.distribution is not None:
            if stochastic_output:
                self.distribution.update(mlp_output)
                return self.distribution.sample(), context
            return self.distribution.deterministic_output(mlp_output), context
        return mlp_output, context
