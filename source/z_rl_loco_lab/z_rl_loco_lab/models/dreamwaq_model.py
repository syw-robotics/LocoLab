# Copyright (c) 2021-2026, ETH Zurich and NVIDIA CORPORATION
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause


from __future__ import annotations

from dataclasses import dataclass

import torch
import torch.nn as nn
import torch.nn.functional as F
from tensordict import TensorDict

from z_rl.models.composition import ComposableModel, LatentSpec
from z_rl.modules import HiddenState, VAE
from z_rl.utils import ObsSelector, resolve_obs_temporal_selector, unpad_trajectories


@dataclass(slots=True)
class DreamWaQEncoderLatentSpec(LatentSpec):
    """Latent spec for a DreamWaQ-style history encoder with VAE and velocity auxiliary losses."""

    latent_dim: int = 128
    dreamwaq_encoder_hidden_dims: tuple[int, ...] | list[int] = (256,)
    dreamwaq_decoder_hidden_dims: tuple[int, ...] | list[int] = (256,)
    dreamwaq_decoder_output_dim: int | None = None
    dreamwaq_vae_activation: str = "elu"
    lin_vel_dim: int = 3
    vae_beta: float = 1.0

    def validate(self, model: nn.Module) -> None:
        """Validate actor observation assumptions and cache temporal selectors."""
        if getattr(model, "obs_groups", None) != ["policy"]:
            raise ValueError(
                "`DreamWaQEncoderLatentSpec` requires exactly one active observation group named 'policy'. "
                f"Got {getattr(model, 'obs_groups', None)}."
            )
        if self.latent_dim <= 0:
            raise ValueError(f"`latent_dim` must be positive, got {self.latent_dim}.")
        if self.lin_vel_dim <= 0:
            raise ValueError(f"`lin_vel_dim` must be positive, got {self.lin_vel_dim}.")
        if self.vae_beta < 0.0:
            raise ValueError(f"`vae_beta` must be non-negative, got {self.vae_beta}.")
        if self.dreamwaq_decoder_output_dim is not None and self.dreamwaq_decoder_output_dim <= 0:
            raise ValueError(
                "`dreamwaq_decoder_output_dim` must be positive when provided, "
                f"got {self.dreamwaq_decoder_output_dim}."
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
        decoder_output_dim = self.dreamwaq_decoder_output_dim or self.exclude_last_obs_selector.dim
        dreamwaq_vae = VAE(
            input_dim=self.exclude_first_obs_selector.dim,
            latent_dim=self.latent_dim,
            decoder_output_dim=decoder_output_dim,
            encoder_hidden_dims=self.dreamwaq_encoder_hidden_dims,
            decoder_hidden_dims=self.dreamwaq_decoder_hidden_dims,
            activation=self.dreamwaq_vae_activation,
        )
        lin_vel_est_head = nn.Linear(self.latent_dim, self.lin_vel_dim)

        return _DreamWaQEncoderLatentAdapter(
            obs_group="policy",
            obs_normalizer=model._build_obs_normalizer(model.obs_normalization),
            vae=dreamwaq_vae,
            lin_vel_est_head=lin_vel_est_head,
            exclude_first_obs_selector=self.exclude_first_obs_selector,
            exclude_last_obs_selector=self.exclude_last_obs_selector,
            last_obs_selector=self.last_obs_selector,
            vae_beta=self.vae_beta,
        )

    def get_latent_dim(self, model: nn.Module) -> int:
        """Return the head input width: velocity estimate, sampled VAE latent, and last observation frame."""
        return self.lin_vel_dim + self.latent_dim + self.last_obs_selector.dim


class _DreamWaQEncoderLatentAdapter(nn.Module):
    """Latent adapter that exposes VAE reconstruction and linear-velocity losses."""

    def __init__(
        self,
        obs_group: str,
        obs_normalizer: nn.Module,
        vae: VAE,
        lin_vel_est_head: nn.Module,
        exclude_first_obs_selector: ObsSelector,
        exclude_last_obs_selector: ObsSelector,
        last_obs_selector: ObsSelector,
        vae_beta: float,
    ) -> None:
        super().__init__()
        self.obs_group = obs_group
        self.obs_normalizer = obs_normalizer
        self.vae = vae
        self.lin_vel_est_head = lin_vel_est_head
        self.lin_vel_dim = lin_vel_est_head.out_features
        self.exclude_first_obs_selector = exclude_first_obs_selector
        self.exclude_last_obs_selector = exclude_last_obs_selector
        self.last_obs_selector = last_obs_selector
        self.vae_beta = vae_beta
        self.reconstruction_dim = vae.decoder_output_dim

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

        mu, log_var = self.vae.encode(exclude_first_obs)
        latent = self.vae.reparameterize(mu, log_var)
        lin_vel_est = self.lin_vel_est_head(latent)

        reconstruction_mu, reconstruction_log_var = self.vae.encode(exclude_last_obs)
        reconstruction_latent = self.vae.reparameterize(reconstruction_mu, reconstruction_log_var)
        reconstruction = self.vae.decode(reconstruction_latent)

        context = {
            "lin_vel_est": lin_vel_est,
            "reconstruction": reconstruction,
            "reconstruction_mu": reconstruction_mu,
            "reconstruction_log_var": reconstruction_log_var,
        }
        return torch.cat([lin_vel_est, latent, last_obs], dim=-1), context

    def forward(self, obs: TensorDict) -> torch.Tensor:
        """Build the rollout latent without computing the reconstruction branch."""
        x = self.obs_normalizer(obs[self.obs_group])
        exclude_first_obs = self.exclude_first_obs_selector.select(x)
        last_obs = self.last_obs_selector.select(x)

        mu, log_var = self.vae.encode(exclude_first_obs)
        latent = self.vae.reparameterize(mu, log_var)
        lin_vel_est = self.lin_vel_est_head(latent)

        return torch.cat([lin_vel_est, latent, last_obs], dim=-1)

    def update_normalization(self, obs: TensorDict) -> None:
        """Update running normalization statistics for the full policy observation group."""
        update = getattr(self.obs_normalizer, "update", None)
        if update is not None:
            update(obs[self.obs_group])

    def as_export_module(self) -> nn.Module:
        """Return a tensor-only adapter for ONNX export."""
        return _DreamWaQEncoderLatentAdapterExporter(
            obs_normalizer=self.obs_normalizer,
            vae=self.vae,
            lin_vel_est_head=self.lin_vel_est_head,
            exclude_first_obs_selector=self.exclude_first_obs_selector,
            last_obs_selector=self.last_obs_selector,
        )


class _DreamWaQEncoderLatentAdapterExporter(nn.Module):
    """Tensor-only export adapter for DreamWaQ encoder latents."""

    def __init__(
        self,
        obs_normalizer: nn.Module,
        vae: VAE,
        lin_vel_est_head: nn.Module,
        exclude_first_obs_selector: ObsSelector,
        last_obs_selector: ObsSelector,
    ) -> None:
        super().__init__()
        self.obs_normalizer = obs_normalizer
        self.vae = vae
        self.lin_vel_est_head = lin_vel_est_head
        self.exclude_first_obs_selector = exclude_first_obs_selector
        self.last_obs_selector = last_obs_selector

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Normalize a flat observation tensor and build a deterministic exported actor latent."""
        x = self.obs_normalizer(x)
        exclude_first_obs = _select_for_export(self.exclude_first_obs_selector, x)
        last_obs = _select_for_export(self.last_obs_selector, x)
        latent, _ = self.vae.encode(exclude_first_obs)
        lin_vel_est = self.lin_vel_est_head(latent)
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
        latent_dim: int = 128,
        dreamwaq_encoder_hidden_dims: tuple[int, ...] | list[int] = (256,),
        dreamwaq_decoder_hidden_dims: tuple[int, ...] | list[int] = (256,),
        dreamwaq_decoder_output_dim: int | None = None,
        dreamwaq_vae_activation: str = "elu",
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
                latent_dim=latent_dim,
                dreamwaq_encoder_hidden_dims=dreamwaq_encoder_hidden_dims,
                dreamwaq_decoder_hidden_dims=dreamwaq_decoder_hidden_dims,
                dreamwaq_decoder_output_dim=dreamwaq_decoder_output_dim,
                dreamwaq_vae_activation=dreamwaq_vae_activation,
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
