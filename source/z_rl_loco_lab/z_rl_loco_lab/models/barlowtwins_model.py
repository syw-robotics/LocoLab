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

from z_rl.modules import HiddenState, MLP
from z_rl.models.composition import ComposableModel, LatentSpec
from z_rl.utils import ObsSelector, resolve_obs_temporal_selector, unpad_trajectories


@dataclass(slots=True)
class BarlowTwinsEncoderLatentSpec(LatentSpec):
    """Latent spec for a history encoder trained with Barlow Twins and velocity estimation losses."""

    barlowtwins_encoder_hidden_dims: tuple[int, ...] | list[int] = (128, 64)
    barlowtwins_encoder_output_dim: int = 32
    barlowtwins_projector_hidden_dims: tuple[int, ...] | list[int] = (32,)
    barlowtwins_projector_output_dim: int = 64
    barlowtwins_latent_head_hidden_dims: tuple[int, ...] | list[int] = (32,)
    barlowtwins_latent_dim: int = 16
    barlowtwins_activation: str = "elu"
    lin_vel_dim: int = 3
    barlowtwins_lambda: float = 0.005

    def validate(self, model: nn.Module) -> None:
        """Validate actor observation assumptions and cache temporal selectors."""
        if getattr(model, "obs_groups", None) != ["policy"]:
            raise ValueError(
                "`BarlowTwinsEncoderLatentSpec` requires exactly one active observation group named 'policy'. "
                f"Got {getattr(model, 'obs_groups', None)}."
            )
        if self.barlowtwins_encoder_output_dim <= 0:
            raise ValueError(
                f"`barlowtwins_encoder_output_dim` must be positive, got {self.barlowtwins_encoder_output_dim}."
            )
        if self.barlowtwins_latent_dim <= 0:
            raise ValueError(f"`barlowtwins_latent_dim` must be positive, got {self.barlowtwins_latent_dim}.")
        if self.barlowtwins_projector_output_dim <= 0:
            raise ValueError(
                "`barlowtwins_projector_output_dim` must be positive, "
                f"got {self.barlowtwins_projector_output_dim}."
            )
        if self.lin_vel_dim <= 0:
            raise ValueError(f"`lin_vel_dim` must be positive, got {self.lin_vel_dim}.")

        # Adjacent history windows form the two Barlow Twins views.
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
        """Build the Barlow Twins encoder adapter."""
        encoder = MLP(
            self.exclude_first_obs_selector.dim,
            self.barlowtwins_encoder_output_dim,
            self.barlowtwins_encoder_hidden_dims,
            self.barlowtwins_activation,
        )
        latent_head = MLP(
            self.barlowtwins_encoder_output_dim,
            self.barlowtwins_latent_dim,
            self.barlowtwins_latent_head_hidden_dims,
            self.barlowtwins_activation,
        )
        lin_vel_est_head = nn.Linear(self.barlowtwins_encoder_output_dim, self.lin_vel_dim)
        projector = MLP(
            self.barlowtwins_latent_dim,
            self.barlowtwins_projector_output_dim,
            self.barlowtwins_projector_hidden_dims,
            self.barlowtwins_activation,
        )
        return _BarlowTwinsEncoderLatentAdapter(
            obs_group="policy",
            obs_normalizer=model._build_obs_normalizer(model.obs_normalization),
            encoder=encoder,
            latent_head=latent_head,
            lin_vel_est_head=lin_vel_est_head,
            projector=projector,
            exclude_first_obs_selector=self.exclude_first_obs_selector,
            exclude_last_obs_selector=self.exclude_last_obs_selector,
            last_obs_selector=self.last_obs_selector,
            barlowtwins_lambda=self.barlowtwins_lambda,
        )

    def get_latent_dim(self, model: nn.Module) -> int:
        """Return the head input width: velocity estimate, compact latent, and last observation frame."""
        return self.lin_vel_dim + self.barlowtwins_latent_dim + self.last_obs_selector.dim


class _BarlowTwinsEncoderLatentAdapter(nn.Module):
    """Latent adapter that exposes auxiliary Barlow Twins and linear-velocity losses."""

    def __init__(
        self,
        obs_group: str,
        obs_normalizer: nn.Module,
        encoder: nn.Module,
        latent_head: nn.Module,
        lin_vel_est_head: nn.Module,
        projector: nn.Module,
        exclude_first_obs_selector: ObsSelector,
        exclude_last_obs_selector: ObsSelector,
        last_obs_selector: ObsSelector,
        barlowtwins_lambda: float,
    ) -> None:
        super().__init__()
        self.obs_group = obs_group
        self.obs_normalizer = obs_normalizer
        self.encoder = encoder
        self.latent_head = latent_head
        self.lin_vel_est_head = lin_vel_est_head
        self.projector = projector
        self.exclude_first_obs_selector = exclude_first_obs_selector
        self.exclude_last_obs_selector = exclude_last_obs_selector
        self.last_obs_selector = last_obs_selector
        self.barlowtwins_lambda = barlowtwins_lambda
        self.lin_vel_dim = lin_vel_est_head.out_features

        self.lin_vel_est: torch.Tensor | None = None
        self.z1: torch.Tensor | None = None
        self.z2: torch.Tensor | None = None

    def forward_with_context(self, obs: TensorDict) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
        """Build the policy latent and return differentiable tensors required by auxiliary losses."""
        x = self.obs_normalizer(obs[self.obs_group])
        exclude_first_obs = self.exclude_first_obs_selector.select(x)
        exclude_last_obs = self.exclude_last_obs_selector.select(x)
        last_obs = self.last_obs_selector.select(x)

        exclude_first_features = self.encoder(exclude_first_obs)
        exclude_last_features = self.encoder(exclude_last_obs)
        latent = self.latent_head(exclude_first_features)
        next_latent = self.latent_head(exclude_last_features)

        lin_vel_est = self.lin_vel_est_head(exclude_first_features)
        z1 = self.projector(latent)
        z2 = self.projector(next_latent)

        context = {
            "lin_vel_est": lin_vel_est,
            "z1": z1,
            "z2": z2,
        }
        return torch.cat([lin_vel_est, latent, last_obs], dim=-1), context

    def forward(self, obs: TensorDict) -> torch.Tensor:
        """Build the rollout latent without computing the second Barlow Twins view."""
        x = self.obs_normalizer(obs[self.obs_group])
        exclude_first_obs = self.exclude_first_obs_selector.select(x)
        last_obs = self.last_obs_selector.select(x)

        features = self.encoder(exclude_first_obs)
        lin_vel_est = self.lin_vel_est_head(features)
        latent = self.latent_head(features)

        return torch.cat([lin_vel_est, latent, last_obs], dim=-1)

    def compute_barlowtwins_loss(self) -> torch.Tensor:
        """Compute Barlow Twins loss between the cached projected latents."""
        if self.z1 is None or self.z2 is None:
            raise RuntimeError("`forward()` must be called before `compute_barlowtwins_loss()`.")
        return self.compute_barlowtwins_loss_from_projected(self.z1, self.z2)

    def compute_barlowtwins_loss_from_projected(self, z1: torch.Tensor, z2: torch.Tensor) -> torch.Tensor:
        """Compute Barlow Twins loss between two projected latent batches."""
        # Diagonal terms encourage invariance; off-diagonal terms reduce collapse.
        z1_norm = (z1 - z1.mean(dim=0)) / z1.std(dim=0, unbiased=False).clamp_min(1e-6)
        z2_norm = (z2 - z2.mean(dim=0)) / z2.std(dim=0, unbiased=False).clamp_min(1e-6)

        cross_corr = torch.mm(z1_norm.T, z2_norm) / z1_norm.size(0)
        on_diag = torch.diagonal(cross_corr).sub(1.0).pow(2).sum()
        off_diag = _off_diagonal(cross_corr).pow(2).sum()
        return on_diag + self.barlowtwins_lambda * off_diag

    def compute_lin_vel_est_loss(self, lin_vel_target: torch.Tensor) -> torch.Tensor:
        """Compute the linear velocity estimation loss against the selected target observation."""
        if self.lin_vel_est is None:
            raise RuntimeError("`forward()` must be called before `compute_lin_vel_est_loss()`.")
        if self.lin_vel_est.shape[-1] != lin_vel_target.shape[-1]:
            raise ValueError(
                "`lin_vel_target` last dimension must match `lin_vel_est`, got "
                f"{lin_vel_target.shape[-1]} and {self.lin_vel_est.shape[-1]}."
            )
        return F.mse_loss(self.lin_vel_est, lin_vel_target)

    def update_normalization(self, obs: TensorDict) -> None:
        """Update running normalization statistics for the full policy observation group."""
        update = getattr(self.obs_normalizer, "update", None)
        if update is not None:
            update(obs[self.obs_group])

    def as_export_module(self) -> nn.Module:
        """Return a tensor-only adapter for ONNX export."""
        return _BarlowTwinsEncoderLatentAdapterExporter(
            obs_normalizer=self.obs_normalizer,
            encoder=self.encoder,
            latent_head=self.latent_head,
            lin_vel_est_head=self.lin_vel_est_head,
            exclude_first_obs_selector=self.exclude_first_obs_selector,
            last_obs_selector=self.last_obs_selector,
        )


def _off_diagonal(x: torch.Tensor) -> torch.Tensor:
    """Return a flattened view of the off-diagonal entries of a square matrix."""
    n, m = x.shape
    if n != m:
        raise ValueError(f"`_off_diagonal` expects a square matrix, got {x.shape}.")
    return x.flatten()[:-1].view(n - 1, n + 1)[:, 1:].flatten()


class _BarlowTwinsEncoderLatentAdapterExporter(nn.Module):
    """Tensor-only export adapter for Barlow Twins encoder latents."""

    def __init__(
        self,
        obs_normalizer: nn.Module,
        encoder: nn.Module,
        latent_head: nn.Module,
        lin_vel_est_head: nn.Module,
        exclude_first_obs_selector: ObsSelector,
        last_obs_selector: ObsSelector,
    ) -> None:
        super().__init__()
        self.obs_normalizer = obs_normalizer
        self.encoder = encoder
        self.latent_head = latent_head
        self.lin_vel_est_head = lin_vel_est_head
        self.exclude_first_obs_selector = exclude_first_obs_selector
        self.last_obs_selector = last_obs_selector

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Normalize a flat observation tensor and build the exported actor latent."""
        x = self.obs_normalizer(x)
        exclude_first_obs = self.exclude_first_obs_selector.select(x)
        last_obs = self.last_obs_selector.select(x)
        features = self.encoder(exclude_first_obs)
        lin_vel_est = self.lin_vel_est_head(features)
        latent = self.latent_head(features)
        return torch.cat([lin_vel_est, latent, last_obs], dim=-1)


class BarlowTwinsEncoderMLPModel(ComposableModel):
    """MLP policy model whose latent is produced by a Barlow Twins history encoder."""

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
        barlowtwins_encoder_hidden_dims: tuple[int, ...] | list[int] = (128, 64),
        barlowtwins_encoder_output_dim: int = 32,
        barlowtwins_projector_hidden_dims: tuple[int, ...] | list[int] = (32,),
        barlowtwins_projector_output_dim: int = 64,
        barlowtwins_latent_head_hidden_dims: tuple[int, ...] | list[int] = (32,),
        barlowtwins_latent_dim: int = 16,
        barlowtwins_activation: str = "elu",
        lin_vel_dim: int = 3,
        barlowtwins_lambda: float = 0.005,
    ) -> None:
        """Initialize the Barlow Twins encoder-based MLP model."""
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
            latent_spec=BarlowTwinsEncoderLatentSpec(
                barlowtwins_encoder_hidden_dims=barlowtwins_encoder_hidden_dims,
                barlowtwins_encoder_output_dim=barlowtwins_encoder_output_dim,
                barlowtwins_projector_hidden_dims=barlowtwins_projector_hidden_dims,
                barlowtwins_projector_output_dim=barlowtwins_projector_output_dim,
                barlowtwins_latent_head_hidden_dims=barlowtwins_latent_head_hidden_dims,
                barlowtwins_latent_dim=barlowtwins_latent_dim,
                barlowtwins_activation=barlowtwins_activation,
                lin_vel_dim=lin_vel_dim,
                barlowtwins_lambda=barlowtwins_lambda,
            ),
        )

    def forward_with_context(
        self,
        obs: TensorDict,
        masks: torch.Tensor | None = None,
        hidden_state: HiddenState = None,
        stochastic_output: bool = False,
    ) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
        """Forward pass used during update when auxiliary Barlow Twins losses need intermediate tensors."""
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

