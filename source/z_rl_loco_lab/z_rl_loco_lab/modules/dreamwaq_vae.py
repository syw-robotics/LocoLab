from __future__ import annotations

import torch

from z_rl.modules import VAE
from z_rl.modules.mlp import MLP


class DreamWaqVAE(VAE):
    """VAE encoder that also predicts a deterministic auxiliary variable."""

    def __init__(
        self,
        input_dim: int,
        latent_dim: int,
        extra_dim: int,
        decoder_input_dim: int | None = None,
        decoder_output_dim: int | None = None,
        encoder_hidden_dims: tuple[int, ...] | list[int] = (256, 256),
        decoder_hidden_dims: tuple[int, ...] | list[int] = (256, 256),
        activation: str = "elu",
    ) -> None:
        if extra_dim <= 0:
            raise ValueError(f"`extra_dim` must be positive, got {extra_dim}.")
        if decoder_input_dim is not None and decoder_input_dim > latent_dim:
            raise ValueError(
                f"`decoder_input_dim` can not exceed `latent_dim`, got {decoder_input_dim} > {latent_dim}."
            )

        super().__init__(
            input_dim=input_dim,
            latent_dim=latent_dim,
            decoder_input_dim=decoder_input_dim,
            decoder_output_dim=decoder_output_dim,
            encoder_hidden_dims=encoder_hidden_dims,
            decoder_hidden_dims=decoder_hidden_dims,
            activation=activation,
        )
        self.extra_dim = extra_dim

        # The base VAE builds a standard encoder; DreamWaQ adds a deterministic auxiliary output.
        self.encoder = MLP(input_dim, 2 * latent_dim + extra_dim, encoder_hidden_dims, activation)

    def encode(self, x: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Encode input into an auxiliary estimate and Gaussian posterior parameters."""
        enc_out = self.encoder(x)
        extra, mu, log_var = torch.split(enc_out, [self.extra_dim, self.latent_dim, self.latent_dim], dim=-1)
        return extra, mu, log_var

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        raise NotImplementedError("DreamWaqVAE does not implement a forward method. Use `encode` and `decode` instead.")
