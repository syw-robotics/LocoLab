from __future__ import annotations

from dataclasses import MISSING

from isaaclab.utils import configclass

from z_rl.adaptor.isaaclab.rl_cfg import ZRlMLPModelCfg, ZRlPpoAlgorithmCfg


# BarlowTwins 
@configclass
class BarlowTwinsPpoAlgorithmCfg(ZRlPpoAlgorithmCfg):
    class_name: str = "z_rl_loco_lab.algorithms.barlowtwins_encoder_ppo:BarlowTwinsPPO"
    lin_vel_est_loss_coef: float = 1.0
    barlowtwins_loss_coef: float = 1.0
    target_obs_group_name: str = "critic"
    target_obs_term_names: list[str] = ["base_lin_vel"]


@configclass
class BarlowTwinsEncoderMLPModelCfg(ZRlMLPModelCfg):
    class_name: str = "z_rl_loco_lab.models.barlowtwins_model:BarlowTwinsEncoderMLPModel"
    barlowtwins_encoder_hidden_dims: list[int] = (128, 64)
    barlowtwins_encoder_output_dim: int = 32
    barlowtwins_projector_hidden_dims: list[int] = (32,)
    barlowtwins_projector_output_dim: int = 64
    barlowtwins_latent_head_hidden_dims: list[int] = (16,)
    barlowtwins_latent_dim: int = 16
    barlowtwins_activation: str = "elu"
    lin_vel_dim: int = 3
    barlowtwins_lambda: float = 0.005


# DreamWaQ
@configclass
class DreamWaQPpoAlgorithmCfg(ZRlPpoAlgorithmCfg):
    class_name: str = "z_rl_loco_lab.algorithms.dreamwaq_encoder_ppo:DreamWaQPPO"
    lin_vel_est_loss_coef: float = 1.0
    vae_loss_coef: float = 1.0
    target_obs_group_name: str = "critic"
    recon_target_obs_term_names: list[str] = ["base_ang_vel", "projected_gravity", "joint_pos", "joint_vel"]
    lin_vel_target_obs_term_names: list[str] = ["base_lin_vel"]


@configclass
class DreamWaQEncoderMLPModelCfg(ZRlMLPModelCfg):
    class_name: str = "z_rl_loco_lab.models.dreamwaq_model:DreamWaQEncoderMLPModel"
    vae_latent_dim: int = 16
    vae_encoder_hidden_dims: list[int] = (128, 64)
    vae_decoder_hidden_dims: list[int] = (32,)
    vae_decoder_output_dim: int = MISSING
    vae_activation: str = "elu"
    lin_vel_dim: int = 3
    vae_beta: float = 1.0
