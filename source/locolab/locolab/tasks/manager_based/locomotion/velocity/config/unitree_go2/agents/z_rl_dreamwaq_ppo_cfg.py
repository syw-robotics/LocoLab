# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from isaaclab.utils import configclass

from z_rl.adaptor.isaaclab import ZRlMLPModelCfg
from z_rl_loco_lab.rl_cfg import DreamWaQEncoderMLPModelCfg, DreamWaQPpoAlgorithmCfg

from .z_rl_ppo_cfg import Go2RoughPPOBaseRunnerCfg


@configclass
class Go2RoughDreamWaQPPORunnerCfg(Go2RoughPPOBaseRunnerCfg):
    experiment_name = "go2_rough_dreamwaq"
    obs_group_concat_mode = "history_major"
    actor = DreamWaQEncoderMLPModelCfg(
        hidden_dims=[256, 128],
        activation="elu",
        obs_normalization=False,
        distribution_cfg=ZRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
        latent_dim=16,
        dreamwaq_encoder_hidden_dims=[128, 64],
        dreamwaq_decoder_hidden_dims=[32,],
        dreamwaq_decoder_output_dim=33,
        dreamwaq_vae_activation="elu",
        lin_vel_dim=3,
        vae_beta=1.0,
    )
    critic = ZRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        obs_normalization=False,
    )
    algorithm = DreamWaQPpoAlgorithmCfg(
        num_learning_epochs=5,
        num_mini_batches=4,
        clip_param=0.2,
        gamma=0.99,
        lam=0.95,
        value_loss_coef=1.0,
        entropy_coef=0.01,
        learning_rate=1.0e-3,
        max_grad_norm=1.0,
        optimizer="adamw",
        use_clipped_value_loss=True,
        schedule="adaptive",
        desired_kl=0.01,
        lin_vel_est_loss_coef=1.0,
        vae_loss_coef=1.0,
        target_obs_group_name="critic",
        recon_target_obs_term_names=["base_lin_vel", "base_ang_vel", "projected_gravity", "joint_pos", "joint_vel"],
        lin_vel_target_obs_term_names=["base_lin_vel"],
    )
