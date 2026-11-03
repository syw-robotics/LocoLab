# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from z_rl.adaptor.isaaclab import (
    ZRlMLPModelCfg,
    ZRlOnPolicyRunnerCfg,
    ZRlPpoAlgorithmCfg,
)

from isaaclab.utils import configclass


@configclass
class G1TrackingPPORunnerCfg(ZRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 10_000
    save_interval = 500
    experiment_name = "g1_flat_tracking"
    obs_groups = {"actor": ["policy"], "critic": ["critic"]}
    actor = ZRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        obs_normalization=True,
        distribution_cfg=ZRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
    )
    critic = ZRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        obs_normalization=True,
    )
    algorithm = ZRlPpoAlgorithmCfg(
        num_learning_epochs=5,
        num_mini_batches=4,
        clip_param=0.2,
        gamma=0.99,
        lam=0.95,
        value_loss_coef=1.0,
        entropy_coef=0.005,
        learning_rate=1.0e-3,
        max_grad_norm=1.0,
        optimizer="adamw",
        use_muon=True,  # using muon seems to accelerate learning convergence
        use_clipped_value_loss=True,
        schedule="adaptive",
        desired_kl=0.01,
        # disable symmetry augmentation as the single motion clip tracking is not symmetric
        symmetry_augmentation=False,
    )
