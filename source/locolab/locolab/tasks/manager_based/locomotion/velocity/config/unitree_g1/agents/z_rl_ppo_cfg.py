# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from isaaclab.utils import configclass

from z_rl.adaptor.isaaclab import (
    ZRlMLPModelCfg,
    ZRlOnPolicyRunnerCfg,
    ZRlPpoAlgorithmCfg,
    ZRlSymmetryCfg,
)


@configclass
class G1RoughPPOBaseRunnerCfg(ZRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 20000
    save_interval = 500
    experiment_name = "g1_rough"
    obs_groups = {"actor": ["policy"], "critic": ["critic"]}
    algorithm = ZRlPpoAlgorithmCfg(
        num_learning_epochs=5,
        num_mini_batches=4,
        clip_param=0.2,
        gamma=0.99,
        lam=0.95,
        value_loss_coef=1.0,
        #  entropy_coef=0.01,
        entropy_coef=0.005,
        learning_rate=1.0e-3,
        max_grad_norm=1.0,
        optimizer="adamw",
        use_clipped_value_loss=True,
        schedule="adaptive",
        desired_kl=0.01,
        #  symmetry_cfg=ZRlSymmetryCfg(
        #      use_data_augmentation=True,
        #      use_mirror_loss=True,
        #      mirror_loss_coeff=1.0,
        #      data_augmentation_func=(
        #          "locolab.tasks.manager_based.locomotion.velocity.mdp.symmetry.g1:compute_symmetric_states"
        #      ),
        #  ),
    )


@configclass
class G1RoughPPORunnerCfg(G1RoughPPOBaseRunnerCfg):
    actor = ZRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        obs_normalization=False,
        distribution_cfg=ZRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
        init_weights=0.01,  # Use orthogonal init, which helps symmetry learning
    )
    critic = ZRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        obs_normalization=False,
        init_weights=0.01,  # Use orthogonal init, which helps symmetry learning
    )


@configclass
class G1FlatPPORunnerCfg(G1RoughPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.max_iterations = 10000
        self.experiment_name = "g1_flat"
