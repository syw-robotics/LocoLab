# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from isaaclab.utils import configclass

from z_rl.adaptor.isaaclab import (
    ZRlEncoderEstimationPpoAlgorithmCfg,
    ZRlEncoderMLPModelCfg,
    ZRlMLPModelCfg,
    ZRlMoEModelCfg,
    ZRlOnPolicyRunnerCfg,
    ZRlPpoAlgorithmCfg,
    ZRlRNNModelCfg,
)


@configclass
class ZRlCompileCfg:
    enabled = True

@configclass
class Go2RoughPPOBaseRunnerCfg(ZRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 10000
    save_interval = 100
    experiment_name = "go2_rough"
    obs_groups = {"actor": ["policy"], "critic": ["critic"]}
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
        use_clipped_value_loss=True,
        schedule="adaptive",
        desired_kl=0.01,
    )
    compile_cfg = ZRlCompileCfg()


@configclass
class Go2RoughPPORunnerCfg(Go2RoughPPOBaseRunnerCfg):
    actor = ZRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        obs_normalization=False,
        distribution_cfg=ZRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
    )
    critic = ZRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        obs_normalization=False,
    )


@configclass
class Go2RoughMoEPPORunnerCfg(Go2RoughPPOBaseRunnerCfg):
    actor = ZRlMoEModelCfg(
        activation="elu",
        obs_normalization=False,
        distribution_cfg=ZRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
        num_experts=4,
        expert_hidden_dims=[256, 128],
        gate_hidden_dims=[256, 128],
    )
    critic = ZRlMoEModelCfg(
        activation="elu",
        obs_normalization=False,
        num_experts=4,
        expert_hidden_dims=[256, 128],
        gate_hidden_dims=[256, 128],
    )


@configclass
class Go2RoughRNNPPORunnerCfg(Go2RoughPPOBaseRunnerCfg):
    actor = ZRlRNNModelCfg(
        hidden_dims=[256, 128],
        activation="elu",
        obs_normalization=False,
        distribution_cfg=ZRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
        rnn_type="lstm",
        rnn_hidden_dim=128,
        rnn_num_layers=1,
    )
    critic = ZRlRNNModelCfg(
        hidden_dims=[256, 128],
        activation="elu",
        obs_normalization=False,
        rnn_type="lstm",
        rnn_hidden_dim=128,
        rnn_num_layers=1,
    )


@configclass
class Go2RoughEncoderEstimationPPORunnerCfg(Go2RoughPPOBaseRunnerCfg):
    actor = ZRlEncoderMLPModelCfg(
        hidden_dims=[256, 128],
        activation="elu",
        obs_normalization=False,
        distribution_cfg=ZRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
        latent_dim=64,
        encoder_hidden_dims=[256, 128],
        encoder_activation="elu",
        concat_last_obs=False,
    )
    critic = ZRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        obs_normalization=False,
    )
    algorithm = ZRlEncoderEstimationPpoAlgorithmCfg(
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
        use_clipped_value_loss=True,
        schedule="adaptive",
        desired_kl=0.01,
        estimation_loss_coef=1.0,
        target_obs_group_name="critic",
        target_obs_term_names=["base_lin_vel"],
    )


@configclass
class Go2FlatPPORunnerCfg(Go2RoughPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.max_iterations = 2000
        self.experiment_name = "go2_flat"


@configclass
class Go2FlatMoEPPORunnerCfg(Go2RoughMoEPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.max_iterations = 2000
        self.experiment_name = "go2_flat_moe"


@configclass
class Go2FlatRNNPPORunnerCfg(Go2RoughRNNPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.max_iterations = 2000
        self.experiment_name = "go2_flat_rnn"


@configclass
class Go2FlatEncoderEstimationPPORunnerCfg(Go2RoughEncoderEstimationPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.max_iterations = 2000
        self.experiment_name = "go2_flat_encoder_estimation"
