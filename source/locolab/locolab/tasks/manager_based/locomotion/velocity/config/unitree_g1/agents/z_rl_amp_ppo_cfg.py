from isaaclab.utils import configclass

from z_rl.adaptor.isaaclab import (
    ZRlAmpPpoAlgorithmCfg,
    ZRlMLPModelCfg,
    ZRlOnPolicyRunnerCfg,
)


@configclass
class G1FlatAMPRunnerCfg(ZRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 10000
    save_interval = 500
    experiment_name = "g1_flat_amp"

    obs_groups = {
        "actor": ["policy"],
        "critic": ["critic"],
        "amp_policy": ["amp_policy"],
        "amp_reference": ["amp_reference"],
    }

    actor = ZRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        obs_normalization=False,
        distribution_cfg=ZRlMLPModelCfg.GaussianDistributionCfg(init_std=1.0),
        init_weights=0.01,
    )

    critic = ZRlMLPModelCfg(
        hidden_dims=[512, 256, 128],
        activation="elu",
        obs_normalization=False,
        init_weights=0.01,
    )

    algorithm = ZRlAmpPpoAlgorithmCfg(
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

        # AMP
        amp_policy_obs_group="amp_policy",
        amp_reference_obs_group="amp_reference",
        amp_reward_coef=0.1,
        amp_task_reward_lerp=0.60,
        amp_loss_coef=1.0,
        amp_grad_penalty_coef=10.0,
        amp_discriminator_hidden_dims=[1024, 512, 256],
        amp_discriminator_activation="relu",
        amp_discriminator_learning_rate=1.0e-3,
        amp_discriminator_optimizer="adam",
    )
