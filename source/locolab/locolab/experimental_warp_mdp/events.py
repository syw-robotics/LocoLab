"""Fixed encoder zero-offset sampling through the Warp event interface."""

import torch
import warp as wp

from isaaclab.envs.mdp.events import randomize_actuator_gains as _ActuatorGains
from isaaclab.envs.mdp.events import randomize_joint_parameters as _JointParameters
from isaaclab_experimental.envs.mdp.events import _mask_to_env_ids
from isaaclab_experimental.managers import ManagerTermBase, SceneEntityCfg
from isaaclab_experimental.utils.warp import WarpCapturable


@wp.kernel(enable_backward=False)
def _sample_bias(
    mask: wp.array(dtype=bool), rng: wp.array(dtype=wp.uint32), low: float, high: float, bias: wp.array2d(dtype=float)
):
    i = wp.tid()
    if not mask or mask[i]:
        state = rng[i]
        for j in range(bias.shape[1]):
            bias[i, j] = wp.randf(state, low, high)
        rng[i] = state


def randomize_encoder_bias(env, env_mask, bias_range=(-0.01, 0.01), asset_cfg=SceneEntityCfg("robot")):
    wp.launch(
        _sample_bias,
        dim=env.num_envs,
        inputs=[env_mask, env.rng_state_wp, *bias_range, env.scene[asset_cfg.name].encoder_bias_wp],
        device=env.device,
    )


class randomize_actuator_gains(_ActuatorGains, ManagerTermBase):
    _warp_capturable = False

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.asset_cfg.finalize(self.asset.device)
        self._group_joint_indices = {
            name: ids if isinstance(ids, slice) else torch.as_tensor(ids, dtype=torch.long, device=self.asset.device)
            for name, ids in self._group_joint_indices.items()
        }

    @WarpCapturable(False, reason="Startup actuator parameter randomization uses backend model updates")
    def __call__(
        self,
        env,
        env_mask,
        asset_cfg,
        stiffness_distribution_params=None,
        damping_distribution_params=None,
        operation="abs",
        distribution="uniform",
    ):
        super().__call__(
            env,
            _mask_to_env_ids(env_mask),
            asset_cfg,
            stiffness_distribution_params,
            damping_distribution_params,
            operation,
            distribution,
        )

    def reset(self, env_mask=None):
        pass


class randomize_joint_parameters(_JointParameters, ManagerTermBase):
    _warp_capturable = False

    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.asset_cfg.finalize(self.asset.device)

    @WarpCapturable(False, reason="Startup joint parameter randomization uses backend model updates")
    def __call__(
        self,
        env,
        env_mask,
        asset_cfg,
        friction_distribution_params=None,
        armature_distribution_params=None,
        lower_limit_distribution_params=None,
        upper_limit_distribution_params=None,
        operation="abs",
        distribution="uniform",
    ):
        super().__call__(
            env,
            _mask_to_env_ids(env_mask),
            asset_cfg,
            friction_distribution_params,
            armature_distribution_params,
            lower_limit_distribution_params,
            upper_limit_distribution_params,
            operation,
            distribution,
        )

    def reset(self, env_mask=None):
        pass
