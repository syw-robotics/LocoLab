"""Fixed encoder zero-offset sampling through the Warp event interface."""

import torch
import warp as wp

from isaaclab.envs.mdp.events import randomize_actuator_gains as _ActuatorGains
from isaaclab.envs.mdp.events import randomize_joint_parameters as _JointParameters
from isaaclab_experimental.envs.mdp.events import _mask_to_env_ids
from isaaclab_experimental.envs.mdp.events import push_by_setting_velocity as _push_by_setting_velocity
from isaaclab_experimental.managers import ManagerTermBase, SceneEntityCfg
from isaaclab_experimental.utils.warp import WarpCapturable


@WarpCapturable(False, reason="Push visualization records transient state outside CUDA graphs")
def push_by_setting_velocity(
    env, env_mask, velocity_range, asset_cfg=SceneEntityCfg("robot"), debug_vis=False, debug_vis_cfg=None
):
    """Use the Warp push unchanged, recording its applied increment only for debug display."""
    if not debug_vis:
        _push_by_setting_velocity(env, env_mask, velocity_range, asset_cfg)
        return

    from locolab.utils.debug_viz.push.push_debug_vis import get_push_debug_vis

    env_ids = _mask_to_env_ids(env_mask)
    before = env.scene[asset_cfg.name].data.root_vel_w.torch[env_ids].clone()
    _push_by_setting_velocity(env, env_mask, velocity_range, asset_cfg)
    after = wp.to_torch(_push_by_setting_velocity._scratch_vel)[env_ids]
    get_push_debug_vis(env, asset_cfg.name, debug_vis_cfg).record(env_ids, after - before)


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
