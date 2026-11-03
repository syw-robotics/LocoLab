"""Z-RL metadata for histories implemented inside Warp observation terms."""

from z_rl.adaptor.isaaclab import ZRlVecEnvWrapper


class ZRlWarpVecEnvWrapper(ZRlVecEnvWrapper):
    def _create_obs_format(self):
        result = super()._create_obs_format()
        manager = self.unwrapped.observation_manager
        for group, names in manager.active_terms.items():
            for name, cfg in zip(names, manager._group_obs_term_cfgs[group]):
                if hasattr(cfg, "_warp_history_length"):
                    result[group][name] = (cfg._warp_history_length, cfg._warp_frame_dim)
        return result
