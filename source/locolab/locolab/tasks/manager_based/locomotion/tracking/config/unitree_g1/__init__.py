# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from locolab.utils.gym_registration import register_manager_based_rl_env

from . import agents


register_manager_based_rl_env(
    task_id="Tracking-Flat-G1",
    env_cfg_module=f"{__name__}.tracking_flat_env_cfg",
    env_cfg_name="G1TrackingEnvCfg",
    agent_cfg_entry_points= {
        "z_rl_cfg_entry_point": f"{agents.__name__}.z_rl_ppo_cfg:G1TrackingPPORunnerCfg",
    },
)
