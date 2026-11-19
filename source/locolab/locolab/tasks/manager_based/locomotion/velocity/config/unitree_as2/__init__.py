# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from locolab.utils.gym_registration import register_manager_based_rl_env

register_manager_based_rl_env(
    task_id="Velocity-Flat-AS2",
    env_cfg_module=f"{__name__}.flat_env_cfg",
    env_cfg_name="AS2FlatEnvCfg",
    agent_cfg_entry_points={
        "rsl_rl_cfg_entry_point": f"{__name__}.agents.rsl_rl_ppo_cfg:AS2FlatPPORunnerCfg",
        "z_rl_cfg_entry_point": f"{__name__}.agents.z_rl_ppo_cfg:AS2FlatPPORunnerCfg",
    },
)
