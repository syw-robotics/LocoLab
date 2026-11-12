# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from locolab.utils.gym_registration import register_manager_based_rl_env

from . import agents

##
# Register Gym environments.
##

_AGENT_CFG_ENTRY_POINTS = {
    "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:G1FlatPPORunnerCfg",
    "z_rl_cfg_entry_point": f"{agents.__name__}.z_rl_ppo_cfg:G1FlatPPORunnerCfg",
}

# ===== Flat terrain =====
register_manager_based_rl_env(
    task_id="Velocity-Flat-G1",
    env_cfg_module=f"{__name__}.flat_env_cfg",
    env_cfg_name="G1FlatEnvCfg",
    agent_cfg_entry_points=_AGENT_CFG_ENTRY_POINTS,
)

