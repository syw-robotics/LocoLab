# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from . import agents
from locolab.utils.gym_registration import register_manager_based_rl_env

##
# Register Gym environments.
##

# ===== Flat terrain =====
register_manager_based_rl_env(
    task_id="Velocity-Flat-B2Z1-EEPosition",
    env_cfg_module=f"{__name__}.base_velocity_arm_position_flat_env_cfg",
    env_cfg_name="B2Z1FlatEEPositionEnvCfg",
    play_env_cfg_name="B2Z1FlatEEPositionEnvCfg_PLAY",
    agent_cfg_entry_points={
        "z_rl_cfg_entry_point": f"{agents.__name__}.z_rl_ppo_cfg:B2Z1FlatEEPositionPPORunnerCfg",
    },
)

register_manager_based_rl_env(
    task_id="Velocity-Flat-B2Z1-EETraj",
    env_cfg_module=f"{__name__}.base_velocity_arm_traj_flat_env_cfg",
    env_cfg_name="B2Z1FlatEETrajEnvCfg",
    play_env_cfg_name="B2Z1FlatEETrajEnvCfg_PLAY",
    agent_cfg_entry_points={
        "z_rl_cfg_entry_point": f"{agents.__name__}.z_rl_ppo_cfg:B2Z1FlatEETrajPPORunnerCfg",
    },
)
