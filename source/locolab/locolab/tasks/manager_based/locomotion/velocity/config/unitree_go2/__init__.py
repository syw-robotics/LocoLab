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
    task_id="Velocity-Flat-Go2",
    env_cfg_module=f"{__name__}.flat_env_cfg",
    env_cfg_name="Go2FlatEnvCfg",
    agent_cfg_entry_points={
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2FlatPPORunnerCfg",
        "z_rl_cfg_entry_point": f"{agents.__name__}.z_rl_ppo_cfg:Go2FlatPPORunnerCfg",
    },
)

# -- test additional z_rl tasks --
register_manager_based_rl_env(
    task_id="Velocity-Flat-Go2-ZRL-MoE",
    env_cfg_module=f"{__name__}.flat_env_cfg",
    env_cfg_name="Go2FlatEnvCfg",
    agent_cfg_entry_points={
        "z_rl_cfg_entry_point": f"{agents.__name__}.z_rl_ppo_cfg:Go2FlatMoEPPORunnerCfg",
    },
)

register_manager_based_rl_env(
    task_id="Velocity-Flat-Go2-ZRL-RNN",
    env_cfg_module=f"{__name__}.flat_env_cfg",
    env_cfg_name="Go2FlatEnvCfg",
    agent_cfg_entry_points={
        "z_rl_cfg_entry_point": f"{agents.__name__}.z_rl_ppo_cfg:Go2FlatRNNPPORunnerCfg",
    },
)

register_manager_based_rl_env(
    task_id="Velocity-Flat-Go2-ZRL-EncoderEstimation",
    env_cfg_module=f"{__name__}.flat_env_cfg",
    env_cfg_name="Go2FlatEnvCfg",
    agent_cfg_entry_points={
        "z_rl_cfg_entry_point": f"{agents.__name__}.z_rl_ppo_cfg:Go2FlatEncoderEstimationPPORunnerCfg",
    },
)


# ===== Rough terrain =====
register_manager_based_rl_env(
    task_id="Velocity-Rough-Go2",
    env_cfg_module=f"{__name__}.rough_env_cfg",
    env_cfg_name="Go2RoughEnvCfg",
    agent_cfg_entry_points={
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:Go2RoughPPORunnerCfg",
        "z_rl_cfg_entry_point": f"{agents.__name__}.z_rl_ppo_cfg:Go2RoughPPORunnerCfg",
    },
)

register_manager_based_rl_env(
    task_id="Velocity-Rough-Go2-ZRL-DreamWaQ",
    env_cfg_module=f"{__name__}.rough_env_cfg",
    env_cfg_name="Go2RoughEnvCfg",
    agent_cfg_entry_points={
        "z_rl_cfg_entry_point": f"{agents.__name__}.z_rl_dreamwaq_ppo_cfg:Go2RoughDreamWaQPPORunnerCfg",
    },
)

register_manager_based_rl_env(
    task_id="Velocity-Rough-Go2-ZRL-BarlowTwins",
    env_cfg_module=f"{__name__}.rough_env_cfg",
    env_cfg_name="Go2RoughEnvCfg",
    agent_cfg_entry_points={
        "z_rl_cfg_entry_point": f"{agents.__name__}.z_rl_barlowtwins_ppo_cfg:Go2RoughBarlowTwinsPPORunnerCfg",
    },
)
