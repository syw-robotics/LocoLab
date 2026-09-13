# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from isaaclab.utils import configclass

from locolab.tasks.manager_based.locomotion.velocity.config.unitree_b2z1.mdp_cfg import (  # isort: skip
    FlatRewardsArmEePosCfg,
    PrivObsArmEePosCfg,
    PropObsArmEePosCfg,
    VelocityEETrajPositionCmdCfg,
)

from .base_velocity_arm_position_flat_env_cfg import B2Z1FlatEnvCfg


@configclass
class B2Z1FlatEETrajObservationsCfg:
    """Observations for B2Z1 flat environment with replayed arm EE trajectory commands."""

    policy: PropObsArmEePosCfg = PropObsArmEePosCfg()
    critic: PrivObsArmEePosCfg = PrivObsArmEePosCfg().replace(height_scan=None)

    policy.history_length = 5


@configclass
class B2Z1FlatEETrajEnvCfg(B2Z1FlatEnvCfg):
    """Flat environment with base velocity and replayed arm EE trajectory commands."""

    observations: B2Z1FlatEETrajObservationsCfg = B2Z1FlatEETrajObservationsCfg()
    commands: VelocityEETrajPositionCmdCfg = VelocityEETrajPositionCmdCfg()
    rewards: FlatRewardsArmEePosCfg = FlatRewardsArmEePosCfg()


@configclass
class B2Z1FlatEETrajEnvCfg_PLAY(B2Z1FlatEETrajEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()

        self.scene.num_envs = 10
        self.scene.env_spacing = 2.5
        self.commands.base_velocity.debug_vis = True
        self.commands.ee_position.debug_vis = True
