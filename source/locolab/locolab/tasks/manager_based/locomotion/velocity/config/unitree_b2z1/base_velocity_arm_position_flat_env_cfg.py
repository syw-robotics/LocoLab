# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass

from locolab.utils.scene import flat_rough_terrain_visual_material_cfg, blue_sky_light_cfg
from locolab.utils.terrains import TerrainImporterCfg

#  from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR


##
# Pre-defined configs
##
from locolab.tasks.manager_based.locomotion.velocity.config.unitree_b2z1.mdp_cfg import (  # isort: skip
    ActionsCfg,
    VelocityEEPositionCmdCfg,
    EventCfg,
    FlatRewardsArmEePosCfg,
    PrivObsArmEePosCfg,
    PropObsArmEePosCfg,
    FlatTerminationsCfg,
    CONTACT_SENSOR_LINK_NAMES,
)
from locolab.assets import UNITREE_B2Z1_CFG  # isort: skip
from locolab.utils.terrains.terrains_cfg import FLAT_ROUGH_TERRAINS_CFG  # isort: skip


##
# MDP definition
##
@configclass
class B2Z1FlatEEPositionObservationsCfg:
    """Observations for B2Z1 flat environment with arm EE position commands."""

    policy: PropObsArmEePosCfg = PropObsArmEePosCfg()
    critic: PrivObsArmEePosCfg = PrivObsArmEePosCfg().replace(height_scan=None)

    policy.history_length = 5


##
# Scene definition
##
@configclass
class B2Z1FlatSceneCfg(InteractiveSceneCfg):
    """Configuration for B2Z1 flat environment on flat terrain scene."""

    # =====  terrain  =====
    terrain: TerrainImporterCfg = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=FLAT_ROUGH_TERRAINS_CFG,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=flat_rough_terrain_visual_material_cfg(),
        debug_vis=False,
    )

    # =====  robots  =====
    robot: ArticulationCfg = UNITREE_B2Z1_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # =====  sensors  =====
    contact_forces: ContactSensorCfg = ContactSensorCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{CONTACT_SENSOR_LINK_NAMES}", history_length=3, track_air_time=True
    )

    # =====  lights  =====
    sky_light: AssetBaseCfg = blue_sky_light_cfg()


##
# Environment configuration
##
@configclass
class B2Z1FlatEnvCfg(ManagerBasedRLEnvCfg):
    """Shared flat environment configuration."""

    # Scene settings
    scene: B2Z1FlatSceneCfg = B2Z1FlatSceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    actions: ActionsCfg = ActionsCfg()
    # MDP settings
    terminations: FlatTerminationsCfg = FlatTerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 4
        self.episode_length_s = 20.0
        # simulation settings
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15
        # update sensor update periods
        # we tick all the sensors based on the smallest update period (physics update period)
        self.scene.contact_forces.update_period = self.sim.dt


@configclass
class B2Z1FlatEEPositionEnvCfg(B2Z1FlatEnvCfg):
    """Flat environment with base velocity and arm EE position commands."""

    observations: B2Z1FlatEEPositionObservationsCfg = B2Z1FlatEEPositionObservationsCfg()
    commands: VelocityEEPositionCmdCfg = VelocityEEPositionCmdCfg()
    rewards: FlatRewardsArmEePosCfg = FlatRewardsArmEePosCfg()



@configclass
class B2Z1FlatEEPositionEnvCfg_PLAY(B2Z1FlatEEPositionEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()

        self.scene.num_envs = 10
        self.scene.env_spacing = 2.5
        self.commands.base_velocity.debug_vis = True
        self.commands.ee_position.debug_vis = True
