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
from locolab.tasks.manager_based.locomotion.velocity.config.unitree_go2.mdp_cfg import (  # isort: skip
    ActionsCfg,
    ActionsCfg_W_Symmetry,
    CommandsCfg,
    EventCfg,
    FlatRewardsCfg,
    PrivObsCfg,
    PropObsCfg,
    PropObsCfg_W_Symmetry,
    PrivObsCfg_W_Symmetry,
    FlatTerminationsCfg,
    FLAT_CONTACT_SENSOR_LINK_NAMES,
)
from locolab.assets import UNITREE_GO2_CFG  # isort: skip
from locolab.utils.terrains.terrains_cfg import FLAT_ROUGH_TERRAINS_CFG  # isort: skip


##
# MDP definition
##
@configclass
class Go2FlatObservationsCfg:
    """Configuration for Go2 on flat terrain observations"""

    # policy: PropObsCfg = PropObsCfg()
    # critic: PrivObsCfg = PrivObsCfg().replace(height_scan=None)
    policy: PropObsCfg_W_Symmetry = PropObsCfg_W_Symmetry()
    critic: PrivObsCfg_W_Symmetry = PrivObsCfg_W_Symmetry().replace(height_scan=None)

    policy.history_length = 5


##
# Scene definition
##
@configclass
class Go2FlatSceneCfg(InteractiveSceneCfg):
    """Configuration for Go2 on flat terrain scene"""

    # =====  terrain  =====
    terrain: TerrainImporterCfg = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=FLAT_ROUGH_TERRAINS_CFG.replace(
            sub_terrains={
                "flat_rough": FLAT_ROUGH_TERRAINS_CFG.sub_terrains["flat_rough"].replace(
                    noise_range=(-0.05, 0.05),
                ),
            },
        ),
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
    robot: ArticulationCfg = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # =====  sensors  =====
    contact_forces: ContactSensorCfg = ContactSensorCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{FLAT_CONTACT_SENSOR_LINK_NAMES}", history_length=3, track_air_time=True
    )

    # =====  lights  =====
    sky_light: AssetBaseCfg = blue_sky_light_cfg()


##
# Environment configuration
##
@configclass
class Go2FlatEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the Go2 flat environment."""

    # Scene settings
    scene: Go2FlatSceneCfg = Go2FlatSceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    observations: Go2FlatObservationsCfg = Go2FlatObservationsCfg()
    actions: ActionsCfg_W_Symmetry = ActionsCfg_W_Symmetry()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: FlatRewardsCfg = FlatRewardsCfg()
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
        # disable self collisions for flat terrain
        self.scene.robot.spawn.articulation_props.enabled_self_collisions = False

@configclass
class Go2FlatEnvCfg_PLAY(Go2FlatEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 10
        self.scene.env_spacing = 2.5
        self.commands.base_velocity.debug_vis = True
