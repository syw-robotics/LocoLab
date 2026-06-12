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
from isaaclab.sensors import ContactSensorCfg, RayCasterCfg, patterns
from isaaclab.utils import configclass

#  from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from locolab.utils.scene import rough_terrain_visual_material_cfg, blue_sky_light_cfg
from locolab.utils.terrains import TerrainImporterCfg

##
# Pre-defined configs
##
from locolab.tasks.manager_based.locomotion.velocity.config.unitree_go2.mdp_cfg import (  # isort: skip
    ActionsCfg,
    CommandsCfg,
    EventCfg,
    RoughRewardsCfg,
    PrivObsCfg,
    PropObsCfg,
    RoughCurriculumsCfg,
    RoughTerminationsCfg,
)
from locolab.assets import UNITREE_GO2_CFG  # isort: skip
from locolab.utils.terrains.terrains_cfg import ROUGH_TERRAINS_CFG  # isort: skip
from locolab.utils.markers import RAY_CASTER_MARKER_CFG  # isort: skip


##
# MDP definition
##
@configclass
class Go2RoughObservationsCfg:
    """Configuration for Go2 on flat terrain observations"""

    policy: PropObsCfg = PropObsCfg()
    critic: PrivObsCfg = PrivObsCfg()

    policy.history_length = 6


##
# Scene definition
##
@configclass
class Go2RoughSceneCfg(InteractiveSceneCfg):
    """Configuration for Go2 on flat terrain scene"""

    # =====  terrain  =====
    terrain: TerrainImporterCfg = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=ROUGH_TERRAINS_CFG,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=rough_terrain_visual_material_cfg(),
        debug_vis=False,
    )

    # =====  robots  =====
    robot: ArticulationCfg = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # =====  sensors  =====
    height_scanner: RayCasterCfg = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
        visualizer_cfg=RAY_CASTER_MARKER_CFG,
    )
    #  height_scanner_base = RayCasterCfg(
    #      prim_path="{ENV_REGEX_NS}/Robot/base",
    #      ray_alignment="yaw",
    #      offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
    #      pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=(0.05, 0.05)),
    #      debug_vis=False,
    #      mesh_prim_paths=["/World/ground"],
    #  )
    contact_forces: ContactSensorCfg = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True
    )

    # =====  lights  =====
    sky_light: AssetBaseCfg = blue_sky_light_cfg()


##
# Environment configuration
##
@configclass
class Go2RoughEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the Go2 flat environment."""

    # Scene settings
    scene: Go2RoughSceneCfg = Go2RoughSceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    observations: Go2RoughObservationsCfg = Go2RoughObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RoughRewardsCfg = RoughRewardsCfg()
    terminations: RoughTerminationsCfg = RoughTerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: RoughCurriculumsCfg = RoughCurriculumsCfg()

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
        self.scene.terrain.max_init_terrain_level = 0
        # update sensor update periods
        # we tick all the sensors based on the smallest update period (physics update period)
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt
        if self.scene.height_scanner is not None:
            self.scene.height_scanner.update_period = self.decimation * self.sim.dt
        if self.scene.height_scanner_base is not None:
            self.scene.height_scanner_base.update_period = self.decimation * self.sim.dt

        self.curriculum.terrain_levels.params["log_by_terrain_type"] = True


@configclass
class Go2RoughEnvCfg_PLAY(Go2RoughEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 10
        self.scene.env_spacing = 2.5
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        # reduce the number of terrains to save memory
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        # show height scan
        self.scene.height_scanner.debug_vis = True

        self.curriculum.terrain_levels.params["log_by_terrain_type"] = False
