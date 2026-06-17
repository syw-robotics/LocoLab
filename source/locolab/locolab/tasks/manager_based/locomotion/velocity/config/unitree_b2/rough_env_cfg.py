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
from locolab.tasks.manager_based.locomotion.velocity.config.unitree_B2.mdp_cfg import (  # isort: skip
    ActionsCfg,
    CommandsCfg,
    EventCfg,
    RoughRewardsCfg,
    PrivObsCfg,
    PropObsCfg,
    RoughCurriculumsCfg,
    RoughTerminationsCfg,
)
from locolab.assets import UNITREE_B2_CFG  # isort: skip
from locolab.utils.terrains.terrains_cfg import ROUGH_TERRAINS_CFG  # isort: skip
from locolab.utils.markers import BLUE_RAY_CASTER_MARKER_CFG, PURPLE_RAY_CASTER_MARKER_CFG


##
# MDP definition
##
@configclass
class B2RoughObservationsCfg:
    """Configuration for B2 on flat terrain observations"""

    policy: PropObsCfg = PropObsCfg()
    critic: PrivObsCfg = PrivObsCfg()

    #  policy.history_length = 6
    policy.history_length = 5
    #  policy.concatenate_terms = False


##
# Scene definition
##
@configclass
class B2RoughSceneCfg(InteractiveSceneCfg):
    """Configuration for B2 on flat terrain scene"""

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
    robot: ArticulationCfg = UNITREE_B2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # =====  sensors  =====
    height_scanner: RayCasterCfg = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 5.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
        visualizer_cfg=BLUE_RAY_CASTER_MARKER_CFG,
    )
    contact_forces: ContactSensorCfg = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True
    )
    #  FL_foot_height_scanner: RayCasterCfg = RayCasterCfg(
    #      prim_path="{ENV_REGEX_NS}/Robot/FL_foot",
    #      offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 5.0)),
    #      ray_alignment="yaw",
    #      pattern_cfg=patterns.GridPatternCfg(resolution=0.05, size=[0.05, 0.05]),
    #      debug_vis=True,
    #      mesh_prim_paths=["/World/ground"],
    #      visualizer_cfg=PURPLE_RAY_CASTER_MARKER_CFG,
    #  )
    #  FR_foot_height_scanner: RayCasterCfg = RayCasterCfg(
    #      prim_path="{ENV_REGEX_NS}/Robot/FR_foot",
    #      offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 5.0)),
    #      ray_alignment="yaw",
    #      pattern_cfg=patterns.GridPatternCfg(resolution=0.05, size=[0.05, 0.05]),
    #      debug_vis=True,
    #      mesh_prim_paths=["/World/ground"],
    #      visualizer_cfg=PURPLE_RAY_CASTER_MARKER_CFG,
    #  )
    #  RL_foot_height_scanner: RayCasterCfg = RayCasterCfg(
    #      prim_path="{ENV_REGEX_NS}/Robot/RL_foot",
    #      offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 5.0)),
    #      ray_alignment="yaw",
    #      pattern_cfg=patterns.GridPatternCfg(resolution=0.05, size=[0.05, 0.05]),
    #      debug_vis=True,
    #      mesh_prim_paths=["/World/ground"],
    #      visualizer_cfg=PURPLE_RAY_CASTER_MARKER_CFG,
    #  )
    #  RR_foot_height_scanner: RayCasterCfg = RayCasterCfg(
    #      prim_path="{ENV_REGEX_NS}/Robot/RR_foot",
    #      offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 5.0)),
    #      ray_alignment="yaw",
    #      pattern_cfg=patterns.GridPatternCfg(resolution=0.05, size=[0.05, 0.05]),
    #      debug_vis=True,
    #      mesh_prim_paths=["/World/ground"],
    #      visualizer_cfg=PURPLE_RAY_CASTER_MARKER_CFG,
    #  )

    # =====  lights  =====
    sky_light: AssetBaseCfg = blue_sky_light_cfg()


##
# Environment configuration
##
@configclass
class B2RoughEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the B2 flat environment."""

    # Scene settings
    scene: B2RoughSceneCfg = B2RoughSceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    observations: B2RoughObservationsCfg = B2RoughObservationsCfg()
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
        self.scene.contact_forces.update_period = self.sim.dt
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt
        #  self.scene.FL_foot_height_scanner.update_period = self.decimation * self.sim.dt
        #  self.scene.FR_foot_height_scanner.update_period = self.decimation * self.sim.dt
        #  self.scene.RL_foot_height_scanner.update_period = self.decimation * self.sim.dt
        #  self.scene.RR_foot_height_scanner.update_period = self.decimation * self.sim.dt

        # log terrain levels by each terrain type
        self.curriculum.terrain_levels.params["log_by_terrain_type"] = True


@configclass
class B2RoughEnvCfg_PLAY(B2RoughEnvCfg):
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
