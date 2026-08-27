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

from locolab.utils.scene import rough_terrain_visual_material_cfg, blue_sky_light_cfg
from locolab.utils.terrains import TerrainImporterCfg

#  from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR


##
# Pre-defined configs
##
from locolab.tasks.manager_based.locomotion.velocity.config.unitree_g1.mdp_cfg import (  # isort: skip
    ActionsCfg,
    CommandsCfg,
    EventCfg,
    RoughRewardsCfg,
    PrivObsCfg,
    PropObsCfg,
    RoughTerminationsCfg,
    RoughCurriculumsCfg,
    CONTACT_SENSOR_LINK_NAMES,
)
from locolab.assets import UNITREE_G1_29DOF_BEYONDMIMIC_TORSO_BASE_CFG  # isort: skip
from locolab.utils.terrains.terrains_cfg import ROUGH_TERRAINS_CFG  # isort: skip
from locolab.utils.markers import BLUE_RAY_CASTER_MARKER_CFG  # isort: skip


##
# MDP definition
##
@configclass
class G1RoughObservationsCfg:
    """Configuration for G1 on rough terrain observations"""

    policy: PropObsCfg = PropObsCfg()
    critic: PrivObsCfg = PrivObsCfg()

    policy.history_length = 5


##
# Scene definition
##
@configclass
class G1RoughSceneCfg(InteractiveSceneCfg):
    """Configuration for G1 on rough terrain scene"""

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
    robot: ArticulationCfg = UNITREE_G1_29DOF_BEYONDMIMIC_TORSO_BASE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

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
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{CONTACT_SENSOR_LINK_NAMES}", history_length=3, track_air_time=True
    )

    # =====  lights  =====
    sky_light: AssetBaseCfg = blue_sky_light_cfg()


##
# Environment configuration
##
@configclass
class G1RoughEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the G1 rough environment."""

    # Scene settings
    scene: G1RoughSceneCfg = G1RoughSceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    observations: G1RoughObservationsCfg = G1RoughObservationsCfg()
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
        # update sensor update periods
        # we tick all the sensors based on the smallest update period (physics update period)
        self.scene.contact_forces.update_period = self.sim.dt


@configclass
class G1RoughEnvCfg_PLAY(G1RoughEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 10
        self.scene.env_spacing = 2.5
        self.commands.base_velocity.debug_vis = True

        # set command ranges to the curriculum limits
        lin_vel_cmd_params = self.curriculum.lin_vel_cmd_levels.params
        ang_vel_cmd_params = self.curriculum.ang_vel_cmd_levels.params
        self.commands.base_velocity.ranges = type(self.commands.base_velocity.ranges)(
            lin_vel_x=lin_vel_cmd_params["max_lin_vel_x_ranges"],
            lin_vel_y=lin_vel_cmd_params["max_lin_vel_y_ranges"],
            ang_vel_z=ang_vel_cmd_params["max_ang_vel_z_ranges"],
            heading=self.commands.base_velocity.ranges.heading,
        )
