# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from isaaclab_physx.sim.spawners.materials import PhysxRigidBodyMaterialCfg

from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass

from locolab.utils.physics import set_self_collisions, set_locomotion_physics
from locolab.utils.scene import blue_sky_light_cfg, flat_terrain_visual_material_cfg
from locolab.utils.terrains import TerrainImporterCfg


##
# Pre-defined configs
##
from locolab.tasks.manager_based.locomotion.velocity.config.unitree_g1.mdp_cfg import (  # isort: skip
    ActionsCfg,
    CommandsCfg,
    EventCfg,
    FlatRewardsCfg,
    PrivObsCfg,
    PropObsCfg,
    FlatTerminationsCfg,
    FlatCurriculumsCfg,
    FOOT_CONTACT_LINK_NAMES,
    SELF_COLLISION_LINK_NAMES,
)
#  from locolab.assets.robots.unitree.G1.g1 import UNITREE_G1_29DOF_TORSO_BASE_CFG    # isort: skip
# from locolab.assets.robots.unitree.G1.g1 import UNITREE_G1_29DOF_PELVIS_BASE_CFG  # isort: skip
from locolab.assets.robots.unitree.G1.g1 import g1_sphere_feet_cfg  # isort: skip


##
# MDP definition
##
@configclass
class G1FlatObservationsCfg:
    """Configuration for G1 on flat terrain observations"""

    policy: PropObsCfg = PropObsCfg()
    critic: PrivObsCfg = PrivObsCfg().replace(height_scan=None)

    policy.history_length = 5


##
# Scene definition
##
@configclass
class G1FlatSceneCfg(InteractiveSceneCfg):
    """Configuration for pelvis based G1 on flat terrain scene"""

    # =====  terrain  =====
    terrain: TerrainImporterCfg = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        physics_material=PhysxRigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=flat_terrain_visual_material_cfg(),
        debug_vis=False,
    )

    # =====  robots  =====
    #  robot: ArticulationCfg = g1_sphere_feet_cfg("torso").replace(
    robot: ArticulationCfg = g1_sphere_feet_cfg("pelvis").replace(
    #  robot: ArticulationCfg = UNITREE_G1_29DOF_PELVIS_BASE_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
    )

    # =====  sensors  =====
    feet_contact_forces: ContactSensorCfg = ContactSensorCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{FOOT_CONTACT_LINK_NAMES}",
        history_length=3,
        track_air_time=True,
    )
    self_collision_contact_forces: ContactSensorCfg = ContactSensorCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{SELF_COLLISION_LINK_NAMES}",
        history_length=3,
        track_air_time=False,
    )

    # =====  lights  =====
    sky_light: AssetBaseCfg = blue_sky_light_cfg()


##
# Environment configuration
##
@configclass
class G1FlatEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the G1 flat environment."""

    seed: int = 42

    # Scene settings
    scene: G1FlatSceneCfg = G1FlatSceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    observations: G1FlatObservationsCfg = G1FlatObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: FlatRewardsCfg = FlatRewardsCfg()
    terminations: FlatTerminationsCfg = FlatTerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: FlatCurriculumsCfg = FlatCurriculumsCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 4
        self.episode_length_s = 20.0
        # simulation settings
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material
        # physics settings
        set_locomotion_physics(
            self.sim, rigid_patch_count=2**16,
            num_substeps=1, cone="elliptic", impratio=5.0, njmax=90, nconmax=20, ls_iterations=30,
        )
        set_self_collisions(self.scene.robot.spawn, True)

        # update sensor update periods
        self.scene.feet_contact_forces.update_period = self.sim.dt
        self.scene.self_collision_contact_forces.update_period = self.sim.dt


def _configure_play_env(cfg: G1FlatEnvCfg) -> None:
    cfg.scene.num_envs = 10
    cfg.scene.env_spacing = 2.5
    cfg.commands.base_velocity.debug_vis = True
    cfg.events.push_robot_by_setting_velocity.params["debug_vis"] = True

    lin_vel_cmd_params = cfg.curriculum.lin_vel_cmd_levels.params
    ang_vel_cmd_params = cfg.curriculum.ang_vel_cmd_levels.params
    push_robot_params = cfg.curriculum.push_robot_levels.params
    cfg.commands.base_velocity.ranges = type(cfg.commands.base_velocity.ranges)(
        lin_vel_x=lin_vel_cmd_params["max_lin_vel_x_ranges"],
        lin_vel_y=lin_vel_cmd_params["max_lin_vel_y_ranges"],
        ang_vel_z=ang_vel_cmd_params["max_ang_vel_z_ranges"],
        heading=cfg.commands.base_velocity.ranges.heading,
    )
    cfg.events.push_robot_by_setting_velocity.params["velocity_range"] = dict(
        push_robot_params["max_push_velocity_ranges"]
    )


@configclass
class G1FlatEnvCfg_PLAY(G1FlatEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()
        _configure_play_env(self)


