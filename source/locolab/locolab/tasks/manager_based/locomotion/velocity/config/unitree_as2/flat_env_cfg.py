# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim.schemas import UsdPhysicsCollisionCfg
from isaaclab.utils import configclass

from isaaclab_physx.sim.spawners.materials import PhysxRigidBodyMaterialCfg

from locolab.assets.robots.unitree.AS2.as2 import UNITREE_AS2_CFG
from locolab.utils.physics import set_locomotion_physics, set_self_collisions
from locolab.utils.scene import blue_sky_light_cfg, flat_rough_terrain_visual_material_cfg
from locolab.utils.terrains import TerrainImporterCfg
from locolab.utils.terrains.terrains_cfg import FLAT_ROUGH_TERRAINS_CFG

from .mdp_cfg import (
    FLAT_CONTACT_SENSOR_LINK_NAMES,
    ActionsCfg,
    CommandsCfg,
    EventCfg,
    FlatRewardsCfg,
    FlatTerminationsCfg,
    PrivObsCfg,
    PropObsCfg,
)


@configclass
class AS2FlatObservationsCfg:
    """Policy and critic observations for AS2 on flat terrain."""

    policy: PropObsCfg = PropObsCfg(history_length=5)
    critic: PrivObsCfg = PrivObsCfg()


@configclass
class AS2FlatSceneCfg(InteractiveSceneCfg):
    """Scene configuration for AS2 on flat terrain."""

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
        physics_material=PhysxRigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=flat_rough_terrain_visual_material_cfg(),
        debug_vis=False,
    )

    robot: ArticulationCfg = UNITREE_AS2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    contact_forces: ContactSensorCfg = ContactSensorCfg(
        prim_path=f"{{ENV_REGEX_NS}}/Robot/{FLAT_CONTACT_SENSOR_LINK_NAMES}", history_length=3, track_air_time=True
    )

    sky_light: AssetBaseCfg = blue_sky_light_cfg()


@configclass
class AS2FlatEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the AS2 flat environment."""

    seed: int = 42

    scene: AS2FlatSceneCfg = AS2FlatSceneCfg(num_envs=4096, env_spacing=2.5)
    observations: AS2FlatObservationsCfg = AS2FlatObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: FlatRewardsCfg = FlatRewardsCfg()
    terminations: FlatTerminationsCfg = FlatTerminationsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 20.0
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material
        set_locomotion_physics(
            self.sim,
            mjwarp=dict(cone="elliptic", impratio=5.0, njmax=128, nconmax=24),
            steel=dict(articulation_constraint_max=128),
            physx=dict(rigid_patch_count=2**16),
        )
        set_self_collisions(self.scene.robot.spawn, False)
        self.scene.robot.spawn.collision_props["/(?!.*(foot|calf)).*"] = [
            UsdPhysicsCollisionCfg(collision_enabled=False)
        ]
        self.scene.contact_forces.update_period = self.sim.dt


@configclass
class AS2FlatEnvCfg_PLAY(AS2FlatEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 10
        self.commands.base_velocity.debug_vis = True
        self.events.push_robot_by_setting_velocity.params["debug_vis"] = True
