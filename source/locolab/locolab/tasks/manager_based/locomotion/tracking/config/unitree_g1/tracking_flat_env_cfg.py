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

from locolab.assets.robots.unitree.G1.g1 import G1_SPATIAL_SYMMETRY_MAPPING, UNITREE_G1_29DOF_PELVIS_BASE_CFG  
from locolab.utils.physics import set_self_collisions, set_locomotion_physics
from locolab.utils.scene import blue_sky_light_cfg, flat_terrain_visual_material_cfg
from locolab.utils.terrains import TerrainImporterCfg

from locolab.tasks.manager_based.locomotion.tracking.config.unitree_g1.mdp_cfg import (  # isort: skip
    ActionsCfg,
    CommandsCfg,
    EventCfg,
    PolicyCfg,
    PrivilegedCfg,
    RewardsCfg,
    TerminationsCfg,
)


@configclass
class ObservationsCfg:
    """Actor and critic groups consumed by z_rl."""

    policy: PolicyCfg = PolicyCfg()
    critic: PrivilegedCfg = PrivilegedCfg()


@configclass
class G1TrackingSceneCfg(InteractiveSceneCfg):
    """Flat ground and one G1. Every environment tracks the same motion clip."""

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
    robot: ArticulationCfg = UNITREE_G1_29DOF_PELVIS_BASE_CFG  .replace(prim_path="{ENV_REGEX_NS}/Robot")
    self_collision_contact_forces: ContactSensorCfg = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*",
        # Each filter names one body, as required by ovphysx. Mapping keys cover all G1 links.
        filter_prim_paths_expr=[f"{{ENV_REGEX_NS}}/Robot/{name}" for name in G1_SPATIAL_SYMMETRY_MAPPING],
        history_length=4,
    )
    sky_light: AssetBaseCfg = blue_sky_light_cfg()


@configclass
class CurriculumCfg:
    """Tracking has no curriculum."""


@configclass
class G1TrackingEnvCfg(ManagerBasedRLEnvCfg):
    """G1 flat motion tracking on Lafan clips under ``LAFAN_DATASET_ROOT``."""

    seed: int = 42

    scene: G1TrackingSceneCfg = G1TrackingSceneCfg(num_envs=4096, env_spacing=2.5)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        self.decimation = 4
        self.episode_length_s = 10.0
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physics_material = self.scene.terrain.physics_material
        # Same preset as G1 velocity flat, with higher contact budget for full-body self collision.
        set_locomotion_physics(self.sim, rigid_patch_count=2**16, njmax=256, nconmax=32)
        set_self_collisions(self.scene.robot.spawn, True)
        # Foot solref is authored in the USD; Newton's default shape gains do not override it.
        self.scene.robot.spawn.collision_props["/.*ankle_roll_link/collisions.*"][0].solref = (0.01, 1.0)
        self.scene.self_collision_contact_forces.update_period = self.sim.dt


@configclass
class G1TrackingEnvCfg_PLAY(G1TrackingEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()
        self.scene.num_envs = 10
        self.scene.env_spacing = 2.5
        self.episode_length_s = 1.0e9
        
        self.events.push_robot = None

        self.commands.motion.pose_range = {}
        self.commands.motion.velocity_range = {}
        self.commands.motion.sampling_mode = "start"
        self.commands.motion.debug_vis = True
