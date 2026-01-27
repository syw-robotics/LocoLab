# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

import isaaclab.sim as sim_utils
import locolab.tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass
from locolab.utils.terrains import TerrainImporterCfg

#  from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR


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
)
from locolab.assets import UNITREE_G1_29DOF_BEYONDMIMIC_CFG  # isort: skip


##
# MDP definition
##
@configclass
class G1FlatObservationsCfg:
    """Configuration for G1 on flat terrain observations"""

    policy: PropObsCfg = PropObsCfg()
    critic: PrivObsCfg = PrivObsCfg().replace(height_scan=None)

    policy.history_length = 5
    #  critic.history_length = 5


##
# Scene definition
##
@configclass
class G1FlatSceneCfg(InteractiveSceneCfg):
    """Configuration for G1 on flat terrain scene"""

    # =====  terrain  =====
    terrain: TerrainImporterCfg = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(1.0, 1.0, 1.0),
            metallic=0.0,
            roughness=0.8,
        ),
        debug_vis=False,
    )

    # =====  robots  =====
    robot: ArticulationCfg = UNITREE_G1_29DOF_BEYONDMIMIC_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # =====  sensors  =====
    contact_forces: ContactSensorCfg = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True
    )
    #  height_scanner = RayCasterCfg(
    #      prim_path="{ENV_REGEX_NS}/Robot/torso_link",
    #      offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
    #      ray_alignment="yaw",
    #      pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
    #      debug_vis=False,
    #      mesh_prim_paths=["/World/ground"],
    #  )

    # =====  lights  =====
    sky_light: AssetBaseCfg = AssetBaseCfg(
        prim_path="/World/skyLight",
        #  spawn=sim_utils.DomeLightCfg(
        #      intensity=750.0,
        #      texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        #  ),
        #  spawn=sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


##
# Environment configuration
##
@configclass
class G1FlatEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the G1 flat environment."""

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
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt
        #  if self.scene.height_scanner is not None:
        #      self.scene.height_scanner.update_period = self.sim.dt * self.decimation


@configclass
class G1FlatEnvCfg_PLAY(G1FlatEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 10
        self.scene.env_spacing = 2.5

        # set command ranges to the max
        import math

        self.commands.base_velocity.ranges = mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(-0.5, 1.0),
            lin_vel_y=(-0.3, 0.3),
            ang_vel_z=(-0.2, 0.2),
            heading=(-math.pi, math.pi),
        )
