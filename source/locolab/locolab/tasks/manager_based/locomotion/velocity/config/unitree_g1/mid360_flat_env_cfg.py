"""G1 velocity scene with the full-rate head MID360 and internal IMU."""

import math

from isaaclab.managers import ObservationGroupCfg, ObservationTermCfg, SceneEntityCfg
from isaaclab.sensors import ImuCfg
from isaaclab.utils import configclass

from locolab.assets.sensors.mid360 import Mid360Cfg, Mid360ImuCfg, mid360_output

from .flat_env_cfg import G1FlatEnvCfg, _configure_play_env

G1_MID360_POS = (0.0002835, 0.00003, 0.41618)
G1_MID360_PITCH = 0.04014257279586953
G1_MID360_QUAT = (math.cos(G1_MID360_PITCH / 2), 0.0, -math.sin(G1_MID360_PITCH / 2), 0.0)


def g1_mid360_cfg(prim_path="{ENV_REGEX_NS}/Robot/torso_link", *, physics_dt=0.005):
    """Return (lidar, imu) configs for the upside-down G1 head mount.

    The torso/head is excluded as in the reference project; arm and leg visual meshes cause self-hits.
    Append moving obstacle RaycastTargetCfg entries to lidar.mesh_prim_paths as needed.
    """
    lidar = Mid360Cfg(
        prim_path=prim_path,
        update_period=physics_dt,
        offset=Mid360Cfg.OffsetCfg(pos=G1_MID360_POS, rot=G1_MID360_QUAT),
        mesh_prim_paths=[
            "/World/ground",
            Mid360Cfg.RaycastTargetCfg(
                prim_expr=prim_path.rsplit("/", 1)[0] + "/(?!torso_link/)[^/]+/visuals",
                is_shared=True,
            ),
        ],
    )
    x, y, z = 0.011, 0.02329, -0.04412
    # R_y(pitch) R_x(pi) transforms the internal IMU offset into torso coordinates.
    pitch = G1_MID360_PITCH
    imu_pos = (
        G1_MID360_POS[0] + math.cos(pitch) * x - math.sin(pitch) * z,
        G1_MID360_POS[1] - y,
        G1_MID360_POS[2] - math.sin(pitch) * x - math.cos(pitch) * z,
    )
    imu = Mid360ImuCfg(
        prim_path=prim_path, update_period=physics_dt, offset=ImuCfg.OffsetCfg(pos=imu_pos, rot=G1_MID360_QUAT)
    )
    return lidar, imu


@configclass
class Mid360ObservationsCfg(ObservationGroupCfg):
    points = ObservationTermCfg(func=mid360_output)
    valid = ObservationTermCfg(func=mid360_output, params={"data_type": "valid"})
    gyro = ObservationTermCfg(
        func=mid360_output, params={"sensor_cfg": SceneEntityCfg("mid360_imu"), "data_type": "gyro"}
    )
    accel_g = ObservationTermCfg(
        func=mid360_output, params={"sensor_cfg": SceneEntityCfg("mid360_imu"), "data_type": "accel_g"}
    )

    def __post_init__(self):
        self.concatenate_terms = False
        self.enable_corruption = False


@configclass
class G1Mid360FlatEnvCfg(G1FlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.events.assistive_force = None
        self.curriculum.assistive_force = None
        self.scene.num_envs = 16
        self.scene.mid360_lidar, self.scene.mid360_imu = g1_mid360_cfg(physics_dt=self.sim.dt)
        self.observations.mid360 = Mid360ObservationsCfg()


@configclass
class G1Mid360EnvCfg_PLAY(G1Mid360FlatEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        _configure_play_env(self)
        self.scene.mid360_lidar.debug_vis = True
