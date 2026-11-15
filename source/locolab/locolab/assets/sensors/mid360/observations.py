"""Observation terms shared by locomotion and perception tasks."""

from isaaclab.managers import SceneEntityCfg


def mid360_output(env, sensor_cfg: SceneEntityCfg = SceneEntityCfg("mid360_lidar"), data_type: str = "points"):
    """Latest complete frame (N, P, 3), mask (N, P), or IMU output (N, 3).

    Returns a GPU view. A frame is held between publications, and zeros until the first full frame.
    Use concatenate_terms=False in the observation group to retain the point dimension.
    """
    return env.scene.sensors[sensor_cfg.name].data.output[data_type]
