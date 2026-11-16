"""Robot-independent MID360 LiDAR and IMU configuration."""

import math

import torch

from isaaclab.sensors import ImuCfg, MultiMeshRayCasterCfg, patterns
from isaaclab.utils import configclass

from locolab.utils.debug_viz.markers import MUTED_RED_RAY_CASTER_MARKER_CFG

from .mid360 import Mid360, Mid360Imu


def _batch_pattern(cfg, device):
    """Allocate the backend's per-batch buffers; the scan kernel supplies live directions."""
    return torch.zeros((cfg.samples, 3), device=device), torch.zeros((cfg.samples, 3), device=device)


@configclass
class _BatchPatternCfg(patterns.PatternBaseCfg):
    func = _batch_pattern
    samples: int = 1000


@configclass
class Mid360Cfg(MultiMeshRayCasterCfg):
    class_type: type = Mid360
    visualizer_cfg = MUTED_RED_RAY_CASTER_MARKER_CFG
    update_period: float = 0.005
    point_rate: float = 200_000.0
    frame_rate: float = 10.0
    blind: float = 0.1
    range_10: float = 40.0
    max_distance: float = 70.0
    angle_sigma: float = math.radians(0.15)
    false_alarm_rate: float = 1e-4
    resolution: float = 0.001
    noise: bool = True
    seed: int = 0
    target_reflectivity: dict[int, float] = {}
    """Diffuse 905 nm reflectivity by mesh_prim_paths index. Unspecified targets use 0.5."""
    pattern_cfg: _BatchPatternCfg = _BatchPatternCfg()


@configclass
class Mid360ImuCfg(ImuCfg):
    class_type: type = Mid360Imu
    update_period: float = 0.005
    noise: bool = True
    seed: int = 12345
    gyro_noise_density: float = math.radians(4.5e-3)
    accel_noise_density: float = 100e-6
