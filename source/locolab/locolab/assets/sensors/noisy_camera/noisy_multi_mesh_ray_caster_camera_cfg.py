# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from isaaclab.sensors.ray_caster import MultiMeshRayCasterCameraCfg
from isaaclab.utils import configclass

from .noisy_camera_cfg import NoisyCameraCfgMixin
from .noisy_multi_mesh_ray_caster_camera import NoisyMultiMeshRayCasterCamera


@configclass
class NoisyMultiMeshRayCasterCameraCfg(NoisyCameraCfgMixin, MultiMeshRayCasterCameraCfg):
    """Noisy wrapper around Isaac Lab's multi-mesh ray-caster camera."""

    class_type: type = NoisyMultiMeshRayCasterCamera
