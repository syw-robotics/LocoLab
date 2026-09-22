# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""
Reference: https://github.com/project-instinct/InstinctLab.git
"""

from isaaclab.sensors.ray_caster import RayCasterCameraCfg
from isaaclab.utils import configclass

from .noisy_camera_cfg import NoisyCameraCfgMixin
from .noisy_raycaster_camera import NoisyRayCasterCamera


@configclass
class NoisyRayCasterCameraCfg(NoisyCameraCfgMixin, RayCasterCameraCfg):
    """
    Configuration class for the NoisyRayCasterCamera sensor and manages image transforms and their parameters.
    """

    class_type: type = NoisyRayCasterCamera
