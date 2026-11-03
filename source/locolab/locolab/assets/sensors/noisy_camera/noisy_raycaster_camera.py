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

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch
import warp as wp

from isaaclab.sensors.ray_caster import RayCasterCamera

from .noisy_camera import NoisyCameraMixin

if TYPE_CHECKING:
    from .noisy_raycaster_camera_cfg import NoisyRayCasterCameraCfg


class NoisyRayCasterCamera:
    """Compose noise processing with the active Isaac Lab ray-camera backend."""

    @classmethod
    def resolve_class(cls):
        return type(cls.__name__, (_NoisyRayCasterCameraMixin, RayCasterCamera.resolve_class()), {})

    def __new__(cls, cfg):
        return cls.resolve_class()(cfg)


class _NoisyRayCasterCameraMixin(NoisyCameraMixin):
    cfg: NoisyRayCasterCameraCfg

    def _initialize_impl(self):
        super()._initialize_impl()  # type: ignore
        self._ALL_INDICES = torch.arange(self.num_instances, device=self.device)
        self.build_noise_pipeline()
        self.build_history_buffers()

    """
    Operations
    """

    def reset(self, env_ids: Sequence[int] | None = None, *, env_mask=None):
        """Reset the sensor and noise pipeline."""
        reset_mask = self._resolve_indices_and_mask(env_ids, env_mask)
        super().reset(env_mask=reset_mask)
        env_ids = wp.to_torch(reset_mask).nonzero(as_tuple=False).flatten()
        self.reset_noise_pipeline(env_ids)
        self.reset_history_buffers(env_ids)

    """
    Implementation
    """

    def _update_buffers_impl(self, env_mask: wp.array):
        """Fills the buffers of the sensor data."""

        super()._update_buffers_impl(env_mask)
        env_ids = wp.to_torch(env_mask).nonzero(as_tuple=False).flatten()
        self.apply_noise_pipeline_to_all_data_types(env_ids)
        self.update_history_buffers(env_ids)
