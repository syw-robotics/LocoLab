# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch
import warp as wp

from isaaclab.sensors.ray_caster.multi_mesh_ray_caster_camera import MultiMeshRayCasterCamera

from .noisy_camera import NoisyCameraMixin

if TYPE_CHECKING:
    from .noisy_multi_mesh_ray_caster_camera_cfg import NoisyMultiMeshRayCasterCameraCfg


class NoisyMultiMeshRayCasterCamera:
    """Compose noise processing with the active multi-mesh camera backend."""

    def __new__(cls, cfg):
        sensor_class = type(cls.__name__, (_NoisyMultiMeshMixin, MultiMeshRayCasterCamera.resolve_class()), {})
        return sensor_class(cfg)


class _NoisyMultiMeshMixin(NoisyCameraMixin):
    cfg: NoisyMultiMeshRayCasterCameraCfg

    def _initialize_impl(self):
        super()._initialize_impl()  # type: ignore
        self._ALL_INDICES = torch.arange(self.num_instances, device=self.device)
        self.build_noise_pipeline()
        self.build_history_buffers()

    def reset(self, env_ids: Sequence[int] | None = None, *, env_mask=None):
        reset_mask = self._resolve_indices_and_mask(env_ids, env_mask)
        super().reset(env_mask=reset_mask)
        env_ids = wp.to_torch(reset_mask).nonzero(as_tuple=False).flatten()
        self.reset_noise_pipeline(env_ids)
        self.reset_history_buffers(env_ids)

    def _update_buffers_impl(self, env_mask: wp.array):
        super()._update_buffers_impl(env_mask)
        env_ids = wp.to_torch(env_mask).nonzero(as_tuple=False).flatten()
        self.apply_noise_pipeline_to_all_data_types(env_ids)
        self.update_history_buffers(env_ids)
