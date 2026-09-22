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

from isaaclab.sensors.ray_caster.multi_mesh_ray_caster_camera import MultiMeshRayCasterCamera

from .noisy_camera import NoisyCameraMixin

if TYPE_CHECKING:
    from .noisy_multi_mesh_ray_caster_camera_cfg import NoisyMultiMeshRayCasterCameraCfg


class NoisyMultiMeshRayCasterCamera(NoisyCameraMixin, MultiMeshRayCasterCamera):
    cfg: NoisyMultiMeshRayCasterCameraCfg

    def _initialize_impl(self):
        super()._initialize_impl()  # type: ignore
        self.build_noise_pipeline()
        self.build_history_buffers()

    def reset(self, env_ids: Sequence[int] | None = None):
        super().reset(env_ids)
        self.reset_noise_pipeline(env_ids)
        self.reset_history_buffers(env_ids)

    def _update_buffers_impl(self, env_ids: Sequence[int]):
        super()._update_buffers_impl(env_ids)
        self.apply_noise_pipeline_to_all_data_types(env_ids)
        self.update_history_buffers(env_ids)
