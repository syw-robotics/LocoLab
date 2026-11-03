# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Keep Kit markers and supply translucent mesh instances to Newton viewers."""

import weakref

import numpy as np
import torch
import warp as wp
from isaaclab_visualizers.newton.newton_visualization_markers import NewtonVisualizationMarkers
from newton.viewer import ViewerGL

from isaaclab.markers import VisualizationMarkers


class MotionGhostMarkers(VisualizationMarkers):
    def __init__(self, cfg, ghost):
        self.ghost = weakref.proxy(ghost)
        super().__init__(cfg)

    def _ensure_newton_backend(self):
        # The upstream Newton marker backend does not support MeshFileCfg or opacity.
        if not any(isinstance(backend, _NewtonGhostMarkers) for backend in self._backends):
            self._backends.append(_NewtonGhostMarkers(self.cfg, self.ghost, self._is_visible))


class _NewtonGhostMarkers(NewtonVisualizationMarkers):
    def __init__(self, cfg, ghost, visible):
        super().__init__(cfg, visible)
        self.ghost = ghost
        self._instance_buffers = {}
        self._visible_viewers = set()

    def render(self, viewer, visible_env_ids, num_envs):
        if self.translations is None:
            return
        meshes = self.ghost.bodies.meshes
        count = self.count // len(meshes)
        # Ghosts cover only the first max_envs, so dense num_envs-based inference
        # in the standard marker backend cannot determine their environment IDs.
        env_ids = list(range(count)) if visible_env_ids is None else [i for i in visible_env_ids if i < count]
        if not self.visible or not env_ids:
            if id(viewer) not in self._visible_viewers:
                return
            for index in range(len(meshes)):
                mesh_name = f"{self.group_id}/meshes/mesh_{index}"
                if (id(viewer), mesh_name) in self._registered_meshes:
                    viewer.log_instances(
                        f"{self.group_id}/mesh_{index}", mesh_name, None, None, None, None, hidden=True
                    )
            self._visible_viewers.discard(id(viewer))
            return
        selection = slice(None) if visible_env_ids is None else env_ids
        positions = self.translations.reshape(count, len(meshes), 3)[selection]
        orientations = self.orientations.reshape(count, len(meshes), 4)[selection]
        if viewer.world_offsets is not None:
            positions = positions + wp.to_torch(viewer.world_offsets)[env_ids, None].to(positions)
        # Pack all transforms once. Per-link Warp arrays below share this storage.
        transforms = torch.cat((positions, orientations), dim=-1).permute(1, 0, 2)
        transforms = transforms.to(device=str(viewer.device)).contiguous()
        buffers = self._instance_buffers.get(id(viewer))
        resized = buffers is None or buffers[0] != len(env_ids)
        if resized:
            buffers = (
                len(env_ids),
                wp.array([self.ghost.color] * len(env_ids), dtype=wp.vec3, device="cpu"),
                wp.array([self.ghost.opacity] * len(env_ids), dtype=wp.float32, device="cpu"),
            )
            self._instance_buffers[id(viewer)] = buffers
        # GL retains style buffers. Upload them only on allocation/count changes;
        # host arrays also avoid the viewer's GPU-to-CPU color/opacity round trips.
        update_style = resized or not isinstance(viewer, ViewerGL)
        for index, mesh in enumerate(meshes):
            mesh_name = f"{self.group_id}/meshes/mesh_{index}"
            batch_name = f"{self.group_id}/mesh_{index}"
            registered_key = (id(viewer), mesh_name)
            if registered_key not in self._registered_meshes:
                viewer.log_mesh(
                    mesh_name,
                    wp.array(np.asarray(mesh.vertices, dtype=np.float32), dtype=wp.vec3, device=viewer.device),
                    wp.array(np.asarray(mesh.faces, dtype=np.int32).ravel(), dtype=wp.int32, device=viewer.device),
                    normals=wp.array(
                        np.asarray(mesh.vertex_normals, dtype=np.float32), dtype=wp.vec3, device=viewer.device
                    ),
                    color=self.ghost.color,
                    opacity=self.ghost.opacity,
                    hidden=True,
                )
                self._registered_meshes.add(registered_key)
            viewer.log_instances(
                batch_name,
                mesh_name,
                wp.from_torch(transforms[index], dtype=wp.transform),
                None,
                buffers[1] if update_style else None,
                None,
                hidden=False,
                opacities=buffers[2] if update_style else None,
            )
        self._visible_viewers.add(id(viewer))
