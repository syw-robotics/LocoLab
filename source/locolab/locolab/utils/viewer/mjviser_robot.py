# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Drive mjviser's robot geometry from Isaac articulation body poses."""

import io
import re
from pathlib import Path

import mujoco
import numpy as np
import trimesh

from ._viser import SO3, ViserMujocoScene, group_geoms_by_visual_compat, merge_geoms, viser
from .assets import load_mjcf_model, robot_mjcf_path
from .mjcf import visual_meshes
from .utils import select_rows, xyzw_to_wxyz


def _enabled_collision_links(spawn, link_names: list[str]) -> set[str]:
    """Apply Isaac collider overrides to the displayed collision colors."""
    enabled = set(link_names)
    props = getattr(spawn, "collision_props", None)
    if not isinstance(props, dict):
        return enabled
    for pattern, fragments in props.items():
        if not isinstance(fragments, (list, tuple)):
            fragments = (fragments,)
        flag = None
        for fragment in fragments:
            value = getattr(fragment, "collision_enabled", None)
            if value is not None:
                flag = bool(value)
        if flag is None:
            continue
        try:
            regex = re.compile("/Robot" + pattern)
        except re.error:
            continue
        for name in link_names:
            probes = (f"/Robot/{name}", f"/Robot/{name}/collisions")
            if any(regex.fullmatch(probe) for probe in probes):
                if flag:
                    enabled.add(name)
                else:
                    enabled.discard(name)
    return enabled


class _RobotScene(ViserMujocoScene):
    def restore_dae_visuals(self, path: Path) -> None:
        """Restore DAE materials lost in assetx's geometry-only STL conversion."""
        from assetx.conversion.usd.visual_meshes import find_visual_mesh_sources, load_visual_mesh_sources

        sidecar = find_visual_mesh_sources(path)
        if sidecar is None:
            return
        sources = load_visual_mesh_sources(sidecar)
        model = self.mj_model
        geom_sources = {}
        for gid in range(model.ngeom):
            if model.geom_group[gid] != 0 or model.geom_type[gid] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            name = model.geom(gid).name or model.mesh(int(model.geom_dataid[gid])).name
            if name in sources:
                geom_sources[gid] = sources[name]
        if not geom_sources:
            return

        handles = [(int(g.body_ids[0]), g.handle) for g in self._mesh_groups if g.group_id == 0]
        handles += [(body, handle) for (body, group, _), handle in self._fixed_geom_handles.items() if group == 0]
        loaded = {}
        for body, handle in handles:
            geom_ids = np.flatnonzero(
                (model.geom_bodyid == body) & (model.geom_group == 0) & (model.geom_rgba[:, 3] != 0)
            ).tolist()
            groups = group_geoms_by_visual_compat(model, geom_ids)
            suffix = re.search(r"/sub(\d+)$", handle.name)
            group = groups[int(suffix[1]) if suffix else 0]
            if not any(gid in geom_sources for gid in group):
                continue
            parts = []
            for gid in group:
                source = geom_sources.get(gid)
                if source is None:
                    parts.append(merge_geoms(model, [gid]))
                    continue
                if source not in loaded:
                    loaded[source] = trimesh.load(source, force="scene", process=False)
                part = loaded[source].copy()
                mesh_id = int(model.geom_dataid[gid])
                part.apply_scale(model.mesh_scale[mesh_id])
                # MuJoCo recenters/rotates mesh vertices and folds that transform
                # into the geom pose. Undo it before using the original DAE.
                mesh_frame = np.eye(4)
                mesh_frame[:3, :3] = SO3(model.mesh_quat[mesh_id]).as_matrix()
                mesh_frame[:3, 3] = model.mesh_pos[mesh_id]
                geom_frame = np.eye(4)
                geom_frame[:3, :3] = SO3(model.geom_quat[gid]).as_matrix()
                geom_frame[:3, 3] = model.geom_pos[gid]
                part.apply_transform(geom_frame @ np.linalg.inv(mesh_frame))
                parts.append(part)
            handle.glb_data = trimesh.scene.scene.append_scenes(parts).export(file_type="glb")

    def _compute_hull_body_meshes(self) -> None:
        # mjviser's default overlay also includes collision meshes and omits
        # visual primitives. Share the ghost's visual-only selection instead.
        meshes, links = visual_meshes(self.mj_model, hull=True)
        by_body = {}
        for mesh, link in zip(meshes, links):
            by_body.setdefault(int(link) + 1, []).append(mesh)
        self._hull_body_meshes = {}
        for body_id, parts in by_body.items():
            mesh = trimesh.util.concatenate(parts)
            self._hull_body_meshes[body_id] = (
                mesh.vertices.astype(np.float32), mesh.faces.astype(np.int32)
            )
        self._hull_mesh_bodies = set(by_body)
        self._hull_opacity = 1.0


class MjviserRobot:
    """Load/map once, create mesh instances once, then update only body poses."""

    def __init__(self, robot, num_envs: int) -> None:
        self.robot = robot
        self.visual_hulls = False
        self.distance_lod = True
        self._mjcf_path = robot_mjcf_path(robot.cfg)
        model = load_mjcf_model(self._mjcf_path)
        self.model = model

        # Match by name, not order. MuJoCo body 0 is the world, not a robot link.
        link_names = [model.body(i).name for i in range(1, model.nbody)]
        sim_body_indices = {name: i for i, name in enumerate(robot.body_names)}
        sim_body_ids = np.full(model.nbody, -1, dtype=np.int64)
        body_offsets = np.tile(np.eye(4), (model.nbody, 1, 1))
        for body_id in range(1, model.nbody):
            name = model.body(body_id).name
            if name in sim_body_indices:
                sim_body_ids[body_id] = sim_body_indices[name]
                continue

            # A backend may merge fixed links. Follow the already-mapped parent
            # and accumulate the fixed offset; moving links must exist in the sim.
            parent_id = int(model.body_parentid[body_id])
            if model.body_jntnum[body_id] or sim_body_ids[parent_id] < 0:
                raise ValueError(f"MJCF body '{name}' is missing from the simulated articulation")
            sim_body_ids[body_id] = sim_body_ids[parent_id]
            local = np.eye(4)
            local[:3, :3] = SO3(model.body_quat[body_id]).as_matrix()
            local[:3, 3] = model.body_pos[body_id]
            body_offsets[body_id] = body_offsets[parent_id] @ local
        self._sim_body_ids = sim_body_ids[1:]
        self._body_offsets = body_offsets[1:]

        # Keep all environment slots for mjviser's selection and cached refresh.
        # The world slot stays at the origin with identity rotation.
        self._positions = np.zeros((num_envs, model.nbody, 3))
        self._rotations = np.tile(np.eye(3), (num_envs, model.nbody, 1, 1))

        # Vendor geom groups differ. In this display-only model, 0 = visual and
        # 1 = collision; Isaac's enable overrides determine collision colors.
        collision = (model.geom_contype != 0) | (model.geom_conaffinity != 0)
        self.has_visual = bool(np.any(~collision))
        self.has_collision = bool(np.any(collision))
        model.geom_group[:] = collision.astype(np.int32)
        enabled = _enabled_collision_links(getattr(robot.cfg, "spawn", None), link_names)
        for geom in np.flatnonzero(collision):
            active = model.body(int(model.geom_bodyid[geom])).name in enabled
            model.geom_matid[geom] = -1
            rgba = (40, 200, 90, 120) if active else (170, 176, 184, 70)  # Green / gray.
            model.geom_rgba[geom] = np.array(rgba) / 255.0

    def create_scene(self, server: viser.ViserServer) -> None:
        """Upload geometry and preserve LocoLab's collision-layer appearance."""
        self.scene = _RobotScene(server, self.model, num_envs=len(self._positions))
        self.scene.restore_dae_visuals(self._mjcf_path)
        # Retain mjviser's per-mesh LOD rules so the toggle can restore them.
        self._visual_lods = [
            (group.handle, group.handle.lod) for group in self.scene._mesh_groups if group.group_id == 0
        ]

        # mjviser exports flat colors as vertex colors without a BLEND material.
        # Private handle access stays in this adapter (mjviser version pinned).
        handles = [group.handle for group in self.scene._mesh_groups if group.group_id == 1]
        handles += [handle for (_, group, _), handle in self.scene._fixed_geom_handles.items() if group == 1]
        for handle in handles:
            meshes = trimesh.load(io.BytesIO(handle.glb_data), file_type="glb", force="scene")
            for mesh in meshes.geometry.values():
                material = trimesh.visual.material.PBRMaterial(
                    baseColorFactor=mesh.visual.vertex_colors[0], alphaMode="BLEND", doubleSided=True
                )
                mesh.visual = trimesh.visual.TextureVisuals(material=material)
            handle.glb_data = meshes.export(file_type="glb")

        # Terrain and debug overlays live under /fixed_bodies too, so mjviser's
        # camera-tracking offset moves the entire world consistently.
        self.scene.site_groups_visible = [False] * 6
        self.set_visibility(self.has_visual, not self.has_visual and self.has_collision, False)

    def set_visibility(self, visual: bool, collision: bool, hull: bool) -> None:
        """Select one shared visual/ghost mesh mode; collision stays independent."""
        self.visual_hulls = hull
        self.scene.geom_groups_visible = [visual and not hull, collision, False, False, False, False]
        # The public hull setter also synchronizes geom/site group visibility.
        self.scene.show_convex_hull = visual and hull
        self.scene.refresh_visualization()

    def set_distance_lod(self, enabled: bool) -> None:
        """Toggle distance reduction on visual meshes without rebuilding them."""
        self.distance_lod = enabled
        for handle, lod in self._visual_lods:
            handle.lod = lod if enabled else "off"

    def body_poses(self, selector):
        """Read world link frames, including fixed links merged by the backend."""
        data = self.robot.data
        position = select_rows(data.body_pos_w, selector)[:, self._sim_body_ids]
        orientation = select_rows(data.body_quat_w, selector)[:, self._sim_body_ids]
        if hasattr(position, "detach"):
            import torch

            # Pack position and xyzw quaternion into a single GPU-to-CPU transfer.
            packed = torch.cat((position, orientation), dim=-1).detach().cpu().numpy()
        else:
            packed = np.concatenate((position, orientation), axis=-1)
        rotation = SO3(xyzw_to_wxyz(packed[..., 3:])).as_matrix()

        # World pose = simulated ancestor pose × fixed body offset.
        offset_position = self._body_offsets[:, :3, 3]
        position = packed[..., :3] + (rotation @ offset_position[..., None])[..., 0]
        return position, rotation @ self._body_offsets[:, :3, :3]

    def update(self, selector, env_idx: int) -> None:
        """Transfer selected body poses once and let mjviser update its instances."""
        position, rotation = self.body_poses(selector)
        self._positions[selector, 1:] = position
        self._rotations[selector, 1:] = rotation
        self.scene.show_only_selected = position.shape[0] != len(self._positions)

        # mjviser caches these arrays and serializes poses on the websocket thread.
        self.scene.update_from_arrays(self._positions.copy(), self._rotations.copy(), env_idx=env_idx)
