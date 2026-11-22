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


class _RobotMeshApi:
    """Finish robot meshes before Viser serializes their creation messages."""

    def __init__(self, api, model: mujoco.MjModel, path: Path, asset_cache) -> None:
        from assetx.conversion.usd.visual_meshes import find_visual_mesh_sources, load_visual_mesh_sources

        self._api = api
        self._asset_cache = asset_cache
        self._model = model
        self._body_ids = {model.body(i).name or f"body_{i}": i for i in range(model.nbody)}
        self._dae_scenes = {}
        sidecar = find_visual_mesh_sources(path)
        sources = load_visual_mesh_sources(sidecar) if sidecar is not None else {}
        self._geom_sources = {}
        if sources:
            for gid in range(model.ngeom):
                if model.geom_group[gid] != 0 or model.geom_type[gid] != mujoco.mjtGeom.mjGEOM_MESH:
                    continue
                name = model.geom(gid).name or model.mesh(int(model.geom_dataid[gid])).name
                if name in sources:
                    self._geom_sources[gid] = sources[name]

    def __getattr__(self, name):
        return getattr(self._api, name)

    def add_mesh_trimesh(self, name, mesh, **kwargs):
        mesh = self._finish_mesh(name, mesh)
        if self._asset_cache is not None:
            return self._api.add_glb(name, self._asset_cache.mesh_reference(name, mesh), **kwargs)
        return self._api.add_mesh_trimesh(name, mesh, **kwargs)

    def add_batched_meshes_trimesh(self, name, mesh, **kwargs):
        mesh = self._finish_mesh(name, mesh)
        if self._asset_cache is not None:
            return self._api.add_batched_glb(name, self._asset_cache.mesh_reference(name, mesh), **kwargs)
        return self._api.add_batched_meshes_trimesh(name, mesh, **kwargs)

    def _finish_mesh(self, name, mesh):
        # mjviser 0.0.14 names each geometry subgroup by its representative body.
        match = re.fullmatch(r"/(bodies|fixed_bodies)/(.+)/group([01])(?:/sub(\d+))?", name)
        if match is None:
            return mesh
        namespace, body_name, group_id, sub_idx = match.groups()
        if group_id == "1":
            # A BLEND material preserves collision transparency in the browser.
            collision = trimesh.load(io.BytesIO(mesh.export(file_type="glb")), file_type="glb", force="scene")
            for part in collision.geometry.values():
                material = trimesh.visual.material.PBRMaterial(
                    baseColorFactor=part.visual.vertex_colors[0], alphaMode="BLEND", doubleSided=True
                )
                part.visual = trimesh.visual.TextureVisuals(material=material)
            return collision
        if not self._geom_sources:
            return mesh

        # Restore DAE materials lost in assetx's geometry-only STL conversion.
        model = self._model
        geom_mask = (model.geom_bodyid == self._body_ids[body_name]) & (model.geom_group == 0)
        geom_mask &= model.geom_rgba[:, 3] != 0
        if namespace == "fixed_bodies":
            geom_mask &= model.geom_type != mujoco.mjtGeom.mjGEOM_PLANE
        groups = group_geoms_by_visual_compat(model, np.flatnonzero(geom_mask).tolist())
        group = groups[int(sub_idx or 0)]
        if not any(gid in self._geom_sources for gid in group):
            return mesh
        parts = []
        for gid in group:
            source = self._geom_sources.get(gid)
            if source is None:
                parts.append(merge_geoms(model, [gid]))
                continue
            if source not in self._dae_scenes:
                self._dae_scenes[source] = trimesh.load(source, force="scene", process=False)
            part = self._dae_scenes[source].copy()
            mesh_id = int(model.geom_dataid[gid])
            part.apply_scale(model.mesh_scale[mesh_id])
            # MuJoCo folds the mesh's centering/rotation into the geom pose.
            mesh_frame = np.eye(4)
            mesh_frame[:3, :3] = SO3(model.mesh_quat[mesh_id]).as_matrix()
            mesh_frame[:3, 3] = model.mesh_pos[mesh_id]
            geom_frame = np.eye(4)
            geom_frame[:3, :3] = SO3(model.geom_quat[gid]).as_matrix()
            geom_frame[:3, 3] = model.geom_pos[gid]
            part.apply_transform(geom_frame @ np.linalg.inv(mesh_frame))
            parts.append(part)
        return trimesh.scene.scene.append_scenes(parts)


class _RobotMeshServer:
    """Use the mesh adapter only while mjviser builds the robot scene."""

    def __init__(self, server, model, path, asset_cache) -> None:
        self._server = server
        self.scene = _RobotMeshApi(server.scene, model, path, asset_cache)

    def __getattr__(self, name):
        return getattr(self._server, name)


class _RobotScene(ViserMujocoScene):
    def __init__(self, server, model, num_envs: int, *, mjcf_path: Path, asset_cache) -> None:
        super().__init__(_RobotMeshServer(server, model, mjcf_path, asset_cache), model, num_envs=num_envs)
        self.server = server

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
        self.distance_lod = False
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

    def create_scene(self, server: viser.ViserServer, *, asset_cache=None) -> None:
        """Upload geometry and preserve LocoLab's collision-layer appearance."""
        self.scene = _RobotScene(
            server, self.model, len(self._positions), mjcf_path=self._mjcf_path, asset_cache=asset_cache
        )
        # Retain mjviser's per-mesh LOD rules so the toggle can restore them.
        self._visual_lods = [
            (group.handle, group.handle.lod) for group in self.scene._mesh_groups if group.group_id == 0
        ]
        self.set_distance_lod(self.distance_lod)

        # Terrain and debug overlays live under /fixed_bodies too, so mjviser's
        # camera-tracking offset moves the entire world consistently.
        self.scene.site_groups_visible = [False] * 6
        self.set_visibility(self.has_visual, not self.has_visual and self.has_collision, False)

    def create_scene_gui(self) -> None:
        """Keep the environment slider synchronized with optional robot clicks."""
        self.scene.create_scene_gui()
        if self.scene.num_envs <= 1:
            return
        # mjviser keeps its slider local; access the pinned Viser handle registry
        # here so clicks use the same selection callback as the native control.
        env_slider = next(
            handle for handle in self.scene.server.gui._gui_input_handle_from_uuid.values()
            if isinstance(handle, viser.GuiSliderHandle) and handle.label == "Select"
        )
        handles = [(group.handle, len(group.body_ids)) for group in self.scene._mesh_groups]
        handles += [(handle, 1) for handle, _ in self.scene._hull_dynamic_handles]
        callbacks = []
        for handle, bodies_per_env in handles:
            def select_env(event, count=bodies_per_env) -> None:
                if event.instance_index is None or self.scene.show_only_selected:
                    return
                # Instances are flattened in environment, then body order.
                env_slider.value = int(event.instance_index) // count
            callbacks.append((handle, select_env))

        environment_folder = self.scene.server.gui._container_handle_from_uuid[env_slider._impl.parent_container_id]
        with environment_folder:
            cursor_select = self.scene.server.gui.add_checkbox(
                "Cursor select", initial_value=False, hint="Click a robot to select its environment."
            )

        @cursor_select.on_update
        def _(_event) -> None:
            for handle, callback in callbacks:
                if cursor_select.value:
                    handle.on_click(callback)
                else:
                    handle.remove_click_callback(callback)

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

    def update(self, selector, env_idx: int, *, capture: bool = False) -> tuple[np.ndarray, np.ndarray] | None:
        """Transfer poses once, including hidden environments when capturing a frame."""
        read_selector = slice(None) if capture else selector
        position, rotation = self.body_poses(read_selector)
        self._positions[read_selector, 1:] = position
        self._rotations[read_selector, 1:] = rotation
        if not capture:
            self.scene.show_only_selected = position.shape[0] != len(self._positions)

        # mjviser caches these arrays and serializes poses on the websocket thread.
        poses = self._positions.copy(), self._rotations.copy()
        self.scene.update_from_arrays(*poses, env_idx=env_idx)
        return poses if capture else None
