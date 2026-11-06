# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Debug drawings for the browser viewer.

Isaac's ``set_debug_vis`` updates viewport markers. Those markers are not part of
the mjviser scene, so a headless browser never sees them. This module reads the same
quantities and draws them in Viser: velocity arrows from a command term, ray-hit
points from a caster such as a height scanner, terrain-origin frames, and virtual
obstacle edge cylinders and foot volume points.
"""

from __future__ import annotations

import inspect

import numpy as np
import trimesh

from locolab.assets.sensors.volume_points import VolumePoints
from locolab.utils.debug_viz.markers import BLUE_RAY_CASTER_MARKER_CFG
from locolab.utils.debug_viz.push.push_debug_vis import PushDebugVis, get_push_debug_vis

from .utils import as_numpy, rotate_wxyz, xyzw_to_wxyz

if (hit := BLUE_RAY_CASTER_MARKER_CFG.markers.get("hit")) is not None:
    _BLUE_RAY_HIT_RADIUS = float(getattr(hit, "radius", 0.02))
    diffuse = getattr(getattr(hit, "visual_material", None), "diffuse_color", (0.2, 0.2, 0.9))
    _BLUE_RAY_HIT_COLOR = tuple(int(np.clip(float(c), 0.0, 1.0) * 255.0) for c in diffuse[:3])
else:
    _BLUE_RAY_HIT_RADIUS = 0.02
    _BLUE_RAY_HIT_COLOR = (51, 51, 230)

_SHAFT_RADIUS = 0.05
_HEAD_RADIUS = 0.10
_HEAD_FRACTION = 0.30
_MIN_ARROW_LENGTH = 0.05
_MAX_RAY_POINTS = 10000
_GOAL_COLOR = (40, 170, 60)
_CURRENT_COLOR = (210, 50, 40)
# ``FRAME_MARKER_CFG`` uses scale 0.5 on a unit frame, so the axes are half a meter.
_FRAME_LENGTH_M = 0.5
_FRAME_RADIUS_M = 0.012

_warned: set[str] = set()


def debug_sources(env) -> list[tuple[str, object]]:
    """Command terms and scene sensors this viewer knows how to draw."""
    unwrapped = env.unwrapped
    found: list[tuple[str, object]] = []
    scene = getattr(unwrapped, "scene", None)
    if scene is not None and hasattr(scene, "keys"):
        for name in scene.keys():
            try:
                obj = scene[name]
            except Exception:
                continue
            if _drawable(obj):
                found.append((str(name), obj))
    terrain = getattr(scene, "terrain", None)
    for name, obstacle in getattr(terrain, "virtual_obstacles", {}).items():
        if hasattr(obstacle, "edges_pyt"):
            found.append((f"virtual_obstacle/{name}", obstacle))
    manager = getattr(unwrapped, "command_manager", None)
    terms = getattr(manager, "_terms", None)
    if isinstance(terms, dict):
        for name, term in terms.items():
            if _drawable(term):
                found.append((f"command/{name}", term))
    manager = getattr(unwrapped, "event_manager", None)
    if manager is not None:
        for names in manager.active_terms.values():
            for name in names:
                cfg = manager.get_term_cfg(name)
                if (
                    getattr(cfg.func, "__name__", "") == "push_by_setting_velocity"
                    and "debug_vis" in inspect.signature(cfg.func).parameters
                ):
                    asset_cfg = cfg.params.get("asset_cfg")
                    obj = get_push_debug_vis(
                        unwrapped, asset_cfg.name if asset_cfg is not None else "robot", cfg.params.get("debug_vis_cfg")
                    )
                    obj.params = cfg.params
                    obj.set_debug_vis(cfg.params.get("debug_vis", False))
                    found.append((f"event/{name}", obj))
    return found


class DebugScene:
    """Batched arrow and point meshes under ``/debug``."""

    def __init__(self, server) -> None:
        self._server = server
        self._batches: dict[str, _Batch] = {}
        self._frames: dict[str, object] = {}
        self._ghosts: dict[str, tuple] = {}
        self._ghost_hulls = False
        self.distance_lod = True
        self._meshes = {
            "shaft": _unit_shaft(),
            "head": _unit_head(),
            "point": trimesh.creation.icosphere(subdivisions=2, radius=1.0),
        }

    def show(
        self, name: str, kind: str, positions, wxyzs, scales, color: tuple[int, int, int],
        opacity: float = 1.0, *, opacities=None,
    ) -> None:
        batch = self._batches.get(name)
        if batch is None or batch.kind != kind:
            if batch is not None:
                batch.remove()
            mesh = self._meshes[kind]
            batch = _Batch(name, kind, np.asarray(mesh.vertices), np.asarray(mesh.faces))
            self._batches[name] = batch
        batch.sync(self._server, positions, wxyzs, scales, color, opacity, opacities=opacities)

    def show_frames(self, name: str, positions) -> None:
        points = np.asarray(positions, dtype=np.float32).reshape(-1, 3)
        count = len(points)
        if count == 0:
            self._drop_frame(name)
            return
        wxyz = np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (count, 1))
        handle = self._frames.get(name)
        if handle is None or len(handle.batched_positions) != count:
            self._drop_frame(name)
            self._frames[name] = self._server.scene.add_batched_axes(
                f"/fixed_bodies/debug/{name}",
                batched_wxyzs=wxyz,
                batched_positions=points,
                axes_length=_FRAME_LENGTH_M,
                axes_radius=_FRAME_RADIUS_M,
            )
            return
        handle.batched_positions = points
        handle.batched_wxyzs = wxyz

    def show_obstacle(self, label: str, obstacle) -> None:
        """Upload static edge cylinders once, until their debug layer is cleared."""
        name = f"{label}/edges"
        if name in self._batches:
            return
        edges = as_numpy(obstacle.edges_pyt, slice(None)).reshape(-1, 6)
        directions = edges[:, 3:] - edges[:, :3]
        lengths = np.linalg.norm(directions, axis=1)
        valid = lengths > 1e-8
        if not np.any(valid):
            return
        lengths = lengths[valid]
        directions = directions[valid] / lengths[:, None]
        radius = obstacle.cfg.cylinder_radius
        _, color = _sphere_style_from_marker_cfg(obstacle.cfg.visualizer, "cylinder", radius, (204, 25, 25))
        marker = obstacle.cfg.visualizer.markers.get("cylinder")
        material = getattr(marker, "visual_material", None)
        opacity = float(getattr(material, "opacity", 0.5))
        # The shared shaft mesh spans z=[0, 1], so position it at the edge start.
        self.show(
            name,
            "shaft",
            edges[valid, :3].astype(np.float32),
            _quats_from_z(directions).astype(np.float32),
            np.column_stack((np.full(len(lengths), radius), np.full(len(lengths), radius), lengths)).astype(np.float32),
            color,
            opacity=opacity,
        )

    def set_ghost_hulls(self, enabled: bool) -> None:
        """Switch browser geometry only when mjviser's hull option changes."""
        if enabled == self._ghost_hulls:
            return
        self._ghost_hulls = enabled
        for _ghost, handles in self._ghosts.values():
            for handle in handles:
                handle.remove()
            handles.clear()

    def show_motion(self, label: str, command, selector) -> None:
        """Instance the reference robot's meshes at its current clip poses."""
        if label not in self._ghosts:
            self._ghosts[label] = (command.get_motion_ghost(), [])
        ghost, handles = self._ghosts[label]
        meshes = ghost.bodies.hull_meshes if self._ghost_hulls else ghost.bodies.meshes
        positions, wxyz = ghost.poses(command, selector)
        count = len(positions)
        if handles and len(handles[0].batched_positions) != count:
            for handle in handles:
                handle.remove()
            handles.clear()
        for index, (mesh, link) in enumerate(zip(meshes, ghost.bodies.mesh_links)):
            # Match mjviser's visual LOD policy; hulls keep their geometry.
            lod_ratio = 1000.0 / len(mesh.vertices)
            lod = ((2.0, lod_ratio),) if self.distance_lod and not self._ghost_hulls and lod_ratio < 0.5 else "off"
            if index == len(handles):
                handles.append(self._server.scene.add_batched_meshes_simple(
                    f"/fixed_bodies/debug/{label}/ghost/{index}",
                    np.asarray(mesh.vertices, dtype=np.float32),
                    np.asarray(mesh.faces, dtype=np.uint32),
                    batched_positions=positions[:, link].astype(np.float32),
                    batched_wxyzs=wxyz[:, link].astype(np.float32),
                    batched_colors=tuple(int(c * 255) for c in ghost.color),
                    opacity=ghost.opacity,
                    cast_shadow=False,
                    receive_shadow=False,
                    lod=lod,
                ))
            else:
                if handles[index].lod != lod:
                    handles[index].lod = lod
                handles[index].visible = True
                handles[index].batched_positions = positions[:, link].astype(np.float32)
                handles[index].batched_wxyzs = wxyz[:, link].astype(np.float32)

    def hide(self, label: str) -> None:
        """Keep ghost meshes cached while disabled; other debug primitives are discarded."""
        if label in self._ghosts:
            for handle in self._ghosts[label][1]:
                handle.visible = False
        else:
            self.clear(label)

    def clear(self, label: str | None = None) -> None:
        for name in list(self._ghosts):
            if label is None or name == label:
                for handle in self._ghosts.pop(name)[1]:
                    handle.remove()
        if label is None:
            names = list(self._batches)
            frames = list(self._frames)
        else:
            prefix = label + "/"
            names = [name for name in self._batches if name.startswith(prefix)]
            frames = [name for name in self._frames if name.startswith(prefix)]
        for name in names:
            self._batches.pop(name).remove()
        for name in frames:
            self._drop_frame(name)

    def _drop_frame(self, name: str) -> None:
        handle = self._frames.pop(name, None)
        if handle is not None:
            handle.remove()


def draw_debug(scene: DebugScene, terms: list[tuple[str, object]], enabled: dict[str, bool], selector) -> None:
    """Refresh every enabled term. ``selector`` matches the robots currently shown."""
    for label, obj in terms:
        if not enabled.get(label, False):
            scene.hide(label)
            continue
        try:
            if isinstance(obj, PushDebugVis):
                _draw_push(scene, label, obj, selector)
            elif hasattr(obj, "motion") and hasattr(obj, "time_steps"):
                scene.show_motion(label, obj, selector)
            elif hasattr(obj, "vel_command_b"):
                _draw_velocity(scene, label, obj, selector)
            elif hasattr(obj, "terrain_origins"):
                _draw_terrain(scene, label, obj)
            elif hasattr(obj, "edges_pyt"):
                scene.show_obstacle(label, obj)
            elif isinstance(obj, VolumePoints):
                _draw_volume_points(scene, label, obj, selector)
            else:
                _draw_rays(scene, label, obj, selector)
        except Exception as exc:
            if label not in _warned:
                _warned.add(label)
                print(f"[WARN] debug_viz {label}: {exc}")
            scene.clear(label)


def _drawable(obj) -> bool:
    if isinstance(obj, VolumePoints):
        return True
    if hasattr(obj, "motion") and hasattr(obj, "time_steps"):
        return True
    if hasattr(obj, "vel_command_b"):
        robot = getattr(obj, "robot", None)
        return getattr(robot, "data", None) is not None
    if _terrain_origins(obj) is not None:
        return True
    return _ray_hits(obj) is not None


def _terrain_origins(obj):
    """Sub-terrain origins, or env origins when the terrain is a single plane."""
    if not hasattr(obj, "terrain_origins"):
        return None
    origins = obj.terrain_origins
    if origins is None:
        origins = getattr(obj, "env_origins", None)
    return origins


def _draw_terrain(scene: DebugScene, label: str, obj) -> None:
    origins = _terrain_origins(obj)
    if origins is None:
        scene.clear(label)
        return
    scene.show_frames(f"{label}/origin", as_numpy(origins, slice(None)).reshape(-1, 3))


def _ray_hits(obj):
    data = getattr(obj, "data", None)
    if data is None:
        return None
    try:
        return getattr(data, "ray_hits_w", None)
    except Exception:
        return None


def _draw_velocity(scene: DebugScene, label: str, obj, selector) -> None:
    robot = obj.robot
    if hasattr(robot, "is_initialized") and not robot.is_initialized:
        scene.clear(label)
        return
    data = robot.data
    origin = as_numpy(data.root_pos_w, selector) + _marker_offset(obj)
    quat = xyzw_to_wxyz(as_numpy(data.root_quat_w, selector))
    gain = _length_gain(obj)
    _draw_arrow(
        scene,
        f"{label}/goal",
        origin,
        quat,
        as_numpy(obj.vel_command_b, selector)[:, :2],
        gain,
        _marker_color(obj, "goal_vel_visualizer_cfg", _GOAL_COLOR),
    )
    _draw_arrow(
        scene,
        f"{label}/current",
        origin,
        quat,
        as_numpy(data.root_lin_vel_b, selector)[:, :2],
        gain,
        _marker_color(obj, "current_vel_visualizer_cfg", _CURRENT_COLOR),
    )


def _draw_push(scene, label, obj, selector) -> None:
    opacity = as_numpy(obj.opacity(selector), slice(None))
    active = opacity > 0.0
    origin = as_numpy(obj.robot.data.root_link_pos_w, selector)[active]
    velocity = as_numpy(obj.velocity_w, selector)[active]
    quat = np.tile([1.0, 0.0, 0.0, 0.0], (len(velocity), 1))
    scale = obj.cfg.visualizer_cfg.markers["arrow"].scale
    # Match Newton's unit arrow: shaft/head radii are 0.2/0.4 times the marker's Y/Z scale.
    _draw_arrow(
        scene, f"{label}/push", origin, quat, velocity, scale[0],
        _marker_color(obj, "visualizer_cfg", (51, 51, 229)), opacities=opacity[active],
        shaft_radii=(scale[1] * 0.2, scale[2] * 0.2),
        head_radii=(scale[1] * 0.4, scale[2] * 0.4),
    )


def _draw_arrow(
    scene, name, origin, quat, xy, gain: float, color: tuple[int, int, int], *, opacities=None,
    shaft_radii=(_SHAFT_RADIUS, _SHAFT_RADIUS), head_radii=(_HEAD_RADIUS, _HEAD_RADIUS),
) -> None:
    speed = np.linalg.norm(xy, axis=1)
    count = len(speed)
    if count == 0:
        scene.clear(name)
        return
    direction_b = np.zeros((count, 3))
    moving = speed >= 1e-4
    direction_b[moving, :xy.shape[1]] = xy[moving] / speed[moving, None]
    direction_b[~moving, 0] = 1.0
    direction_w = rotate_wxyz(quat, direction_b)
    direction_w /= np.maximum(np.linalg.norm(direction_w, axis=1, keepdims=True), 1e-8)
    length = np.where(moving, speed * gain, _MIN_ARROW_LENGTH)
    shaft_len = length * (1.0 - _HEAD_FRACTION)
    head_len = length * _HEAD_FRACTION
    wxyz = _quats_from_z(direction_w).astype(np.float32)
    shaft_pos = origin.astype(np.float32)
    head_pos = (origin + direction_w * shaft_len[:, None]).astype(np.float32)
    shaft_scale = np.column_stack((np.full(count, shaft_radii[0]), np.full(count, shaft_radii[1]), shaft_len)).astype(
        np.float32
    )
    head_scale = np.column_stack((np.full(count, head_radii[0]), np.full(count, head_radii[1]), head_len)).astype(
        np.float32
    )
    style = {} if opacities is None else {"opacities": opacities}
    scene.show(f"{name}/shaft", "shaft", shaft_pos, wxyz, shaft_scale, color, **style)
    scene.show(f"{name}/head", "head", head_pos, wxyz, head_scale, color, **style)


def _draw_rays(scene: DebugScene, label: str, obj, selector) -> None:
    hits = as_numpy(_ray_hits(obj), selector).reshape(-1, 3)
    finite = np.isfinite(hits).all(axis=1)
    points = hits[finite]
    if len(points) > _MAX_RAY_POINTS:
        points = points[:: int(np.ceil(len(points) / _MAX_RAY_POINTS))]
    if len(points) == 0:
        scene.clear(label)
        return
    count = len(points)
    radius, ray_color = _ray_hit_style(obj)
    scene.show(
        f"{label}/points",
        "point",
        points.astype(np.float32),
        np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (count, 1)),
        np.full((count, 3), radius, dtype=np.float32),
        ray_color,
    )


def _draw_volume_points(scene: DebugScene, label: str, obj: VolumePoints, selector) -> None:
    data = obj.data
    points = as_numpy(data.points_pos_w, selector).reshape(-1, 3)
    offsets = as_numpy(data.penetration_offset, selector).reshape(-1, 3)
    penetrated = np.linalg.norm(offsets, axis=1) > 0.0
    finite = np.isfinite(points).all(axis=1)
    for marker_name, mask, default_color in (
        ("sphere", finite & ~penetrated, (0, 255, 0)),
        ("sphere_penetrated", finite & penetrated, (255, 0, 0)),
    ):
        positions = points[mask].astype(np.float32)
        radius, color = _sphere_style_from_marker_cfg(obj.cfg.visualizer_cfg, marker_name, 0.01, default_color)
        count = len(positions)
        scene.show(
            f"{label}/{marker_name}",
            "point",
            positions,
            np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (count, 1)),
            np.full((count, 3), radius, dtype=np.float32),
            color,
        )


def _marker_offset(obj) -> np.ndarray:
    cfg = getattr(obj, "cfg", None)
    if cfg is not None and hasattr(cfg, "vel_visualizer_offset_z"):
        return np.array([0.0, 0.0, float(cfg.vel_visualizer_offset_z)])
    raw = getattr(cfg, "marker_pos_offset", (0.0, 0.0, 0.5))
    return np.asarray(raw, dtype=np.float64).reshape(3)


def _length_gain(obj) -> float:
    """Match ``arrow_scale[:, 0] *= speed * 3`` in the velocity command."""
    cfg = getattr(obj, "cfg", None)
    vis = getattr(cfg, "goal_vel_visualizer_cfg", None)
    markers = getattr(vis, "markers", None)
    arrow = markers.get("arrow") if isinstance(markers, dict) else None
    scale = getattr(arrow, "scale", (0.5, 0.5, 0.5))
    return float(scale[0])


def _ray_hit_style(obj) -> tuple[float, tuple[int, int, int]]:
    cfg = getattr(obj, "cfg", None)
    vis = getattr(cfg, "visualizer_cfg", None)
    return _sphere_style_from_marker_cfg(vis, "hit", _BLUE_RAY_HIT_RADIUS, _BLUE_RAY_HIT_COLOR)


def _sphere_style_from_marker_cfg(
    visualizer_cfg,
    marker_name: str,
    default_radius: float,
    default_color: tuple[int, int, int],
) -> tuple[float, tuple[int, int, int]]:
    markers = getattr(visualizer_cfg, "markers", None)
    marker = markers.get(marker_name) if isinstance(markers, dict) else None
    radius = float(getattr(marker, "radius", default_radius))
    material = getattr(marker, "visual_material", None)
    diffuse = getattr(material, "diffuse_color", None)
    if diffuse is None or len(diffuse) < 3:
        return radius, default_color
    color = tuple(int(np.clip(float(channel), 0.0, 1.0) * 255.0) for channel in diffuse[:3])
    return radius, color


def _marker_color(obj, attr: str, default: tuple[int, int, int]) -> tuple[int, int, int]:
    cfg = getattr(obj, "cfg", None)
    vis = getattr(cfg, attr, None)
    markers = getattr(vis, "markers", None)
    arrow = markers.get("arrow") if isinstance(markers, dict) else None
    color = getattr(getattr(arrow, "visual_material", None), "diffuse_color", None)
    if color is None or len(color) < 3:
        return default
    return tuple(int(np.clip(float(channel), 0.0, 1.0) * 255.0) for channel in color[:3])


def _quats_from_z(directions: np.ndarray) -> np.ndarray:
    """Wxyz rotation taking +Z onto each unit direction."""
    z_axis = np.zeros_like(directions)
    z_axis[:, 2] = 1.0
    cross = np.cross(z_axis, directions)
    dot = np.clip(directions[:, 2], -1.0, 1.0)
    quat = np.empty((len(directions), 4))
    quat[:, 0] = 1.0 + dot
    quat[:, 1:] = cross
    quat[dot < -1.0 + 1e-6] = (0.0, 1.0, 0.0, 0.0)
    norm = np.linalg.norm(quat, axis=1, keepdims=True)
    return quat / np.maximum(norm, 1e-12)


def _unit_shaft() -> trimesh.Trimesh:
    mesh = trimesh.creation.cylinder(radius=1.0, height=1.0, sections=16)
    mesh.apply_translation((0.0, 0.0, 0.5))
    return mesh


def _unit_head() -> trimesh.Trimesh:
    return trimesh.creation.cone(radius=1.0, height=1.0, sections=16)


class _Batch:
    def __init__(self, name: str, kind: str, vertices: np.ndarray, faces: np.ndarray) -> None:
        self.kind = kind
        self._name = name
        self._vertices = np.asarray(vertices, dtype=np.float32)
        self._faces = np.asarray(faces, dtype=np.int32)
        self._handle = None
        self._count = 0

    def sync(
        self, server, positions, wxyzs, scales, color: tuple[int, int, int], opacity: float = 1.0, *, opacities=None
    ) -> None:
        count = len(positions)
        if count == 0:
            self.remove()
            return
        if self._handle is None or count != self._count:
            self.remove()
            self._handle = server.scene.add_batched_meshes_simple(
                f"/fixed_bodies/debug/{self._name}",
                self._vertices,
                self._faces,
                batched_wxyzs=wxyzs,
                batched_positions=positions,
                batched_scales=scales,
                batched_colors=color,
                opacity=opacity,
                flat_shading=False,
                cast_shadow=False,
                receive_shadow=False,
            )
            self._count = count
            if opacities is not None:
                self._handle.batched_opacities = np.asarray(opacities, dtype=np.float32)
            return
        self._handle.batched_positions = positions
        self._handle.batched_wxyzs = wxyzs
        self._handle.batched_scales = scales
        if opacities is not None:
            self._handle.batched_opacities = np.asarray(opacities, dtype=np.float32)

    def remove(self) -> None:
        if self._handle is not None:
            self._handle.remove()
            self._handle = None
            self._count = 0
