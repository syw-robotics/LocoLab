# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Debug drawings for the browser viewer.

Isaac's ``set_debug_vis`` updates viewport markers. Those markers are not part of
the URDF scene, so a headless browser never sees them. This module reads the same
quantities and draws them in Viser: velocity arrows from a command term, ray-hit
points from a caster such as a height scanner, and a frame at each terrain origin.
Ray points use that caster's ``visualizer_cfg`` color and radius.
"""

from __future__ import annotations

import numpy as np
import trimesh

_SHAFT_RADIUS = 0.05
_HEAD_RADIUS = 0.10
_HEAD_FRACTION = 0.30
_MIN_ARROW_LENGTH = 0.05
_MAX_RAY_POINTS = 1000
_GOAL_COLOR = (40, 170, 60)
_CURRENT_COLOR = (210, 50, 40)
# Used only when a caster cfg has no marker color. Task configs set ``visualizer_cfg``.
_RAY_COLOR = (240, 170, 40)
_RAY_RADIUS_M = 0.02
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
    manager = getattr(unwrapped, "command_manager", None)
    terms = getattr(manager, "_terms", None)
    if isinstance(terms, dict):
        for name, term in terms.items():
            if _drawable(term):
                found.append((f"command/{name}", term))
    return found


class DebugScene:
    """Batched arrow and point meshes under ``/debug``."""

    def __init__(self, server) -> None:
        self._server = server
        self._batches: dict[str, _Batch] = {}
        self._frames: dict[str, object] = {}
        self._meshes = {
            "shaft": _unit_shaft(),
            "head": _unit_head(),
            "point": trimesh.creation.icosphere(subdivisions=1, radius=1.0),
        }

    def show(self, name: str, kind: str, positions, wxyzs, scales, color: tuple[int, int, int]) -> None:
        batch = self._batches.get(name)
        if batch is None or batch.kind != kind:
            if batch is not None:
                batch.remove()
            mesh = self._meshes[kind]
            batch = _Batch(name, kind, np.asarray(mesh.vertices), np.asarray(mesh.faces))
            self._batches[name] = batch
        batch.sync(self._server, positions, wxyzs, scales, color)

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
                f"/debug/{name}",
                batched_wxyzs=wxyz,
                batched_positions=points,
                axes_length=_FRAME_LENGTH_M,
                axes_radius=_FRAME_RADIUS_M,
            )
            return
        handle.batched_positions = points
        handle.batched_wxyzs = wxyz

    def clear(self, label: str | None = None) -> None:
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
            scene.clear(label)
            continue
        try:
            if hasattr(obj, "vel_command_b"):
                _draw_velocity(scene, label, obj, selector)
            elif hasattr(obj, "terrain_origins"):
                _draw_terrain(scene, label, obj)
            else:
                _draw_rays(scene, label, obj, selector)
        except Exception as exc:
            if label not in _warned:
                _warned.add(label)
                print(f"[WARN] debug_viz {label}: {exc}")
            scene.clear(label)


def _drawable(obj) -> bool:
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
    scene.show_frames(f"{label}/origin", _numpy(origins, slice(None)).reshape(-1, 3))


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
    origin = _numpy(data.root_pos_w, selector) + _marker_offset(obj)
    quat = _numpy(data.root_quat_w, selector)
    gain = _length_gain(obj)
    _draw_arrow(
        scene,
        f"{label}/goal",
        origin,
        quat,
        _numpy(obj.vel_command_b, selector)[:, :2],
        gain,
        _marker_color(obj, "goal_vel_visualizer_cfg", _GOAL_COLOR),
    )
    _draw_arrow(
        scene,
        f"{label}/current",
        origin,
        quat,
        _numpy(data.root_lin_vel_b, selector)[:, :2],
        gain,
        _marker_color(obj, "current_vel_visualizer_cfg", _CURRENT_COLOR),
    )


def _draw_arrow(scene, name, origin, quat, xy, gain: float, color: tuple[int, int, int]) -> None:
    speed = np.linalg.norm(xy, axis=1)
    count = len(speed)
    if count == 0:
        scene.clear(name)
        return
    direction_b = np.zeros((count, 3))
    moving = speed >= 1e-4
    direction_b[moving, 0] = xy[moving, 0] / speed[moving]
    direction_b[moving, 1] = xy[moving, 1] / speed[moving]
    direction_b[~moving, 0] = 1.0
    direction_w = _quat_apply(quat, direction_b)
    direction_w /= np.maximum(np.linalg.norm(direction_w, axis=1, keepdims=True), 1e-8)
    length = np.where(moving, speed * gain, _MIN_ARROW_LENGTH)
    shaft_len = length * (1.0 - _HEAD_FRACTION)
    head_len = length * _HEAD_FRACTION
    wxyz = _quats_from_z(direction_w).astype(np.float32)
    shaft_pos = origin.astype(np.float32)
    head_pos = (origin + direction_w * shaft_len[:, None]).astype(np.float32)
    shaft_scale = np.column_stack((np.full(count, _SHAFT_RADIUS), np.full(count, _SHAFT_RADIUS), shaft_len)).astype(
        np.float32
    )
    head_scale = np.column_stack((np.full(count, _HEAD_RADIUS), np.full(count, _HEAD_RADIUS), head_len)).astype(
        np.float32
    )
    scene.show(f"{name}/shaft", "shaft", shaft_pos, wxyz, shaft_scale, color)
    scene.show(f"{name}/head", "head", head_pos, wxyz, head_scale, color)


def _draw_rays(scene: DebugScene, label: str, obj, selector) -> None:
    hits = _numpy(_ray_hits(obj), selector).reshape(-1, 3)
    finite = np.isfinite(hits).all(axis=1)
    points = hits[finite]
    if len(points) > _MAX_RAY_POINTS:
        points = points[:: int(np.ceil(len(points) / _MAX_RAY_POINTS))]
    if len(points) == 0:
        scene.clear(label)
        return
    count = len(points)
    color, radius = _ray_appearance(obj)
    scene.show(
        f"{label}/points",
        "point",
        points.astype(np.float32),
        np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (count, 1)),
        np.full((count, 3), radius, dtype=np.float32),
        color,
    )


def _numpy(value, selector) -> np.ndarray:
    if hasattr(value, "torch"):
        value = value.torch
    value = value[selector]
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=np.float64)


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


def _marker_color(obj, attr: str, default: tuple[int, int, int]) -> tuple[int, int, int]:
    marker = _named_marker(obj, attr, "arrow")
    if marker is None:
        return default
    return _diffuse_rgb(getattr(marker, "visual_material", None), default)


def _ray_appearance(obj) -> tuple[tuple[int, int, int], float]:
    """Hit-point color and radius from the caster's ``visualizer_cfg``."""
    marker = _named_marker(obj, "visualizer_cfg", "hit")
    if marker is None:
        return _RAY_COLOR, _RAY_RADIUS_M
    return _diffuse_rgb(getattr(marker, "visual_material", None), _RAY_COLOR), _marker_radius(marker)


def _named_marker(obj, attr: str, name: str):
    cfg = getattr(obj, "cfg", None)
    vis = getattr(cfg, attr, None)
    markers = getattr(vis, "markers", None)
    if not isinstance(markers, dict) or not markers:
        return None
    marker = markers.get(name)
    if marker is None and name == "hit":
        marker = next(iter(markers.values()))
    return marker


def _marker_radius(marker) -> float:
    radius = getattr(marker, "radius", None)
    if radius is not None:
        return float(radius)
    scale = getattr(marker, "scale", None)
    if scale is not None and len(scale) > 0:
        return float(scale[0])
    return _RAY_RADIUS_M


def _diffuse_rgb(material, default: tuple[int, int, int]) -> tuple[int, int, int]:
    color = getattr(material, "diffuse_color", None)
    if color is None or len(color) < 3:
        return default
    channels = [float(channel) for channel in color[:3]]
    if max(channels) > 1.0:
        return tuple(int(np.clip(channel, 0.0, 255.0)) for channel in channels)
    return tuple(int(np.clip(channel, 0.0, 1.0) * 255.0) for channel in channels)


def _quat_apply(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """Rotate vectors by wxyz quaternions. Both arrays are shaped ``(N, *)``."""
    xyz = quat[:, 1:]
    t = np.cross(xyz, vec) * 2.0
    return vec + quat[:, :1] * t + np.cross(xyz, t)


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
    mesh = trimesh.creation.cylinder(radius=1.0, height=1.0, sections=10)
    mesh.apply_translation((0.0, 0.0, 0.5))
    return mesh


def _unit_head() -> trimesh.Trimesh:
    return trimesh.creation.cone(radius=1.0, height=1.0, sections=10)


class _Batch:
    def __init__(self, name: str, kind: str, vertices: np.ndarray, faces: np.ndarray) -> None:
        self.kind = kind
        self._name = name
        self._vertices = np.asarray(vertices, dtype=np.float32)
        self._faces = np.asarray(faces, dtype=np.int32)
        self._handle = None
        self._count = 0

    def sync(self, server, positions, wxyzs, scales, color: tuple[int, int, int]) -> None:
        count = len(positions)
        if count == 0:
            self.remove()
            return
        if self._handle is None or count != self._count:
            self.remove()
            self._handle = server.scene.add_batched_meshes_simple(
                f"/debug/{self._name}",
                self._vertices,
                self._faces,
                batched_wxyzs=wxyzs,
                batched_positions=positions,
                batched_scales=scales,
                batched_colors=color,
                opacity=1.0,
                flat_shading=True,
                cast_shadow=False,
                receive_shadow=False,
            )
            self._count = count
            return
        self._handle.batched_positions = positions
        self._handle.batched_wxyzs = wxyzs
        self._handle.batched_scales = scales

    def remove(self) -> None:
        if self._handle is not None:
            self._handle.remove()
            self._handle = None
            self._count = 0
