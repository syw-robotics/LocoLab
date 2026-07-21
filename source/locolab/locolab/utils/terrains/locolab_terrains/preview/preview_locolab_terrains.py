"""Generate offline previews for all LocoLab custom terrains.

This script intentionally does not touch training terrain configs. It builds one
representative config per custom terrain, renders PNG previews with matplotlib,
and writes an index.html gallery.
"""

from __future__ import annotations

import argparse
import copy
import html
import importlib
import os
import sys
import types
from dataclasses import MISSING
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-locolab-preview")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from matplotlib import colors
from matplotlib.collections import LineCollection, PolyCollection
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

from preview_locolab_terrains_cfg import ABSTRACT_CFG_NAMES, PREVIEW_CASES


# Paths are fixed relative to this preview folder, so the script can be run
# either from this directory or from the parent terrains package.
PREVIEW_DIR = Path(__file__).resolve().parent
PACKAGE_PARENT_DIR = PREVIEW_DIR.parent.parent
README_PATH = PREVIEW_DIR.parent / "README.md"
DEFAULT_OUT_DIR = PREVIEW_DIR / "terrain_previews"
README_BEGIN = "<!-- BEGIN_AUTO_PREVIEWS -->"
README_END = "<!-- END_AUTO_PREVIEWS -->"
FIGURE_BG = "#f7f5ef"
SURFACE_EDGE = "#23231f"
SIDE_FACE_COLOR = "#c9c1b2"
HEIGHT_CMAP = colors.LinearSegmentedColormap.from_list(
    "locolab_height",
    ["#24364b", "#316d72", "#8ca980", "#d7c48d", "#b86b4d"],
)


# ---------------------------------------------------------------------------
# Preview-only IsaacLab compatibility layer
# ---------------------------------------------------------------------------
#
# A complete IsaacLab import usually needs Omniverse modules such as carb/omni.
# The terrain functions only need a much smaller API surface, so the fallback
# below lets this preview script run on machines without a full IsaacSim app.


def _configclass(cls):
    """Minimal stand-in for IsaacLab's configclass when Omniverse is unavailable."""
    annotations: dict[str, object] = {}
    for base in reversed(cls.__mro__):
        annotations.update(getattr(base, "__annotations__", {}))
    if "function" not in annotations and hasattr(cls, "function"):
        annotations["function"] = object

    def __init__(self, **kwargs):
        for name in annotations:
            if name in kwargs:
                value = kwargs.pop(name)
            else:
                value = getattr(cls, name, MISSING)
            if value is not MISSING:
                setattr(self, name, copy.deepcopy(value))
        for name, value in kwargs.items():
            setattr(self, name, value)

    def cfg_copy(self):
        return copy.deepcopy(self)

    def to_dict(self):
        return copy.deepcopy(self.__dict__)

    cls.__init__ = __init__
    cls.copy = cfg_copy
    cls.to_dict = to_dict
    return cls


@_configclass
class _SubTerrainBaseCfg:
    function: object = MISSING
    proportion: float = 1.0
    size: tuple[float, float] = (10.0, 10.0)
    flat_patch_sampling: dict | None = None


@_configclass
class _HfTerrainBaseCfg(_SubTerrainBaseCfg):
    border_width: float = 0.0
    horizontal_scale: float = 0.1
    vertical_scale: float = 0.005
    slope_threshold: float | None = None


@_configclass
class _HfRandomUniformTerrainCfg(_HfTerrainBaseCfg):
    noise_range: tuple[float, float] = MISSING
    noise_step: float = MISSING
    downsampled_scale: float | None = None


def _module(name: str, path: Path | None = None) -> types.ModuleType:
    """Create an importable module object for the preview-only IsaacLab stub."""
    mod = types.ModuleType(name)
    if path is not None:
        mod.__path__ = [str(path)]
    sys.modules[name] = mod
    return mod


def _convert_height_field_to_mesh(
    height_field: np.ndarray,
    horizontal_scale: float,
    vertical_scale: float,
    slope_threshold: float | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Mirror IsaacLab's height-field conversion for offline preview fallback."""
    num_rows, num_cols = height_field.shape
    y = np.linspace(0, (num_cols - 1) * horizontal_scale, num_cols)
    x = np.linspace(0, (num_rows - 1) * horizontal_scale, num_rows)
    yy, xx = np.meshgrid(y, x)
    hf = height_field.copy()

    if slope_threshold is not None:
        slope_threshold *= horizontal_scale / vertical_scale
        move_x = np.zeros((num_rows, num_cols))
        move_y = np.zeros((num_rows, num_cols))
        move_corners = np.zeros((num_rows, num_cols))
        move_x[: num_rows - 1, :] += hf[1:num_rows, :] - hf[: num_rows - 1, :] > slope_threshold
        move_x[1:num_rows, :] -= hf[: num_rows - 1, :] - hf[1:num_rows, :] > slope_threshold
        move_y[:, : num_cols - 1] += hf[:, 1:num_cols] - hf[:, : num_cols - 1] > slope_threshold
        move_y[:, 1:num_cols] -= hf[:, : num_cols - 1] - hf[:, 1:num_cols] > slope_threshold
        move_corners[: num_rows - 1, : num_cols - 1] += (
            hf[1:num_rows, 1:num_cols] - hf[: num_rows - 1, : num_cols - 1] > slope_threshold
        )
        move_corners[1:num_rows, 1:num_cols] -= (
            hf[: num_rows - 1, : num_cols - 1] - hf[1:num_rows, 1:num_cols] > slope_threshold
        )
        xx += (move_x + move_corners * (move_x == 0)) * horizontal_scale
        yy += (move_y + move_corners * (move_y == 0)) * horizontal_scale

    vertices = np.zeros((num_rows * num_cols, 3), dtype=np.float32)
    vertices[:, 0] = xx.flatten()
    vertices[:, 1] = yy.flatten()
    vertices[:, 2] = hf.flatten() * vertical_scale

    triangles = -np.ones((2 * (num_rows - 1) * (num_cols - 1), 3), dtype=np.uint32)
    for i in range(num_rows - 1):
        ind0 = np.arange(0, num_cols - 1) + i * num_cols
        ind1 = ind0 + 1
        ind2 = ind0 + num_cols
        ind3 = ind2 + 1
        start = 2 * i * (num_cols - 1)
        stop = start + 2 * (num_cols - 1)
        triangles[start:stop:2, 0] = ind0
        triangles[start:stop:2, 1] = ind3
        triangles[start:stop:2, 2] = ind1
        triangles[start + 1 : stop : 2, 0] = ind0
        triangles[start + 1 : stop : 2, 1] = ind2
        triangles[start + 1 : stop : 2, 2] = ind3
    return vertices, triangles


def _height_field_to_mesh(func):
    """Preview-only version of IsaacLab's height_field_to_mesh decorator."""

    def wrapper(difficulty: float, cfg: _HfTerrainBaseCfg):
        if cfg.border_width > 0 and cfg.border_width < cfg.horizontal_scale:
            raise ValueError(
                f"The border width ({cfg.border_width}) must be greater than or equal to the "
                f"horizontal scale ({cfg.horizontal_scale})."
            )
        width_pixels = int(cfg.size[0] / cfg.horizontal_scale) + 1
        length_pixels = int(cfg.size[1] / cfg.horizontal_scale) + 1
        border_pixels = int(cfg.border_width / cfg.horizontal_scale) + 1
        heights = np.zeros((width_pixels, length_pixels), dtype=np.int16)

        terrain_size = copy.deepcopy(cfg.size)
        sub_terrain_size = (
            (width_pixels - 2 * border_pixels) * cfg.horizontal_scale,
            (length_pixels - 2 * border_pixels) * cfg.horizontal_scale,
        )
        cfg.size = sub_terrain_size
        z_gen = func(difficulty, cfg)
        heights[border_pixels:-border_pixels, border_pixels:-border_pixels] = z_gen
        cfg.size = terrain_size

        vertices, triangles = _convert_height_field_to_mesh(
            heights, cfg.horizontal_scale, cfg.vertical_scale, cfg.slope_threshold
        )
        mesh = trimesh.Trimesh(vertices=vertices, faces=triangles, process=False)
        x1 = int((cfg.size[0] * 0.5 - 1) / cfg.horizontal_scale)
        x2 = int((cfg.size[0] * 0.5 + 1) / cfg.horizontal_scale)
        y1 = int((cfg.size[1] * 0.5 - 1) / cfg.horizontal_scale)
        y2 = int((cfg.size[1] * 0.5 + 1) / cfg.horizontal_scale)
        origin_z = np.max(heights[x1:x2, y1:y2]) * cfg.vertical_scale
        return [mesh], np.array([0.5 * cfg.size[0], 0.5 * cfg.size[1], origin_z])

    return wrapper


def _make_border(
    size: tuple[float, float],
    inner_size: tuple[float, float],
    height: float,
    position: tuple[float, float, float],
) -> list[trimesh.Trimesh]:
    """Preview-only copy of the rectangular border helper used by mesh terrains."""
    thickness_x = (size[0] - inner_size[0]) / 2.0
    thickness_y = (size[1] - inner_size[1]) / 2.0
    if thickness_x <= 0 or thickness_y <= 0:
        raise ValueError(f"Border inner_size {inner_size} must fit inside size {size}.")

    box_dims = (size[0], thickness_y, height)
    top_pos = (position[0], position[1] + inner_size[1] / 2.0 + thickness_y / 2.0, position[2])
    bottom_pos = (position[0], position[1] - inner_size[1] / 2.0 - thickness_y / 2.0, position[2])
    box_mesh_top = trimesh.creation.box(box_dims, trimesh.transformations.translation_matrix(top_pos))
    box_mesh_bottom = trimesh.creation.box(box_dims, trimesh.transformations.translation_matrix(bottom_pos))

    box_dims = (thickness_x, inner_size[1], height)
    left_pos = (position[0] - inner_size[0] / 2.0 - thickness_x / 2.0, position[1], position[2])
    right_pos = (position[0] + inner_size[0] / 2.0 + thickness_x / 2.0, position[1], position[2])
    box_mesh_left = trimesh.creation.box(box_dims, trimesh.transformations.translation_matrix(left_pos))
    box_mesh_right = trimesh.creation.box(box_dims, trimesh.transformations.translation_matrix(right_pos))
    return [box_mesh_left, box_mesh_right, box_mesh_top, box_mesh_bottom]


def _install_isaaclab_preview_stubs():
    """Install the small IsaacLab API surface needed by terrain generation code."""
    isaaclab = _module("isaaclab")
    isaaclab_utils = _module("isaaclab.utils")
    isaaclab_utils.configclass = _configclass
    isaaclab.utils = isaaclab_utils

    terrains = _module("isaaclab.terrains")
    terrains.SubTerrainBaseCfg = _SubTerrainBaseCfg
    isaaclab.terrains = terrains

    hf_pkg = _module("isaaclab.terrains.height_field")
    hf_cfg = _module("isaaclab.terrains.height_field.hf_terrains_cfg")
    hf_cfg.HfTerrainBaseCfg = _HfTerrainBaseCfg
    hf_cfg.HfRandomUniformTerrainCfg = _HfRandomUniformTerrainCfg
    hf_utils = _module("isaaclab.terrains.height_field.utils")
    hf_utils.height_field_to_mesh = _height_field_to_mesh
    hf_pkg.HfTerrainBaseCfg = _HfTerrainBaseCfg
    hf_pkg.hf_terrains_cfg = hf_cfg
    hf_pkg.utils = hf_utils

    trimesh_pkg = _module("isaaclab.terrains.trimesh")
    trimesh_utils = _module("isaaclab.terrains.trimesh.utils")
    trimesh_utils.make_border = _make_border
    trimesh_pkg.utils = trimesh_utils


def _clear_imports():
    """Remove terrain and IsaacLab modules before retrying with preview stubs."""
    for name in list(sys.modules):
        if name == "locolab_terrains" or name.startswith("locolab_terrains."):
            del sys.modules[name]
        elif name == "isaaclab" or name.startswith("isaaclab."):
            del sys.modules[name]


# ---------------------------------------------------------------------------
# Terrain discovery and config construction
# ---------------------------------------------------------------------------


def import_locolab_terrains():
    """Import terrain cfgs, falling back to stubs on machines without Omniverse."""
    sys.path.insert(0, str(PACKAGE_PARENT_DIR))
    try:
        return importlib.import_module("locolab_terrains"), "isaaclab"
    except Exception as exc:
        print(f"[WARN] Could not import through the active IsaacLab environment: {exc}")
        print("[WARN] Falling back to lightweight preview-only IsaacLab stubs.")
        _clear_imports()
        _install_isaaclab_preview_stubs()
        return importlib.import_module("locolab_terrains"), "preview-stub"


def discover_cfgs(pkg) -> dict[str, type]:
    """Find concrete terrain cfg classes exported by locolab_terrains."""
    cfgs = {}
    for name in dir(pkg):
        if name in ABSTRACT_CFG_NAMES or not name.endswith("TerrainCfg"):
            continue
        obj = getattr(pkg, name)
        if isinstance(obj, type):
            cfgs[name] = obj
    return dict(sorted(cfgs.items()))


def is_height_field_cfg(cfg_cls: type) -> bool:
    """Return whether a cfg class is backed by IsaacLab's height-field base cfg."""
    return any(base.__name__ in {"HfTerrainBaseCfg", "_HfTerrainBaseCfg"} for base in cfg_cls.mro())


def build_cfg(cfg_cls: type, common_kwargs: dict, hf_kwargs: dict, case_kwargs: dict):
    """Merge common preview options with terrain-specific example parameters."""
    kwargs = dict(common_kwargs)
    if is_height_field_cfg(cfg_cls):
        kwargs.update(hf_kwargs)
    elif "border_width" in case_kwargs:
        raise RuntimeError(f"{cfg_cls.__name__} is not a height-field terrain but defines border_width.")
    kwargs.update(case_kwargs)
    return cfg_cls(**kwargs)


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------


def assert_mesh(mesh: trimesh.Trimesh, origin: np.ndarray):
    """Fail fast if a generated terrain is not a usable mesh/origin pair."""
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.faces)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) == 0:
        raise RuntimeError("generated mesh has invalid vertices")
    if faces.ndim != 2 or faces.shape[1] != 3 or len(faces) == 0:
        raise RuntimeError("generated mesh has invalid faces")
    if not np.isfinite(vertices).all():
        raise RuntimeError("generated mesh has non-finite vertices")
    if np.asarray(origin).shape != (3,) or not np.isfinite(origin).all():
        raise RuntimeError(f"generated origin is invalid: {origin}")


def height_normalizer(heights: np.ndarray) -> colors.Normalize:
    """Create a stable color scale even when the terrain has near-constant height."""
    min_height = float(np.min(heights))
    max_height = float(np.max(heights))
    if np.isclose(min_height, max_height):
        pad = max(abs(min_height) * 0.05, 0.05)
        min_height -= pad
        max_height += pad
    return colors.Normalize(vmin=min_height, vmax=max_height)


def add_height_colorbar(fig, ax, normalizer: colors.Normalize):
    """Add the compact height legend used by every preview image."""
    scalar_map = plt.cm.ScalarMappable(norm=normalizer, cmap=HEIGHT_CMAP)
    scalar_map.set_array([])
    colorbar = fig.colorbar(scalar_map, ax=ax, fraction=0.045, pad=0.035)
    colorbar.set_label("height (m)", fontsize=8)
    colorbar.ax.tick_params(labelsize=7, length=2)


def collect_crease_segments(vertices: np.ndarray, faces: np.ndarray, face_normals: np.ndarray) -> list[np.ndarray]:
    """Find boundary and sharp edges for readable structure outlines."""
    edge_to_faces: dict[tuple[int, int], list[int]] = {}
    for face_id, face in enumerate(faces):
        for idx0, idx1 in ((0, 1), (1, 2), (2, 0)):
            edge = tuple(sorted((int(face[idx0]), int(face[idx1]))))
            edge_to_faces.setdefault(edge, []).append(face_id)

    crease_segments = []
    for edge, face_ids in edge_to_faces.items():
        draw_edge = len(face_ids) == 1
        if len(face_ids) == 2:
            normal_dot = float(np.dot(face_normals[face_ids[0]], face_normals[face_ids[1]]))
            draw_edge = normal_dot < 0.999
        if draw_edge:
            segment = vertices[list(edge)]
            if not np.allclose(segment[0], segment[1]):
                crease_segments.append(segment)
    return crease_segments


def collect_surface_outline_segments(
    vertices: np.ndarray, faces: np.ndarray, face_normals: np.ndarray
) -> list[np.ndarray]:
    """Find top-surface outlines for 3D mesh previews."""
    edge_to_faces: dict[tuple[int, int], list[int]] = {}
    for face_id, face in enumerate(faces):
        for idx0, idx1 in ((0, 1), (1, 2), (2, 0)):
            edge = tuple(sorted((int(face[idx0]), int(face[idx1]))))
            edge_to_faces.setdefault(edge, []).append(face_id)

    outline_segments = []
    for edge, face_ids in edge_to_faces.items():
        segment = vertices[list(edge)]
        if not np.isclose(segment[0, 2], segment[1, 2]):
            continue

        horizontal_faces = [face_id for face_id in face_ids if face_normals[face_id, 2] > 0.35]
        if not horizontal_faces:
            continue

        draw_edge = len(face_ids) == 1
        if len(face_ids) == 2:
            draw_edge = any(face_normals[face_id, 2] <= 0.35 for face_id in face_ids)
        if draw_edge and not np.allclose(segment[0], segment[1]):
            outline_segments.append(segment)
    return outline_segments


def set_equal_3d_limits(ax, vertices: np.ndarray):
    """Set stable terrain framing for 3D preview images."""
    mins = vertices.min(axis=0)
    maxs = vertices.max(axis=0)
    centers = (mins + maxs) / 2.0
    radius = max((maxs - mins).max() / 2.0, 1e-3)
    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(centers[2] - radius * 0.45, centers[2] + radius * 0.45)
    ax.set_box_aspect((1, 1, 0.45))


def render_mesh_top_view(
    vertices: np.ndarray,
    faces: np.ndarray,
    face_normals: np.ndarray,
    output_path: Path,
    title: str,
    image_size: int,
):
    """Render direct mesh terrains from above without triangle color artifacts."""
    triangles = vertices[faces]
    triangle_heights = triangles[:, :, 2].mean(axis=1)

    horizontal = np.abs(face_normals[:, 2]) > 0.35
    if not horizontal.any():
        raise RuntimeError("mesh preview found no horizontal faces to render")
    polygons = triangles[horizontal, :, :2]
    heights = triangle_heights[horizontal]
    order = np.argsort(heights)
    normalizer = height_normalizer(heights)
    face_colors = HEIGHT_CMAP(normalizer(heights))

    fig, ax = plt.subplots(figsize=(image_size / 100, image_size / 100), dpi=100)
    fig.patch.set_facecolor(FIGURE_BG)
    ax.set_facecolor(FIGURE_BG)
    collection = PolyCollection(
        polygons[order],
        facecolors=face_colors[order],
        edgecolors="none",
        linewidths=0.0,
        antialiaseds=False,
    )
    ax.add_collection(collection)

    crease_segments = [segment[:, :2] for segment in collect_crease_segments(vertices, faces, face_normals)]
    if crease_segments:
        ax.add_collection(
            LineCollection(
                crease_segments,
                colors=SURFACE_EDGE,
                linewidths=0.7,
                antialiaseds=False,
            )
        )

    mins = vertices[:, :2].min(axis=0)
    maxs = vertices[:, :2].max(axis=0)
    padding = max((maxs - mins).max() * 0.04, 0.05)
    ax.set_xlim(mins[0] - padding, maxs[0] + padding)
    ax.set_ylim(mins[1] - padding, maxs[1] + padding)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=9)
    ax.set_axis_off()
    add_height_colorbar(fig, ax, normalizer)
    fig.tight_layout(pad=0.1)
    fig.savefig(output_path, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)


def render_mesh_3d_view(
    vertices: np.ndarray,
    faces: np.ndarray,
    face_normals: np.ndarray,
    output_path: Path,
    title: str,
    image_size: int,
):
    """Render direct mesh terrains in 3D with subdued side faces."""
    triangles = vertices[faces]
    triangle_heights = triangles[:, :, 2].mean(axis=1)

    # Keep height colors on horizontal surfaces only. Coloring vertical triangles
    # by average height makes rectangular walls show distracting diagonal bands.
    horizontal = face_normals[:, 2] > 0.35
    side = np.abs(face_normals[:, 2]) <= 0.35
    if not horizontal.any():
        raise RuntimeError("mesh preview found no upward faces to render")

    normalizer = height_normalizer(triangle_heights[horizontal])
    horizontal_colors = HEIGHT_CMAP(normalizer(triangle_heights[horizontal]))
    side_colors = [colors.to_rgba(SIDE_FACE_COLOR, alpha=0.84)] * int(side.sum())

    fig = plt.figure(figsize=(image_size / 100, image_size / 100), dpi=100)
    fig.patch.set_facecolor(FIGURE_BG)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor(FIGURE_BG)

    if side.any():
        side_collection = Poly3DCollection(
            triangles[side],
            facecolors=side_colors,
            edgecolors="none",
            linewidths=0.0,
            antialiaseds=False,
        )
        ax.add_collection3d(side_collection)

    surface_collection = Poly3DCollection(
        triangles[horizontal],
        facecolors=horizontal_colors,
        edgecolors="none",
        linewidths=0.0,
        antialiaseds=False,
    )
    ax.add_collection3d(surface_collection)

    crease_segments = collect_surface_outline_segments(vertices, faces, face_normals)
    if crease_segments:
        ax.add_collection3d(
            Line3DCollection(
                crease_segments,
                colors=colors.to_rgba(SURFACE_EDGE, alpha=0.78),
                linewidths=0.55,
                antialiaseds=False,
            )
        )

    ax.view_init(elev=34, azim=-135)
    ax.set_title(title, fontsize=9)
    ax.set_axis_off()
    set_equal_3d_limits(ax, vertices)
    add_height_colorbar(fig, ax, normalizer)
    fig.tight_layout(pad=0.1)
    fig.savefig(output_path, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)


def render_height_field_3d(
    vertices: np.ndarray,
    faces: np.ndarray,
    output_path: Path,
    title: str,
    image_size: int,
):
    """Render height-field terrains in 3D."""
    triangles = vertices[faces]
    triangle_heights = triangles[:, :, 2].mean(axis=1)
    normalizer = height_normalizer(triangle_heights)
    face_colors = HEIGHT_CMAP(normalizer(triangle_heights))

    fig = plt.figure(figsize=(image_size / 100, image_size / 100), dpi=100)
    fig.patch.set_facecolor(FIGURE_BG)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor(FIGURE_BG)
    collection = Poly3DCollection(
        triangles,
        facecolors=face_colors,
        edgecolors=face_colors,
        linewidths=0.0,
        antialiaseds=False,
    )
    ax.add_collection3d(collection)
    ax.view_init(elev=34, azim=-135)
    ax.set_title(title, fontsize=9)
    ax.set_axis_off()

    set_equal_3d_limits(ax, vertices)
    add_height_colorbar(fig, ax, normalizer)
    fig.tight_layout(pad=0.1)
    fig.savefig(output_path, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)


def render_mesh(
    mesh: trimesh.Trimesh,
    output_path: Path,
    title: str,
    image_size: int,
    *,
    is_height_field: bool,
    mesh_view: str,
):
    """Render one terrain mesh to a PNG preview."""
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.faces)
    face_normals = np.asarray(mesh.face_normals)

    if is_height_field:
        render_height_field_3d(vertices, faces, output_path, title, image_size)
    elif mesh_view == "3d":
        render_mesh_3d_view(vertices, faces, face_normals, output_path, title, image_size)
    else:
        render_mesh_top_view(vertices, faces, face_normals, output_path, title, image_size)


# ---------------------------------------------------------------------------
# Output files
# ---------------------------------------------------------------------------


def write_index(output_dir: Path, rows: list[dict]):
    """Write a lightweight HTML gallery next to the generated preview files."""
    cards = []
    for row in rows:
        image_rel = html.escape(row["png"].name)
        label = html.escape(row["label"])
        cards.append(
            f'<article><a href="{image_rel}"><img src="{image_rel}" alt="{label}"></a><h2>{label}</h2></article>'
        )
    body = "\n".join(cards)
    (output_dir / "index.html").write_text(
        f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>LocoLab Terrain Previews</title>
  <style>
    body {{ font-family: sans-serif; margin: 24px; background: #f6f7f8; color: #1f2933; }}
    main {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(260px, 1fr)); gap: 16px; }}
    article {{ background: white; border: 1px solid #d8dee4; border-radius: 6px; padding: 10px; }}
    img {{ width: 100%; display: block; background: #eef1f4; }}
    h2 {{ font-size: 14px; margin: 8px 0 4px; }}
    p {{ margin: 0; font-size: 13px; }}
  </style>
</head>
<body>
  <h1>LocoLab Terrain Previews</h1>
  <main>
{body}
  </main>
</body>
</html>
""",
        encoding="utf-8",
    )


def path_relative_to(path: Path, base: Path) -> str:
    """Return a POSIX relative path for Markdown links."""
    return path.resolve().relative_to(base.resolve()).as_posix()


def write_readme_preview_section(readme_path: Path, output_dir: Path, rows: list[dict]):
    """Refresh the generated gallery block in the package README."""
    try:
        rows_rel = [
            {
                "label": row["label"],
                "png": path_relative_to(row["png"], readme_path.parent),
            }
            for row in rows
        ]
    except ValueError:
        print(f"[WARN] Skipping README update because output is outside {readme_path.parent}.")
        return

    lines = [
        README_BEGIN,
        "",
        "This section is generated by `preview/preview_locolab_terrains.py`.",
        "",
        "| Terrain | Preview |",
        "| --- | --- |",
    ]
    for row in rows_rel:
        label = row["label"]
        lines.append(f"| `{label}` | ![{label}]({row['png']}) |")
    lines.extend(["", README_END])
    generated = "\n".join(lines)

    if readme_path.exists():
        content = readme_path.read_text(encoding="utf-8")
    else:
        content = "# LocoLab Terrains\n\n## Preview Gallery\n\n" + README_BEGIN + "\n" + README_END + "\n"

    if README_BEGIN in content and README_END in content:
        start = content.index(README_BEGIN)
        end = content.index(README_END) + len(README_END)
        content = content[:start] + generated + content[end:]
    else:
        content = content.rstrip() + "\n\n## Preview Gallery\n\n" + generated + "\n"
    readme_path.write_text(content.rstrip() + "\n", encoding="utf-8")
    print(f"[OK] Updated README previews: {readme_path}")


def parse_args() -> argparse.Namespace:
    """Parse preview generation options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out",
        type=Path,
        default=DEFAULT_OUT_DIR,
        help="Output directory for PNG/index files. Defaults to this preview folder's terrain_previews directory.",
    )
    parser.add_argument("--difficulties", type=float, nargs="+", default=[0.5])
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--size", type=float, nargs=2, default=[8.0, 8.0], metavar=("X", "Y"))
    parser.add_argument("--horizontal-scale", type=float, default=0.1)
    parser.add_argument("--vertical-scale", type=float, default=0.01)
    parser.add_argument("--slope-threshold", type=float, default=0.75)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument(
        "--mesh-view",
        choices=("top", "3d"),
        default="top",
        help="Render direct mesh terrains as clean top views or optional 3D previews.",
    )
    parser.add_argument(
        "--no-readme-update",
        action="store_true",
        help="Do not refresh the README preview gallery.",
    )
    return parser.parse_args()


def main():
    """Generate meshes, images, and README links for all configured preview cases."""
    args = parse_args()
    output_dir = args.out.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    pkg, import_mode = import_locolab_terrains()
    cfgs = discover_cfgs(pkg)
    missing_cases = sorted(set(cfgs) - set(PREVIEW_CASES))
    stale_cases = sorted(set(PREVIEW_CASES) - set(cfgs))
    if missing_cases:
        raise RuntimeError(f"Missing preview parameters for: {', '.join(missing_cases)}")
    if stale_cases:
        raise RuntimeError(f"Preview parameters reference missing terrain cfgs: {', '.join(stale_cases)}")

    common_kwargs = {
        "size": tuple(args.size),
    }
    hf_kwargs = {
        "horizontal_scale": args.horizontal_scale,
        "vertical_scale": args.vertical_scale,
        "slope_threshold": args.slope_threshold,
    }
    rows: list[dict] = []
    failures: list[str] = []

    for case_index, (cfg_name, cfg_cls) in enumerate(cfgs.items()):
        for difficulty_index, difficulty in enumerate(args.difficulties):
            label = cfg_name if len(args.difficulties) == 1 else f"{cfg_name} d={difficulty:g}"
            stem = cfg_name if len(args.difficulties) == 1 else f"{cfg_name}_d{difficulty:g}".replace(".", "p")
            png_path = output_dir / f"{stem}.png"
            try:
                np.random.seed(args.seed + case_index * 100 + difficulty_index)
                cfg = build_cfg(cfg_cls, common_kwargs, hf_kwargs, PREVIEW_CASES[cfg_name])
                meshes, origin = cfg.function(float(difficulty), cfg)
                mesh = trimesh.util.concatenate(meshes)
                assert_mesh(mesh, origin)
                render_mesh(
                    mesh,
                    png_path,
                    label,
                    args.image_size,
                    is_height_field=is_height_field_cfg(cfg_cls),
                    mesh_view=args.mesh_view,
                )
                rows.append({"label": label, "png": png_path})
                print(f"[OK] {label}: vertices={len(mesh.vertices)} faces={len(mesh.faces)} origin={origin.tolist()}")
            except Exception as exc:
                failures.append(f"{label}: {exc}")
                print(f"[FAIL] {label}: {exc}")

    if failures:
        raise RuntimeError("Preview generation failed:\n" + "\n".join(failures))

    write_index(output_dir, rows)
    if not args.no_readme_update:
        write_readme_preview_section(README_PATH, output_dir, rows)

    print(f"[DONE] import_mode={import_mode} output={output_dir}")


if __name__ == "__main__":
    main()
