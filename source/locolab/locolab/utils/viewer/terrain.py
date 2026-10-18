# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Terrain mesh kept for the viewer, and the vertex colors painted onto it."""

from __future__ import annotations

import numpy as np
import trimesh

# Same absolute scale mjlab uses for heightfield textures: 0.75 m is full
# saturation, so a few centimeters of roughness stays a light tint.
HEIGHT_COLOR_SCALE_M = 0.75
# Solid albedo used when height coloring is off. Dark enough that the
# scene light still shows ridges; the Terrain panel can replace it.
SOLID_COLOR = (135, 115, 100)


def retain_terrain_mesh_for_viewer(env_cfg) -> None:
    """Ask the terrain importer to keep a CPU mesh for the Viser ground.

    Training and the default play path leave the flag unset. Call this only
    after the play config is finalized and before ``gym.make``.
    """
    scene = getattr(env_cfg, "scene", None)
    terrain_cfg = getattr(scene, "terrain", None)
    if terrain_cfg is not None and hasattr(terrain_cfg, "retain_mesh"):
        terrain_cfg.retain_mesh = True


def paint_terrain(
    mesh: trimesh.Trimesh,
    *,
    by_height: bool,
    scale_m: float,
    rgb: tuple[int, int, int],
) -> trimesh.Trimesh:
    """Return a copy colored by absolute height, or with one solid albedo.

    Both paths assign ``ColorVisuals``, so the uploaded GLB stays lit the same way.
    ``scale_m`` is the height, in meters, that reaches full red or full blue.
    """
    if by_height:
        colors = _height_colors(np.asarray(mesh.vertices[:, 2]), scale_m)
    else:
        colors = np.empty((len(mesh.vertices), 4), dtype=np.uint8)
        colors[:, 0] = rgb[0]
        colors[:, 1] = rgb[1]
        colors[:, 2] = rgb[2]
        colors[:, 3] = 255
    painted = mesh.copy()
    painted.visual = trimesh.visual.ColorVisuals(mesh=painted, vertex_colors=colors)
    return painted


def _height_colors(heights: np.ndarray, scale_m: float) -> np.ndarray:
    """Green at z=0, red above, blue below. ``scale_m`` reaches full saturation."""
    scale = float(scale_m) if scale_m > 1e-6 else 1e-6
    signed = np.clip(np.asarray(heights, dtype=np.float64) / scale, -1.0, 1.0)
    hue = 0.33 - 0.33 * signed
    saturation = 0.45 + 0.25 * np.abs(signed)
    value = 0.45 + 0.25 * np.abs(signed)
    chroma = value * saturation
    secondary = chroma * (1.0 - np.abs((hue * 6.0) % 2.0 - 1.0))
    match = value - chroma
    sector = np.floor(hue * 6.0).astype(np.int64) % 6
    zero = np.zeros_like(hue)
    channels = (
        (chroma, secondary, zero),
        (secondary, chroma, zero),
        (zero, chroma, secondary),
        (zero, secondary, chroma),
        (secondary, zero, chroma),
        (chroma, zero, secondary),
    )
    red = np.zeros_like(hue)
    green = np.zeros_like(hue)
    blue = np.zeros_like(hue)
    for index, (red_term, green_term, blue_term) in enumerate(channels):
        mask = sector == index
        red[mask] = red_term[mask]
        green[mask] = green_term[mask]
        blue[mask] = blue_term[mask]
    rgb = np.stack((red + match, green + match, blue + match), axis=-1)
    colors = np.empty((len(heights), 4), dtype=np.uint8)
    colors[:, :3] = np.clip(rgb * 255.0, 0, 255).astype(np.uint8)
    colors[:, 3] = 255
    return colors
