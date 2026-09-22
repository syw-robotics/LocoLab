
"""LocoLab custom mesh terrains."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import trimesh

from isaaclab.terrains.trimesh import mesh_terrains as isaac_mesh_terrains
from isaaclab.terrains.trimesh.utils import make_border, make_box, make_plane

from .utils import (
    _sample_pole_placements,
    generate_roughness_height_field,
    sample_height_field,
    should_apply_poles,
    should_apply_roughness,
)


if TYPE_CHECKING:
    from . import locolab_mesh_terrains_cfg


def _mesh_grid_shape(cfg) -> tuple[int, int]:
    """Vertex counts that span ``[0, size]`` so edge samples are not clipped."""
    return int(cfg.size[0] / cfg.horizontal_scale) + 1, int(cfg.size[1] / cfg.horizontal_scale) + 1


def _samples_including_ends(start: float, stop: float, max_edge: float) -> np.ndarray:
    """Sample an interval so both metric endpoints are kept."""
    if abs(stop - start) <= max_edge:
        return np.array([start, stop], dtype=np.float64)
    count = int(np.ceil(abs(stop - start) / max_edge))
    return np.linspace(start, stop, count + 1, dtype=np.float64)


def _grid_faces(nx: int, ny: int, flip: bool = False) -> np.ndarray:
    if nx < 2 or ny < 2:
        return np.empty((0, 3), dtype=np.int64)
    i, j = np.mgrid[0 : nx - 1, 0 : ny - 1]
    a = i * ny + j
    b = (i + 1) * ny + j
    c = (i + 1) * ny + (j + 1)
    d = i * ny + (j + 1)
    if flip:
        return np.stack((a, d, c, a, c, b), axis=-1).reshape(-1, 3)
    return np.stack((a, b, c, a, c, d), axis=-1).reshape(-1, 3)


def _grid_z(xs: np.ndarray, ys: np.ndarray, z: float, flip: bool = False) -> trimesh.Trimesh:
    xx, yy = np.meshgrid(xs, ys, indexing="ij")
    verts = np.stack((xx.ravel(), yy.ravel(), np.full(xx.size, z)), axis=1)
    return trimesh.Trimesh(vertices=verts, faces=_grid_faces(*xx.shape, flip=flip), process=False)


def _strip_const_y(xs: np.ndarray, z0: float, z1: float, y: float, flip: bool = False) -> trimesh.Trimesh:
    xx, zz = np.meshgrid(xs, np.array([z0, z1], dtype=np.float64), indexing="ij")
    verts = np.stack((xx.ravel(), np.full(xx.size, y), zz.ravel()), axis=1)
    return trimesh.Trimesh(vertices=verts, faces=_grid_faces(*xx.shape, flip=flip), process=False)


def _strip_const_x(ys: np.ndarray, z0: float, z1: float, x: float, flip: bool = False) -> trimesh.Trimesh:
    yy, zz = np.meshgrid(ys, np.array([z0, z1], dtype=np.float64), indexing="ij")
    verts = np.stack((np.full(yy.size, x), yy.ravel(), zz.ravel()), axis=1)
    return trimesh.Trimesh(vertices=verts, faces=_grid_faces(*yy.shape, flip=flip), process=False)


def _tessellate_for_height_noise(mesh: trimesh.Trimesh, max_edge: float) -> trimesh.Trimesh:
    """Refine an axis-aligned box/plane without moving its XY footprint.

    Treads become an XY grid; risers become vertical strips. Endpoints stay on
    the original box edges, so stair width is not snapped to ``horizontal_scale``.
    """
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    if len(vertices) == 0:
        return mesh
    xs = np.unique(np.round(vertices[:, 0], 6))
    ys = np.unique(np.round(vertices[:, 1], 6))
    zs = np.unique(np.round(vertices[:, 2], 6))
    if len(xs) != 2 or len(ys) != 2:
        return mesh

    x0, x1 = float(xs[0]), float(xs[1])
    y0, y1 = float(ys[0]), float(ys[1])
    gx = _samples_including_ends(x0, x1, max_edge)
    gy = _samples_including_ends(y0, y1, max_edge)
    parts: list[trimesh.Trimesh] = []
    if len(zs) == 1:
        parts.append(_grid_z(gx, gy, float(zs[0])))
    elif len(zs) == 2:
        z0, z1 = float(zs[0]), float(zs[1])
        parts.append(_grid_z(gx, gy, z1))
        parts.append(_grid_z(np.array([x0, x1]), np.array([y0, y1]), z0, flip=True))
        parts.append(_strip_const_y(gx, z0, z1, y0, flip=True))
        parts.append(_strip_const_y(gx, z0, z1, y1, flip=False))
        parts.append(_strip_const_x(gy, z0, z1, x0, flip=True))
        parts.append(_strip_const_x(gy, z0, z1, x1, flip=False))
    else:
        return mesh
    return trimesh.util.concatenate(parts)


def _border_noise_weight(x: np.ndarray, y: np.ndarray, cfg) -> np.ndarray:
    """Keep the outer rim un-noised so neighboring sub-terrains still meet."""
    border = float(getattr(cfg, "border_width", 0.0))
    if border <= 0.0:
        border = float(cfg.horizontal_scale)
    weight = np.ones(np.shape(x), dtype=np.float64)
    weight[x < border] = 0.0
    weight[y < border] = 0.0
    weight[x > cfg.size[0] - border] = 0.0
    weight[y > cfg.size[1] - border] = 0.0
    return weight


def apply_mesh_roughness(
    meshes: list[trimesh.Trimesh],
    origin: np.ndarray,
    cfg: locolab_mesh_terrains_cfg.MeshRoughTerrainCfg,
    difficulty: float,
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Add shared Perlin roughness onto metric meshes without snapping XY size.

    Stair/box footprints stay at their original meters. Only z is displaced,
    after tessellating treads (and vertical strips) densely enough to carry
    the noise field. The configured border is left flat for sub-terrain seams.
    """
    if not should_apply_roughness(cfg):
        return meshes, origin

    noise_hf = generate_roughness_height_field(cfg, difficulty, shape=_mesh_grid_shape(cfg))
    if not np.any(noise_hf):
        return meshes, origin

    noise_m = noise_hf.astype(np.float64) * cfg.vertical_scale
    rough_meshes = []
    for mesh in meshes:
        tessellated = _tessellate_for_height_noise(mesh, cfg.horizontal_scale)
        vertices = np.asarray(tessellated.vertices, dtype=np.float64).copy()
        dz = sample_height_field(noise_m, vertices[:, 0], vertices[:, 1], cfg.horizontal_scale)
        vertices[:, 2] += dz * _border_noise_weight(vertices[:, 0], vertices[:, 1], cfg)
        tessellated.vertices = vertices
        rough_meshes.append(tessellated)

    origin = np.asarray(origin, dtype=np.float64).copy()
    origin_weight = _border_noise_weight(origin[0:1], origin[1:2], cfg)[0]
    origin[2] += origin_weight * sample_height_field(
        noise_m, origin[0:1], origin[1:2], cfg.horizontal_scale
    )[0]
    return rough_meshes, origin


def _mesh_surface_z(scene: trimesh.Trimesh | None, x: float, y: float) -> float | None:
    """Raycast downward onto concatenated meshes to find the local surface height."""
    if scene is None:
        return 0.0
    locations, _, _ = scene.ray.intersects_location(
        ray_origins=np.array([[x, y, 100.0]], dtype=np.float64),
        ray_directions=np.array([[0.0, 0.0, -1.0]], dtype=np.float64),
        multiple_hits=True,
    )
    if len(locations) == 0:
        return None
    return float(np.max(locations[:, 2]))


def apply_mesh_poles(
    meshes: list[trimesh.Trimesh],
    origin: np.ndarray,
    cfg: locolab_mesh_terrains_cfg.MeshRoughTerrainCfg,
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Add cylinders or square prisms that sit on the local mesh surface."""
    if not should_apply_poles(cfg):
        return meshes, origin

    placements = _sample_pole_placements(cfg, (float(cfg.size[0]), float(cfg.size[1])))
    if not placements:
        return meshes, origin

    scene = trimesh.util.concatenate(meshes) if meshes else None
    pole_meshes: list[trimesh.Trimesh] = []
    for x, y, kind, size, height in placements:
        z0 = _mesh_surface_z(scene, x, y)
        if z0 is None:
            continue
        transform = trimesh.transformations.translation_matrix((x, y, z0 + 0.5 * height))
        if kind == "cylinder":
            pole_meshes.append(
                trimesh.creation.cylinder(radius=size, height=height, sections=16, transform=transform)
            )
        else:
            pole_meshes.append(trimesh.creation.box(extents=(size, size, height), transform=transform))
    return meshes + pole_meshes, origin


def apply_mesh_surface_details(
    meshes: list[trimesh.Trimesh],
    origin: np.ndarray,
    cfg: locolab_mesh_terrains_cfg.MeshRoughTerrainCfg,
    difficulty: float,
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Apply shared mesh surface details: roughness then optional poles."""
    meshes, origin = apply_mesh_roughness(meshes, origin, cfg, difficulty)
    return apply_mesh_poles(meshes, origin, cfg)


def mesh_random_size_repeated_boxes_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshRepeatedBoxesTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate repeated boxes whose dimensions are sampled independently per object."""
    num_min, num_max = cfg.num_objects_range
    if num_min < 0 or num_min > num_max:
        raise ValueError(f"Invalid num_objects_range: {cfg.num_objects_range}.")
    if cfg.num_objects_type == "difficulty":
        num_objects = int(round(num_min + difficulty * (num_max - num_min)))
    elif cfg.num_objects_type == "random":
        num_objects = int(np.random.randint(num_min, num_max + 1))
    else:
        raise ValueError(f"Unsupported num_objects_type: {cfg.num_objects_type}.")

    height_min, height_max = cfg.box_height_range
    height = height_min + difficulty * (height_max - height_min)
    platform_height = cfg.platform_height if cfg.platform_height >= 0.0 else height
    length_min, length_max = cfg.box_length_range
    width_min, width_max = cfg.box_width_range
    if min(length_min, width_min, height) <= 0 or length_min > length_max or width_min > width_max:
        raise ValueError(
            f"Invalid repeated-box dimensions: length={cfg.box_length_range}, "
            f"width={cfg.box_width_range}, height={height}."
        )

    origin = np.asarray((0.5 * cfg.size[0], 0.5 * cfg.size[1], 0.5 * platform_height))
    clearance = 1.1
    platform_half = cfg.platform_width * 0.5 * clearance
    objects = []
    for _ in range(max(0, num_objects)):
        for _attempt in range(1000):
            length = np.random.uniform(length_min, length_max)
            width = np.random.uniform(width_min, width_max)
            x = np.random.uniform(length / 2, cfg.size[0] - length / 2)
            y = np.random.uniform(width / 2, cfg.size[1] - width / 2)
            if not (abs(x - origin[0]) <= platform_half + length / 2 and abs(y - origin[1]) <= platform_half + width / 2):
                objects.append((x, y, 0.0, length, width))
                break
        else:
            break

    meshes = [make_plane(cfg.size, height=0.0, center_zero=False)]
    for x, y, z, length, width in objects:
        if cfg.angle_type == "difficulty":
            angle_max = cfg.angle_range[0] + difficulty * (cfg.angle_range[1] - cfg.angle_range[0])
            angle = np.random.uniform(cfg.angle_range[0], angle_max)
        elif cfg.angle_type == "random":
            angle = np.random.uniform(*cfg.angle_range)
        else:
            raise ValueError(f"Unsupported angle_type: {cfg.angle_type}.")
        abs_noise = np.random.uniform(*cfg.abs_height_noise)
        rel_noise = np.random.uniform(*cfg.rel_height_noise)
        object_height = height * rel_noise + abs_noise
        if object_height > 0.0:
            meshes.append(make_box(length, width, object_height, (x, y, z), angle, cfg.angle_degrees))

    platform = trimesh.creation.box(
        (cfg.platform_width, cfg.platform_width, 0.5 * platform_height),
        trimesh.transformations.translation_matrix((origin[0], origin[1], 0.25 * platform_height)),
    )
    meshes.append(platform)
    return meshes, origin


def mesh_pyramid_stairs_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshPyramidStairsTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate metric pyramid stairs, then apply optional shared roughness and poles."""
    terrain_function = (
        isaac_mesh_terrains.inverted_pyramid_stairs_terrain
        if cfg.inverted
        else isaac_mesh_terrains.pyramid_stairs_terrain
    )
    meshes, origin = terrain_function(difficulty, cfg)
    return apply_mesh_surface_details(meshes, origin, cfg, difficulty)


def mesh_random_width_pyramid_stairs_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshRandomWidthPyramidStairsTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate pyramid stairs with a discretely sampled step width."""
    width_min, width_max = cfg.step_width_range
    if width_min <= 0.0 or width_min > width_max:
        raise ValueError(f"Invalid step_width_range: {cfg.step_width_range}.")
    if cfg.step_width_step <= 0.0:
        raise ValueError(f"step_width_step must be positive, got {cfg.step_width_step}.")

    num_increments = int(np.floor((width_max - width_min) / cfg.step_width_step + 1e-9))
    seed = getattr(cfg, "seed", None)
    if seed is None:
        width_index = np.random.randint(num_increments + 1)
    else:
        difficulty_bits = int(np.float64(difficulty).view(np.uint64))
        seed_sequence = np.random.SeedSequence([seed, difficulty_bits & 0xFFFFFFFF, difficulty_bits >> 32])
        width_index = np.random.default_rng(seed_sequence).integers(num_increments + 1)
    resolved_cfg = cfg.copy()
    resolved_cfg.step_width = width_min + width_index * cfg.step_width_step
    return mesh_pyramid_stairs_terrain(difficulty, resolved_cfg)


def mesh_straight_gap_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshStraightGapTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate a terrain with a gap in x direction."""
    # Calculate gap parameters
    gap_width = cfg.gap_width_range[0] + difficulty * (cfg.gap_width_range[1] - cfg.gap_width_range[0])
    if cfg.gap_depth_type == "difficulty":
        gap_depth = (cfg.gap_depth_range[1] - cfg.gap_depth_range[0]) * difficulty + cfg.gap_depth_range[0]
    elif cfg.gap_depth_type == "random":
        gap_depth = np.random.uniform(cfg.gap_depth_range[0], cfg.gap_depth_range[1])
    else:
        raise ValueError(f"cfg.gap_depth_type' must be 'difficulty' or 'random'. Current value is `{cfg.gap_depth_type}`.")
    gap_y_length = np.random.uniform(cfg.gap_y_length_range[0], cfg.gap_y_length_range[1])
    gap_x_offset = np.random.uniform(cfg.gap_x_offset_range[0], cfg.gap_x_offset_range[1])

    cx, cy = 0.5 * cfg.size[0], 0.5 * cfg.size[1]
    x1 = np.clip(cx + gap_x_offset - gap_width / 2, 0, cfg.size[0])
    x2 = np.clip(cx + gap_x_offset + gap_width / 2, 0, cfg.size[0])
    y1 = np.clip(cy - gap_y_length / 2, 0, cfg.size[1])
    y2 = np.clip(cy + gap_y_length / 2, 0, cfg.size[1])

    z_top, z_gap, z_bot = 0.0, -gap_depth, -1.0

    vertices, faces, vertex_map = [], [], {}

    def v(x, y, z):
        key = (round(x, 6), round(y, 6), round(z, 6))
        if key not in vertex_map:
            vertex_map[key] = len(vertices)
            vertices.append([x, y, z])
        return vertex_map[key]

    def quad(p1, p2, p3, p4, rev=False):
        i1, i2, i3, i4 = v(*p1), v(*p2), v(*p3), v(*p4)
        if rev:
            faces.extend([[i1, i4, i3], [i1, i3, i2]])
        else:
            faces.extend([[i1, i2, i3], [i1, i3, i4]])

    # Top surface
    quad([0, 0, z_top], [cfg.size[0], 0, z_top], [cfg.size[0], y1, z_top], [0, y1, z_top])
    if x1 > 0:
        quad([0, y1, z_top], [x1, y1, z_top], [x1, y2, z_top], [0, y2, z_top])
    if x2 < cfg.size[0]:
        quad([x2, y1, z_top], [cfg.size[0], y1, z_top], [cfg.size[0], y2, z_top], [x2, y2, z_top])
    quad([0, y2, z_top], [cfg.size[0], y2, z_top], [cfg.size[0], cfg.size[1], z_top], [0, cfg.size[1], z_top])

    # Gap walls and floor
    quad([x1, y1, z_top], [x2, y1, z_top], [x2, y1, z_gap], [x1, y1, z_gap], True)
    quad([x1, y2, z_top], [x2, y2, z_top], [x2, y2, z_gap], [x1, y2, z_gap])
    quad([x1, y1, z_top], [x1, y2, z_top], [x1, y2, z_gap], [x1, y1, z_gap])
    quad([x2, y1, z_top], [x2, y2, z_top], [x2, y2, z_gap], [x2, y1, z_gap], True)
    quad([x1, y1, z_gap], [x2, y1, z_gap], [x2, y2, z_gap], [x1, y2, z_gap])

    # Bottom surfaces
    quad([0, 0, z_bot], [cfg.size[0], 0, z_bot], [cfg.size[0], cfg.size[1], z_bot], [0, cfg.size[1], z_bot], True)
    quad([x1, y1, z_bot], [x2, y1, z_bot], [x2, y2, z_bot], [x1, y2, z_bot])

    # Gap wall extensions
    quad([x1, y1, z_gap], [x2, y1, z_gap], [x2, y1, z_bot], [x1, y1, z_bot])
    quad([x1, y2, z_gap], [x2, y2, z_gap], [x2, y2, z_bot], [x1, y2, z_bot], True)
    quad([x1, y1, z_gap], [x1, y2, z_gap], [x1, y2, z_bot], [x1, y1, z_bot], True)
    quad([x2, y1, z_gap], [x2, y2, z_gap], [x2, y2, z_bot], [x2, y1, z_bot])

    # Outer walls
    quad([0, 0, z_top], [cfg.size[0], 0, z_top], [cfg.size[0], 0, z_bot], [0, 0, z_bot])
    quad(
        [0, cfg.size[1], z_top],
        [cfg.size[0], cfg.size[1], z_top],
        [cfg.size[0], cfg.size[1], z_bot],
        [0, cfg.size[1], z_bot],
        True,
    )
    quad([0, 0, z_top], [0, cfg.size[1], z_top], [0, cfg.size[1], z_bot], [0, 0, z_bot], True)
    quad(
        [cfg.size[0], 0, z_top],
        [cfg.size[0], cfg.size[1], z_top],
        [cfg.size[0], cfg.size[1], z_bot],
        [cfg.size[0], 0, z_bot],
    )

    mesh = trimesh.Trimesh(
        vertices=np.array(vertices, dtype=np.float64), faces=np.array(faces, dtype=np.int64), process=True
    )
    return [mesh], np.array([cx, cy, 0.0])


def _sample_hurdle_count(spec: int | tuple[int, ...]) -> int:
    """Sample an inclusive hurdle-ring count, matching ``num_gaps_per_side_range``."""
    if isinstance(spec, int):
        num_min = num_max = spec
    elif len(spec) == 1:
        num_min = num_max = int(spec[0])
    elif len(spec) == 2:
        num_min, num_max = int(spec[0]), int(spec[1])
    else:
        raise ValueError(f"Invalid num_hurdles_per_side_range: {spec}.")
    if num_min < 1 or num_min > num_max:
        raise ValueError(f"Invalid num_hurdles_per_side_range: {spec}.")
    return int(np.random.randint(num_min, num_max + 1))


def mesh_hurdle_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshHurdleTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate a rectangular hurdle terrain with exact metric dimensions.

    The terrain is a flat plane with a raised perimeter and concentric raised
    square rings around the center platform.  Boxes are used directly so hurdle
    dimensions are not quantized by height-field horizontal or vertical scales.
    """
    width_min, width_max = cfg.hurdle_width_range
    height_min, height_max = cfg.hurdle_height_range
    platform_min, platform_max = cfg.platform_width_range
    spacing_min, spacing_max = cfg.spacing_range

    if width_min <= 0.0 or width_min > width_max:
        raise ValueError(f"Invalid hurdle_width_range: {cfg.hurdle_width_range}.")
    if height_min < 0.0 or height_min > height_max:
        raise ValueError(f"Invalid hurdle_height_range: {cfg.hurdle_height_range}.")
    if platform_min <= 0.0 or platform_min > platform_max:
        raise ValueError(f"Invalid platform_width_range: {cfg.platform_width_range}.")
    if spacing_min < 0.0 or spacing_min > spacing_max:
        raise ValueError(f"Invalid spacing_range: {cfg.spacing_range}.")

    hurdle_width = width_max + difficulty * (width_min - width_max)
    hurdle_height = height_min + difficulty * (height_max - height_min)
    platform_width = np.random.uniform(platform_min, platform_max)
    num_hurdles = _sample_hurdle_count(cfg.num_hurdles_per_side_range)
    # Flat ground between ring i and ring i + 1. The innermost ring stays flush.
    spacings = [float(np.random.uniform(spacing_min, spacing_max)) for _ in range(num_hurdles - 1)]

    def _outer_span(count: int) -> float:
        return platform_width + 2.0 * count * hurdle_width + 2.0 * sum(spacings[: max(count - 1, 0)])

    limit = min(cfg.size)
    while num_hurdles > 1 and _outer_span(num_hurdles) > limit:
        overflow = _outer_span(num_hurdles) - limit
        for index in range(len(spacings) - 1, -1, -1):
            reduction = min(spacings[index], 0.5 * overflow)
            spacings[index] -= reduction
            overflow -= 2.0 * reduction
            if overflow <= 1e-9:
                break
        if _outer_span(num_hurdles) > limit + 1e-6:
            num_hurdles -= 1
            spacings = spacings[: max(num_hurdles - 1, 0)]
    if _outer_span(num_hurdles) > limit:
        raise ValueError(
            "The center platform and hurdle rings must fit inside the terrain: "
            f"platform_width={platform_width}, hurdle_width={hurdle_width}, "
            f"num_hurdles={num_hurdles}, size={cfg.size}."
        )

    cx, cy = 0.5 * cfg.size[0], 0.5 * cfg.size[1]
    meshes = [make_plane(cfg.size, height=0.0, center_zero=False)]
    if hurdle_height == 0.0:
        return meshes, np.array([cx, cy, 0.0])

    def add_box(length: float, width: float, x: float, y: float) -> None:
        meshes.append(
            trimesh.creation.box(
                (length, width, hurdle_height),
                trimesh.transformations.translation_matrix((x, y, 0.5 * hurdle_height)),
            )
        )

    # Raised terrain boundary.  The original HF layout occupies half a hurdle
    # width at each edge, while retaining the full requested terrain size.
    edge = 0.5 * hurdle_width
    add_box(edge, cfg.size[1], 0.5 * edge, cy)
    add_box(edge, cfg.size[1], cfg.size[0] - 0.5 * edge, cy)
    add_box(cfg.size[0] - 2.0 * edge, edge, cx, 0.5 * edge)
    add_box(cfg.size[0] - 2.0 * edge, edge, cx, cfg.size[1] - 0.5 * edge)

    # Each ring is four exact-width boxes. Ring 0 is flush with the platform.
    inset = 0.0
    for ring in range(num_hurdles):
        if ring > 0:
            inset += spacings[ring - 1] + hurdle_width
        inner = platform_width + 2.0 * inset
        outer = inner + 2.0 * hurdle_width
        offset = 0.5 * inner + 0.5 * hurdle_width
        add_box(hurdle_width, outer, cx - offset, cy)
        add_box(hurdle_width, outer, cx + offset, cy)
        add_box(inner, hurdle_width, cx, cy - offset)
        add_box(inner, hurdle_width, cx, cy + offset)
    return meshes, np.array([cx, cy, 0.0])


def mesh_straight_stairs_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshStraightStairsTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate stairs terrain (up then down) in x direction."""
    # resolve the terrain configuration
    stair_height = cfg.stair_height_range[0] + difficulty * (cfg.stair_height_range[1] - cfg.stair_height_range[0])
    stair_width = cfg.stair_width + np.random.uniform(cfg.stair_width_noise_range[0], cfg.stair_width_noise_range[1])
    stair_length = np.random.uniform(cfg.stair_length_range[0], cfg.stair_length_range[1])
    platform_width = np.random.uniform(cfg.platform_width_range[0], cfg.platform_width_range[1])
    num_stairs = np.random.randint(cfg.num_stairs_range[0], cfg.num_stairs_range[1])

    # initialize list of meshes
    meshes_list = list()

    # make plane
    border_center = [0.5 * cfg.size[0], 0.5 * cfg.size[1], -0.5 * stair_height]
    border_inner_size = (2 * num_stairs * stair_width + platform_width, stair_length)
    make_borders = make_border(cfg.size, border_inner_size, stair_height, border_center)
    # add the border meshes to the list of meshes
    meshes_list += make_borders

    # generate the terrain
    # -- compute the position of the center of the terrain
    terrain_center = [0.5 * cfg.size[0], 0.5 * cfg.size[1], 0.0]
    # -- generate the stair pattern
    for k in range(num_stairs):
        # compute the quantities of the box
        # -- location
        box_z = terrain_center[2] + (k + 1) * stair_height / 2.0
        box_offset = (platform_width + stair_width) / 2.0 + (num_stairs - k - 1) * stair_width
        # -- dimensions
        box_height = (k + 1) * stair_height
        # generate the boxes
        box_dims = (stair_width, stair_length, box_height)
        # -- right
        box_pos = (terrain_center[0] - box_offset, terrain_center[1], box_z)
        box_right = trimesh.creation.box(box_dims, trimesh.transformations.translation_matrix(box_pos))
        # -- left
        box_pos = (terrain_center[0] + box_offset, terrain_center[1], box_z)
        box_left = trimesh.creation.box(box_dims, trimesh.transformations.translation_matrix(box_pos))
        # add the boxes to the list of meshes
        meshes_list += [box_right, box_left]

    # generate final box for the middle of the terrain
    box_dims = (platform_width, stair_length, num_stairs * stair_height)
    box_pos = (terrain_center[0], terrain_center[1], terrain_center[2] + num_stairs * stair_height / 2)
    box_middle = trimesh.creation.box(box_dims, trimesh.transformations.translation_matrix(box_pos))
    meshes_list.append(box_middle)

    # origin of the terrain
    origin = np.array([terrain_center[0], terrain_center[1], num_stairs * stair_height])

    return meshes_list, origin


def mesh_inverted_straight_stairs_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshStraightStairsTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate stairs terrain (down then up) in x direction."""
    # resolve the terrain configuration
    stair_height = cfg.stair_height_range[0] + difficulty * (cfg.stair_height_range[1] - cfg.stair_height_range[0])
    stair_width = cfg.stair_width + np.random.uniform(cfg.stair_width_noise_range[0], cfg.stair_width_noise_range[1])
    stair_length = np.random.uniform(cfg.stair_length_range[0], cfg.stair_length_range[1])
    platform_width = np.random.uniform(cfg.platform_width_range[0], cfg.platform_width_range[1])
    num_stairs = np.random.randint(cfg.num_stairs_range[0], cfg.num_stairs_range[1])

    # total height of the terrain
    total_height = num_stairs * stair_height

    # initialize list of meshes
    meshes_list = list()

    # make plane
    border_center = [0.5 * cfg.size[0], 0.5 * cfg.size[1], -0.5 * stair_height]
    border_inner_size = (2 * num_stairs * stair_width + platform_width, stair_length)
    make_borders = make_border(cfg.size, border_inner_size, stair_height, border_center)
    # add the border meshes to the list of meshes
    meshes_list += make_borders

    # generate the terrain
    # -- compute the position of the center of the terrain
    terrain_center = [0.5 * cfg.size[0], 0.5 * cfg.size[1], 0.0]
    # -- generate the stair pattern
    for k in range(num_stairs):
        # compute the quantities of the box
        # -- location
        box_z = terrain_center[2] - stair_height / 2.0 - k * stair_height
        box_offset = (platform_width + stair_width) / 2.0 + (num_stairs - k - 1) * stair_width
        # -- dimensions
        # generate the boxes
        box_dims = (stair_width, stair_length, stair_height)
        # -- right
        box_pos = (terrain_center[0] - box_offset, terrain_center[1], box_z)
        box_right = trimesh.creation.box(box_dims, trimesh.transformations.translation_matrix(box_pos))
        # -- left
        box_pos = (terrain_center[0] + box_offset, terrain_center[1], box_z)
        box_left = trimesh.creation.box(box_dims, trimesh.transformations.translation_matrix(box_pos))
        # add the boxes to the list of meshes
        meshes_list += [box_right, box_left]
    # -- generate side wall
    box_dims = (platform_width + 2 * num_stairs * stair_width, 0.2, total_height)
    box_pos_1 = (terrain_center[0], terrain_center[1] - stair_length / 2 - 0.1, terrain_center[2] - total_height / 2)
    box_pos_2 = (terrain_center[0], terrain_center[1] + stair_length / 2 + 0.1, terrain_center[2] - total_height / 2)
    box_side_1 = trimesh.creation.box(box_dims, trimesh.transformations.translation_matrix(box_pos_1))
    box_side_2 = trimesh.creation.box(box_dims, trimesh.transformations.translation_matrix(box_pos_2))
    meshes_list += [box_side_1, box_side_2]

    # generate final box for the middle of the terrain
    box_dims = (platform_width, stair_length, stair_height)
    box_pos = (terrain_center[0], terrain_center[1], terrain_center[2] - total_height - stair_height / 2)
    box_middle = trimesh.creation.box(box_dims, trimesh.transformations.translation_matrix(box_pos))
    meshes_list.append(box_middle)

    # origin of the terrain
    origin = np.array([terrain_center[0], terrain_center[1], terrain_center[2] - total_height])

    return meshes_list, origin
