
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


# Mesh Box Terrain Definition
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
    platform_half = cfg.center_platform_width * 0.5 * clearance
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
        (cfg.center_platform_width, cfg.center_platform_width, 0.5 * platform_height),
        trimesh.transformations.translation_matrix((origin[0], origin[1], 0.25 * platform_height)),
    )
    meshes.append(platform)
    return meshes, origin


# Mesh Pyramid Stairs Terrain Definition
def mesh_pyramid_stairs_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshPyramidStairsTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate metric pyramid stairs, then apply optional shared roughness and poles.

    Isaac Lab reads ``step_height_range``, ``step_width``, and ``platform_width``.
    Those are filled on a copy so the shared config keeps LocoLab's parameter names.
    """
    noise_min, noise_max = cfg.stair_width_noise_range
    if cfg.stair_width <= 0.0:
        raise ValueError(f"stair_width must be positive, got {cfg.stair_width}.")
    if noise_min > noise_max:
        raise ValueError(f"Invalid stair_width_noise_range: {cfg.stair_width_noise_range}.")
    if cfg.stair_width_noise_step <= 0.0:
        raise ValueError(f"stair_width_noise_step must be positive, got {cfg.stair_width_noise_step}.")
    noise_step_min = int(np.ceil(noise_min / cfg.stair_width_noise_step))
    noise_step_max = int(np.floor(noise_max / cfg.stair_width_noise_step))
    if noise_step_min > noise_step_max:
        raise ValueError(
            f"stair_width_noise_range={cfg.stair_width_noise_range} contains no multiples of "
            f"stair_width_noise_step={cfg.stair_width_noise_step}."
        )
    noise_steps = np.random.randint(noise_step_min, noise_step_max + 1)
    stair_width_noise = noise_steps * cfg.stair_width_noise_step
    stair_width = float(cfg.stair_width + stair_width_noise)
    if stair_width <= 0.0:
        raise ValueError(
            f"Sampled stair width must be positive, got {stair_width} from {cfg.stair_width} + {cfg.stair_width_noise_range}."
        )
    resolved = cfg.copy()
    resolved.step_height_range = cfg.stair_height_range
    resolved.step_width = stair_width
    resolved.platform_width = cfg.center_platform_width
    terrain_function = (
        isaac_mesh_terrains.inverted_pyramid_stairs_terrain
        if cfg.inverted
        else isaac_mesh_terrains.pyramid_stairs_terrain
    )
    meshes, origin = terrain_function(difficulty, resolved)
    return apply_mesh_surface_details(meshes, origin, cfg, difficulty)


def _add_floating_slab(
    meshes: list[trimesh.Trimesh], length: float, width: float, thickness: float, x: float, y: float, top_z: float
) -> None:
    """Add one axis-aligned tread whose top face is at ``top_z``."""
    if length <= 1e-6 or width <= 1e-6:
        return
    center_z = top_z - 0.5 * thickness
    meshes.append(
        trimesh.creation.box(
            (length, width, thickness),
            trimesh.transformations.translation_matrix((x, y, center_z)),
        )
    )


# Mesh Floating Pyramid Stairs Terrain Definition
def mesh_floating_pyramid_stairs_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshFloatingPyramidStairsTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate floating stairs around a center platform.

    ``pyramid`` follows Isaac Lab pyramid stair footprints. ``cross`` keeps four
    flights whose width equals the center platform. ``random`` chooses one of
    those footprints per sub-terrain. Treads are ``stair_thickness`` tall,
    measured down from the tread surface.
    """
    stair_height = cfg.stair_height_range[0] + difficulty * (cfg.stair_height_range[1] - cfg.stair_height_range[0])
    if stair_height < 0.0:
        raise ValueError(f"Resolved stair height must be non-negative, got {stair_height}.")
    noise_min, noise_max = cfg.stair_width_noise_range
    if cfg.stair_width <= 0.0:
        raise ValueError(f"stair_width must be positive, got {cfg.stair_width}.")
    if noise_min > noise_max:
        raise ValueError(f"Invalid stair_width_noise_range: {cfg.stair_width_noise_range}.")
    stair_width = float(cfg.stair_width + np.random.uniform(noise_min, noise_max))
    if stair_width <= 0.0:
        raise ValueError(
            f"Sampled stair width must be positive, got {stair_width} from {cfg.stair_width} + {cfg.stair_width_noise_range}."
        )
    if cfg.stair_type == "random":
        stair_type = "cross" if np.random.random() < 0.5 else "pyramid"
    elif cfg.stair_type in ("pyramid", "cross"):
        stair_type = cfg.stair_type
    else:
        raise ValueError(f"stair_type must be 'pyramid', 'cross', or 'random', got {cfg.stair_type!r}.")
    if cfg.stair_thickness <= 0.0:
        raise ValueError(f"stair_thickness must be positive, got {cfg.stair_thickness}.")
    width_min, width_max = cfg.center_platform_width
    if width_min <= 0.0 or width_min > width_max:
        raise ValueError(f"Invalid center_platform_width range: {cfg.center_platform_width}.")
    center_platform_width = float(np.random.uniform(width_min, width_max))
    if cfg.border_width < 0.0:
        raise ValueError(f"border_width must be non-negative, got {cfg.border_width}.")
    terrain_size = (cfg.size[0] - 2 * cfg.border_width, cfg.size[1] - 2 * cfg.border_width)
    if min(terrain_size) <= center_platform_width:
        raise ValueError(
            "The center platform and border must leave room for stairs: "
            f"size={cfg.size}, border_width={cfg.border_width}, center_platform_width={center_platform_width}."
        )
    span_x = (terrain_size[0] - center_platform_width) // (2 * stair_width)
    span_y = (terrain_size[1] - center_platform_width) // (2 * stair_width)
    # Pyramid rings absorb the remainder into their widths so the center matches the sampled width.
    extra_ring = 1 if stair_type == "pyramid" else 0
    num_steps = int(min(span_x, span_y) + extra_ring)
    if num_steps < 1:
        raise ValueError(
            "Floating stairs need at least one step: "
            f"size={cfg.size}, stair_width={stair_width}, center_platform_width={center_platform_width}."
        )

    meshes_list: list[trimesh.Trimesh] = []
    terrain_center = [0.5 * cfg.size[0], 0.5 * cfg.size[1], 0.0]
    cx, cy = terrain_center[0], terrain_center[1]
    # Ascending stairs rise toward the center. Inverted stairs drop into a pit.
    level_sign = -1.0 if cfg.inverted else 1.0

    if stair_type == "pyramid":
        center_size = (center_platform_width, center_platform_width)
        ring_width_x = (terrain_size[0] - center_platform_width) / (2 * num_steps)
        ring_width_y = (terrain_size[1] - center_platform_width) / (2 * num_steps)
        for k in range(num_steps):
            box_size = (terrain_size[0] - 2 * k * ring_width_x, terrain_size[1] - 2 * k * ring_width_y)
            top_z = level_sign * (k + 1) * stair_height
            box_offset_x = (k + 0.5) * ring_width_x
            box_offset_y = (k + 0.5) * ring_width_y
            side_length = box_size[1] - 2 * ring_width_y
            _add_floating_slab(
                meshes_list,
                box_size[0],
                ring_width_y,
                cfg.stair_thickness,
                cx,
                cy + terrain_size[1] / 2.0 - box_offset_y,
                top_z,
            )
            _add_floating_slab(
                meshes_list,
                box_size[0],
                ring_width_y,
                cfg.stair_thickness,
                cx,
                cy - terrain_size[1] / 2.0 + box_offset_y,
                top_z,
            )
            _add_floating_slab(
                meshes_list,
                ring_width_x,
                side_length,
                cfg.stair_thickness,
                cx + terrain_size[0] / 2.0 - box_offset_x,
                cy,
                top_z,
            )
            _add_floating_slab(
                meshes_list,
                ring_width_x,
                side_length,
                cfg.stair_thickness,
                cx - terrain_size[0] / 2.0 + box_offset_x,
                cy,
                top_z,
            )
    else:
        center_size = (center_platform_width, center_platform_width)
        half_platform = 0.5 * center_platform_width
        for k in range(num_steps):
            # k = 0 is the outer stair. Flights meet the platform edge.
            inset = (num_steps - k - 0.5) * stair_width
            top_z = level_sign * (k + 1) * stair_height
            _add_floating_slab(
                meshes_list, center_platform_width, stair_width, cfg.stair_thickness, cx, cy + half_platform + inset, top_z
            )
            _add_floating_slab(
                meshes_list, center_platform_width, stair_width, cfg.stair_thickness, cx, cy - half_platform - inset, top_z
            )
            _add_floating_slab(
                meshes_list, stair_width, center_platform_width, cfg.stair_thickness, cx + half_platform + inset, cy, top_z
            )
            _add_floating_slab(
                meshes_list, stair_width, center_platform_width, cfg.stair_thickness, cx - half_platform - inset, cy, top_z
            )

    center_top = level_sign * (num_steps + 1) * stair_height
    _add_floating_slab(
        meshes_list,
        center_size[0],
        center_size[1],
        cfg.stair_thickness,
        cx,
        cy,
        center_top,
    )

    if cfg.inverted:
        # The approach surface stays at z = 0. The pit floor sits one step below the center slab.
        if cfg.floor_thickness <= 0.0:
            raise ValueError(f"floor_thickness must be positive, got {cfg.floor_thickness}.")
        if cfg.border_width > 0.0:
            border_center = [0.5 * cfg.size[0], 0.5 * cfg.size[1], -0.5 * stair_height]
            border_inner_size = (cfg.size[0] - 2 * cfg.border_width, cfg.size[1] - 2 * cfg.border_width)
            meshes_list += make_border(cfg.size, border_inner_size, stair_height, border_center)
        floor_z = center_top - cfg.stair_thickness - stair_height
        floor_size = terrain_size if cfg.extend_center_platform_under_stairs else center_size
        meshes_list.append(
            trimesh.creation.box(
                extents=(floor_size[0], floor_size[1], cfg.floor_thickness),
                transform=trimesh.transformations.translation_matrix(
                    (cx, cy, floor_z - cfg.floor_thickness / 2.0)
                ),
            )
        )
    else:
        meshes_list.append(make_plane(cfg.size, height=0.0, center_zero=False))

    origin = np.array([terrain_center[0], terrain_center[1], center_top])
    return apply_mesh_surface_details(meshes_list, origin, cfg, difficulty)


# Mesh Straight Gap Terrain Definition
def mesh_straight_gap_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshStraightGapTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate a y-limited corridor with independently placed islands along x.

    Layout matches :func:`locolab_hf_terrains.straight_gap_terrain`. Spans stay in
    meters, so a sampled gap or island is not snapped to a height-field cell.
    """
    # landing | gap | island | ... | gap | center | gap | ... | island | gap | landing
    num_gaps = cfg.num_gaps_per_side_range
    if isinstance(num_gaps, int):
        num_min = num_max = num_gaps
    elif len(num_gaps) == 1:
        num_min = num_max = int(num_gaps[0])
    elif len(num_gaps) == 2:
        num_min, num_max = int(num_gaps[0]), int(num_gaps[1])
    else:
        raise ValueError(f"Invalid num_gaps_per_side_range: {cfg.num_gaps_per_side_range}.")
    if num_min < 1 or num_min > num_max:
        raise ValueError(f"Invalid num_gaps_per_side_range: {cfg.num_gaps_per_side_range}.")
    num_gaps = int(np.random.randint(num_min, num_max + 1))

    gap_min, gap_max = cfg.gap_width_range
    if gap_min <= 0.0 or gap_min > gap_max:
        raise ValueError(f"Invalid gap_width_range: {cfg.gap_width_range}.")
    depth_min, depth_max = cfg.gap_depth_range
    if depth_min < 0.0 or depth_min > depth_max:
        raise ValueError(f"Invalid gap_depth_range: {cfg.gap_depth_range}.")
    platform_min, platform_max = cfg.center_platform_width_range
    if platform_min <= 0.0 or platform_min > platform_max:
        raise ValueError(f"Invalid center_platform_width_range: {cfg.center_platform_width_range}.")
    island_w_min, island_w_max = cfg.island_width_range
    if island_w_min <= 0.0 or island_w_min > island_w_max:
        raise ValueError(f"Invalid island_width_range: {cfg.island_width_range}.")
    offset_min, offset_max = cfg.island_y_offset_range
    if offset_min > offset_max:
        raise ValueError(f"Invalid island_y_offset_range: {cfg.island_y_offset_range}.")
    height_min, height_max = cfg.island_height_offset_range
    if height_min > height_max:
        raise ValueError(f"Invalid island_height_offset_range: {cfg.island_height_offset_range}.")

    gap_width = gap_min + difficulty * (gap_max - gap_min)
    if cfg.gap_depth_type == "difficulty":
        gap_depth = depth_min + difficulty * (depth_max - depth_min)
    elif cfg.gap_depth_type == "random":
        gap_depth = float(np.random.uniform(depth_min, depth_max))
    else:
        raise ValueError(
            f"cfg.gap_depth_type must be 'difficulty' or 'random'. Current value is `{cfg.gap_depth_type}`."
        )

    num_islands = num_gaps - 1

    def _sample_islands() -> list[tuple[float, float, float, float]]:
        # (x-width, y-width, y-offset, height) in meters, inner island first.
        return [
            (
                float(np.random.uniform(island_w_min, island_w_max)),
                float(np.random.uniform(island_w_min, island_w_max)),
                float(np.random.uniform(offset_min, offset_max)),
                float(np.random.uniform(height_min, height_max)),
            )
            for _ in range(num_islands)
        ]

    left_islands = _sample_islands() if num_islands > 0 else []
    right_islands = _sample_islands() if num_islands > 0 else []

    cx, cy = 0.5 * cfg.size[0], 0.5 * cfg.size[1]
    center_platform_width = float(np.random.uniform(platform_min, platform_max))
    inner_left = max(cx - 0.5 * center_platform_width, 0.0)
    inner_right = min(cx + 0.5 * center_platform_width, cfg.size[0])

    def _fit_side(
        budget: float,
        side_gaps: int,
        side_gap_width: float,
        islands: list[tuple[float, float, float, float]],
    ) -> tuple[int, float, list[tuple[float, float, float, float]]]:
        # Keep the sampled layout if it fits. Otherwise drop gaps, then cap gap and island x-widths.
        budget = max(float(budget), 0.0)
        side_gap_width = max(float(side_gap_width), 0.0)
        islands = list(islands)
        side_gaps = max(int(side_gaps), 0)
        min_piece = 1e-4
        if budget < min_piece or side_gaps < 1 or side_gap_width < min_piece:
            return 0, 0.0, []
        while side_gaps > 1 and (2 * side_gaps - 1) * min_piece > budget:
            side_gaps -= 1
            islands = islands[: side_gaps - 1]
        islands = islands[: max(side_gaps - 1, 0)]
        side_gap_width = min(side_gap_width, max((budget - len(islands) * min_piece) / side_gaps, min_piece))
        remain = max(budget - side_gaps * side_gap_width, 0.0)
        if remain <= min_piece or not islands:
            islands = []
        else:
            widths = [max(float(width), 0.0) for width, *_ in islands]
            overflow = sum(widths) - remain
            if overflow > 1e-8:
                for index in range(len(widths) - 1, -1, -1):
                    take = min(overflow, widths[index])
                    widths[index] -= take
                    overflow -= take
                    if overflow <= 1e-8:
                        break
            islands = [
                (width, y_width, y_offset, height)
                for width, (_, y_width, y_offset, height) in zip(widths, islands)
                if width > 1e-6
            ]
        return side_gaps, side_gap_width, islands

    def _edge_landing_reserve() -> float:
        return 0.5 * float(np.random.uniform(island_w_min, island_w_max))

    left_n, left_gap, left_islands = _fit_side(
        max(inner_left - _edge_landing_reserve(), 0.0), num_gaps, gap_width, left_islands
    )
    right_n, right_gap, right_islands = _fit_side(
        max(cfg.size[0] - inner_right - _edge_landing_reserve(), 0.0),
        num_gaps,
        gap_width,
        right_islands,
    )
    outer_left = inner_left - (left_n * left_gap + sum(width for width, *_ in left_islands))
    outer_right = inner_right + (right_n * right_gap + sum(width for width, *_ in right_islands))

    meshes = [make_plane(cfg.size, height=-gap_depth, center_zero=False)]

    def add_block(x0: float, x1: float, y_offset: float, y_width: float, top_z: float) -> None:
        x0 = float(np.clip(x0, 0.0, cfg.size[0]))
        x1 = float(np.clip(x1, 0.0, cfg.size[0]))
        if x1 - x0 <= 1e-6:
            return
        y0 = float(np.clip(cy + y_offset - 0.5 * y_width, 0.0, cfg.size[1]))
        y1 = float(np.clip(cy + y_offset + 0.5 * y_width, 0.0, cfg.size[1]))
        if y1 - y0 <= 1e-6:
            return
        height = top_z + gap_depth
        if height <= 1e-6:
            return
        meshes.append(
            trimesh.creation.box(
                (x1 - x0, y1 - y0, height),
                trimesh.transformations.translation_matrix(
                    ((x0 + x1) * 0.5, (y0 + y1) * 0.5, -gap_depth + 0.5 * height)
                ),
            )
        )

    add_block(0.0, outer_left, 0.0, float(np.random.uniform(island_w_min, island_w_max)), 0.0)
    add_block(inner_left, inner_right, 0.0, float(np.random.uniform(island_w_min, island_w_max)), 0.0)
    add_block(outer_right, cfg.size[0], 0.0, float(np.random.uniform(island_w_min, island_w_max)), 0.0)
    for inner, sign, gap_span, islands in (
        (inner_left, -1.0, left_gap, left_islands),
        (inner_right, 1.0, right_gap, right_islands),
    ):
        cursor = inner
        for x_width, y_width, y_offset, height in islands:
            cursor += sign * gap_span
            island_start = cursor
            cursor += sign * x_width
            lo, hi = (cursor, island_start) if sign < 0.0 else (island_start, cursor)
            add_block(lo, hi, y_offset, y_width, height)
    # Roughness follows the height-field mask: walkable tops only, pit floor stays flat.
    origin = np.array([cx, cy, 0.0])
    walkable, origin = apply_mesh_surface_details(meshes[1:], origin, cfg, difficulty)
    # Thin perimeter wall closes the pit against neighboring tiles, whose border is at z=0.
    border_width = float(getattr(cfg, "border_width", 0.0))
    if border_width < 0.0 or 2.0 * border_width >= min(cfg.size):
        raise ValueError(f"Invalid border_width: {border_width} for size {cfg.size}.")
    border = []
    if border_width > 0.0 and gap_depth > 1e-6:
        border = make_border(
            cfg.size,
            (cfg.size[0] - 2.0 * border_width, cfg.size[1] - 2.0 * border_width),
            gap_depth,
            (cx, cy, -0.5 * gap_depth),
        )
    return [meshes[0], *walkable, *border], origin


# Mesh Hurdle Terrain Definition
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
    platform_min, platform_max = cfg.center_platform_width_range
    spacing_min, spacing_max = cfg.spacing_range

    if width_min <= 0.0 or width_min > width_max:
        raise ValueError(f"Invalid hurdle_width_range: {cfg.hurdle_width_range}.")
    if height_min < 0.0 or height_min > height_max:
        raise ValueError(f"Invalid hurdle_height_range: {cfg.hurdle_height_range}.")
    if platform_min <= 0.0 or platform_min > platform_max:
        raise ValueError(f"Invalid center_platform_width_range: {cfg.center_platform_width_range}.")
    if spacing_min < 0.0 or spacing_min > spacing_max:
        raise ValueError(f"Invalid spacing_range: {cfg.spacing_range}.")

    hurdle_width = width_max + difficulty * (width_min - width_max)
    hurdle_height = height_min + difficulty * (height_max - height_min)
    center_platform_width = np.random.uniform(platform_min, platform_max)
    hurdle_count = cfg.num_hurdles_per_side_range
    if isinstance(hurdle_count, int):
        hurdle_min = hurdle_max = hurdle_count
    elif len(hurdle_count) == 1:
        hurdle_min = hurdle_max = int(hurdle_count[0])
    elif len(hurdle_count) == 2:
        hurdle_min, hurdle_max = int(hurdle_count[0]), int(hurdle_count[1])
    else:
        raise ValueError(f"Invalid num_hurdles_per_side_range: {hurdle_count}.")
    if hurdle_min < 1 or hurdle_min > hurdle_max:
        raise ValueError(f"Invalid num_hurdles_per_side_range: {hurdle_count}.")
    num_hurdles = int(np.random.randint(hurdle_min, hurdle_max + 1))
    # Flat ground between ring i and ring i + 1. The innermost ring stays flush.
    spacings = [float(np.random.uniform(spacing_min, spacing_max)) for _ in range(num_hurdles - 1)]

    def _outer_span(count: int) -> float:
        return center_platform_width + 2.0 * count * hurdle_width + 2.0 * sum(spacings[: max(count - 1, 0)])

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
            f"center_platform_width={center_platform_width}, hurdle_width={hurdle_width}, "
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
        inner = center_platform_width + 2.0 * inset
        outer = inner + 2.0 * hurdle_width
        offset = 0.5 * inner + 0.5 * hurdle_width
        add_box(hurdle_width, outer, cx - offset, cy)
        add_box(hurdle_width, outer, cx + offset, cy)
        add_box(inner, hurdle_width, cx, cy - offset)
        add_box(inner, hurdle_width, cx, cy + offset)
    return meshes, np.array([cx, cy, 0.0])


def _sample_straight_stair_count(cfg: locolab_mesh_terrains_cfg.MeshStraightStairsTerrainCfg, stair_width: float, center_platform_width: float) -> int:
    """Sample a stair count whose two flights and platform fit along x.

    ``num_stairs_range`` keeps the existing exclusive upper bound. When that
    range asks for more stairs than the terrain can hold, the count is capped
    at the largest fitting value.
    """
    num_min, num_max = int(cfg.num_stairs_range[0]), int(cfg.num_stairs_range[1])
    if num_min < 1 or num_min >= num_max:
        raise ValueError(
            f"Invalid num_stairs_range: {cfg.num_stairs_range}. The upper bound is exclusive and must be greater than the minimum."
        )
    if stair_width <= 0.0:
        raise ValueError(f"stair_width must be positive, got {stair_width}.")
    if center_platform_width <= 0.0 or center_platform_width >= cfg.size[0]:
        raise ValueError(
            f"center_platform_width must be inside the terrain length, got {center_platform_width} for size {cfg.size[0]}."
        )

    max_stairs = int(np.floor((cfg.size[0] - center_platform_width) / (2.0 * stair_width)))
    if max_stairs < 1:
        raise ValueError(
            "One stair on each side of the platform must fit inside the terrain: "
            f"size={cfg.size[0]}, center_platform_width={center_platform_width}, stair_width={stair_width}."
        )
    num_min = min(num_min, max_stairs)
    num_max = min(num_max, max_stairs + 1)
    if num_min >= num_max:
        return max_stairs
    return int(np.random.randint(num_min, num_max))


# Mesh Straight Stairs Terrain Definition
def mesh_straight_stairs_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshStraightStairsTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate stairs terrain (up then down) in x direction."""
    # resolve the terrain configuration
    stair_height = cfg.stair_height_range[0] + difficulty * (cfg.stair_height_range[1] - cfg.stair_height_range[0])
    stair_width = cfg.stair_width + np.random.uniform(cfg.stair_width_noise_range[0], cfg.stair_width_noise_range[1])
    stair_length = np.random.uniform(cfg.stair_length_range[0], cfg.stair_length_range[1])
    center_platform_width = np.random.uniform(cfg.center_platform_width_range[0], cfg.center_platform_width_range[1])
    num_stairs = _sample_straight_stair_count(cfg, stair_width, center_platform_width)

    # initialize list of meshes
    meshes_list = list()

    # make plane
    border_center = [0.5 * cfg.size[0], 0.5 * cfg.size[1], -0.5 * stair_height]
    border_inner_size = (2 * num_stairs * stair_width + center_platform_width, stair_length)
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
        box_offset = (center_platform_width + stair_width) / 2.0 + (num_stairs - k - 1) * stair_width
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
    box_dims = (center_platform_width, stair_length, num_stairs * stair_height)
    box_pos = (terrain_center[0], terrain_center[1], terrain_center[2] + num_stairs * stair_height / 2)
    box_middle = trimesh.creation.box(box_dims, trimesh.transformations.translation_matrix(box_pos))
    meshes_list.append(box_middle)

    # origin of the terrain
    origin = np.array([terrain_center[0], terrain_center[1], num_stairs * stair_height])

    return meshes_list, origin


# Mesh Inverted Straight Stairs Terrain Definition
def mesh_inverted_straight_stairs_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshStraightStairsTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate stairs terrain (down then up) in x direction."""
    # resolve the terrain configuration
    stair_height = cfg.stair_height_range[0] + difficulty * (cfg.stair_height_range[1] - cfg.stair_height_range[0])
    stair_width = cfg.stair_width + np.random.uniform(cfg.stair_width_noise_range[0], cfg.stair_width_noise_range[1])
    stair_length = np.random.uniform(cfg.stair_length_range[0], cfg.stair_length_range[1])
    center_platform_width = np.random.uniform(cfg.center_platform_width_range[0], cfg.center_platform_width_range[1])
    num_stairs = _sample_straight_stair_count(cfg, stair_width, center_platform_width)

    # total height of the terrain
    total_height = num_stairs * stair_height

    # initialize list of meshes
    meshes_list = list()

    # make plane
    border_center = [0.5 * cfg.size[0], 0.5 * cfg.size[1], -0.5 * stair_height]
    border_inner_size = (2 * num_stairs * stair_width + center_platform_width, stair_length)
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
        box_offset = (center_platform_width + stair_width) / 2.0 + (num_stairs - k - 1) * stair_width
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
    box_dims = (center_platform_width + 2 * num_stairs * stair_width, 0.2, total_height)
    box_pos_1 = (terrain_center[0], terrain_center[1] - stair_length / 2 - 0.1, terrain_center[2] - total_height / 2)
    box_pos_2 = (terrain_center[0], terrain_center[1] + stair_length / 2 + 0.1, terrain_center[2] - total_height / 2)
    box_side_1 = trimesh.creation.box(box_dims, trimesh.transformations.translation_matrix(box_pos_1))
    box_side_2 = trimesh.creation.box(box_dims, trimesh.transformations.translation_matrix(box_pos_2))
    meshes_list += [box_side_1, box_side_2]

    # generate final box for the middle of the terrain
    box_dims = (center_platform_width, stair_length, stair_height)
    box_pos = (terrain_center[0], terrain_center[1], terrain_center[2] - total_height - stair_height / 2)
    box_middle = trimesh.creation.box(box_dims, trimesh.transformations.translation_matrix(box_pos))
    meshes_list.append(box_middle)

    # origin of the terrain
    origin = np.array([terrain_center[0], terrain_center[1], terrain_center[2] - total_height])

    return meshes_list, origin


# -------------------- Internal helpers -------------------- #


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

    # Include both terrain edges so border samples are not clipped.
    noise_shape = (int(cfg.size[0] / cfg.horizontal_scale) + 1, int(cfg.size[1] / cfg.horizontal_scale) + 1)
    noise_hf = generate_roughness_height_field(cfg, difficulty, shape=noise_shape)
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
        if scene is None:
            z0 = 0.0
        else:
            locations, _, _ = scene.ray.intersects_location(
                ray_origins=np.array([[x, y, 100.0]], dtype=np.float64),
                ray_directions=np.array([[0.0, 0.0, -1.0]], dtype=np.float64),
                multiple_hits=True,
            )
            if len(locations) == 0:
                continue
            z0 = float(np.max(locations[:, 2]))
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


def _interpolate_range(value_range: tuple[float, float], difficulty: float) -> float:
    lower, upper = value_range
    if lower > upper:
        raise ValueError(f"Range must be ordered, got {value_range}.")
    return float(lower + np.clip(difficulty, 0.0, 1.0) * (upper - lower))


def _interpolate_decreasing_range(value_range: tuple[float, float], difficulty: float) -> float:
    lower, upper = value_range
    if lower > upper:
        raise ValueError(f"Range must be ordered, got {value_range}.")
    return float(upper - np.clip(difficulty, 0.0, 1.0) * (upper - lower))


def _sample_count(count_range: tuple[int, int], difficulty: float) -> int:
    lower, upper = count_range
    if lower < 0 or lower > upper:
        raise ValueError(f"Invalid count range, got {count_range}.")
    return int(round(lower + np.clip(difficulty, 0.0, 1.0) * (upper - lower)))


def _rotated_box(
    extents: tuple[float, float, float], position: tuple[float, float, float], angle: float = 0.0
) -> trimesh.Trimesh:
    transform = trimesh.transformations.translation_matrix(position)
    if angle:
        transform = transform @ trimesh.transformations.rotation_matrix(angle, (0.0, 0.0, 1.0))
    return trimesh.creation.box(extents=extents, transform=transform)


def mesh_corridor_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshCorridorTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate two walls with exact-width centered openings."""
    wall_width = float(np.random.uniform(*cfg.wall_width_range))
    wall_height = float(np.random.uniform(*cfg.wall_height_range))
    platform_min, platform_max = cfg.center_platform_width_range
    if platform_min <= 0.0 or platform_min > platform_max:
        raise ValueError(f"Invalid center_platform_width_range: {cfg.center_platform_width_range}.")
    center_platform_width = float(np.random.uniform(platform_min, platform_max))
    gap_width = _interpolate_decreasing_range(cfg.wall_spacing_range, difficulty)
    usable_x = cfg.size[0] - 2.0 * cfg.border_width
    usable_y = cfg.size[1] - 2.0 * cfg.border_width
    if min(wall_width, wall_height, gap_width) <= 0.0:
        raise ValueError("Wall width, height, and opening must be positive.")
    if 2.0 * wall_width + center_platform_width > usable_x or gap_width >= usable_y:
        raise ValueError("Corridor walls and openings must fit within the terrain.")

    center_x, center_y = cfg.size[0] / 2.0, cfg.size[1] / 2.0
    origin = np.array([center_x, center_y, 0.0])
    meshes, origin = apply_mesh_surface_details(
        [make_plane(cfg.size, height=0.0, center_zero=False)], origin, cfg, difficulty
    )
    segment_length = (usable_y - gap_width) / 2.0
    wall_x_offset = (wall_width + center_platform_width) / 2.0
    wall_y_offset = (gap_width + segment_length) / 2.0
    for x in (center_x - wall_x_offset, center_x + wall_x_offset):
        for y in (center_y - wall_y_offset, center_y + wall_y_offset):
            meshes.append(_rotated_box((wall_width, segment_length, wall_height), (x, y, wall_height / 2.0)))
    return meshes, origin


def mesh_circular_doors_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshCircularDoorsTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate concentric circular walls with evenly spaced door openings."""
    inner_radius = cfg.inner_radius + np.random.uniform(*cfg.inner_radius_noise_range)
    ring_spacing = cfg.ring_spacing + np.random.uniform(*cfg.ring_spacing_noise_range)
    wall_arc_length = cfg.wall_arc_length + np.random.uniform(*cfg.wall_arc_length_noise_range)
    door_width = cfg.door_width + np.random.uniform(*cfg.door_width_noise_range)
    if (
        wall_arc_length <= 0.0
        or door_width <= 0.0
        or ring_spacing <= 0.0
        or cfg.wall_panel_length <= 0.0
        or cfg.wall_thickness <= 0.0
        or inner_radius <= 0.0
    ):
        raise ValueError(
            "inner_radius, wall_arc_length, door_width, ring_spacing, wall_panel_length, and wall_thickness "
            "must be positive."
        )
    wall_height = _interpolate_range(cfg.wall_height_range, difficulty)
    center_x, center_y = cfg.size[0] / 2.0, cfg.size[1] / 2.0
    max_radius = min(cfg.size) / 2.0 - cfg.border_width - cfg.wall_thickness / 2.0
    if inner_radius > max_radius:
        raise ValueError(f"Circular door inner radius {inner_radius:.3f} exceeds terrain bounds {max_radius:.3f}.")

    meshes = [make_plane(cfg.size, height=0.0, center_zero=False)]
    radius = inner_radius
    while radius <= max_radius + 1e-9:
        unit_length = wall_arc_length + door_width
        slot_count = max(1, int(np.floor(2.0 * np.pi * radius / unit_length)))
        unit_angle = 2.0 * np.pi / slot_count
        wall_angle = unit_angle * wall_arc_length / unit_length
        door_angle = unit_angle - wall_angle
        theta = -0.5 * np.pi
        for _ in range(slot_count):
            wall_panel_count = max(1, int(np.ceil(radius * wall_angle / cfg.wall_panel_length)))
            panel_angle = wall_angle / wall_panel_count
            for panel_index in range(wall_panel_count):
                panel_theta = theta + (panel_index + 0.5) * panel_angle
                chord = max(0.05, 2.0 * radius * np.sin(0.5 * panel_angle))
                meshes.append(
                    _rotated_box(
                        (cfg.wall_thickness, chord, wall_height),
                        (
                            center_x + radius * np.cos(panel_theta),
                            center_y + radius * np.sin(panel_theta),
                            wall_height / 2.0,
                        ),
                        panel_theta,
                    )
                )
            for edge_theta in (theta + wall_angle, theta + wall_angle + door_angle):
                meshes.append(
                    _rotated_box(
                        (cfg.wall_thickness, cfg.door_frame_width, wall_height),
                        (
                            center_x + radius * np.cos(edge_theta),
                            center_y + radius * np.sin(edge_theta),
                            wall_height / 2.0,
                        ),
                        edge_theta,
                    )
                )
            theta += unit_angle
        radius += ring_spacing
    return meshes, np.array([center_x, center_y, 0.0])


def mesh_ceiling_obstacles_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshRandomCeilingObstaclesTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate overhead blocks while keeping a clear central spawn area."""
    block_count = _sample_count(cfg.block_num_range, difficulty)
    clearance = _interpolate_decreasing_range(cfg.clearance_height_range, difficulty)
    meshes = [make_plane(cfg.size, height=0.0, center_zero=False)]
    center_x, center_y = cfg.size[0] / 2.0, cfg.size[1] / 2.0
    placed_blocks: list[tuple[float, float, float, float]] = []
    max_attempts = max(100, block_count * 30)
    for _ in range(max_attempts):
        if len(placed_blocks) >= block_count:
            break
        block_width = np.random.uniform(*cfg.block_width_range)
        block_depth = np.random.uniform(*cfg.block_width_range)
        block_thickness = np.random.uniform(*cfg.block_thickness_range)
        x = np.random.uniform(
            cfg.edge_margin + block_width / 2.0, cfg.size[0] - cfg.edge_margin - block_width / 2.0
        )
        y = np.random.uniform(
            cfg.edge_margin + block_depth / 2.0, cfg.size[1] - cfg.edge_margin - block_depth / 2.0
        )
        if (
            abs(x - center_x) < cfg.center_clearance + block_width / 2.0
            and abs(y - center_y) < cfg.center_clearance + block_depth / 2.0
        ):
            continue
        if any(
            abs(x - other_x) < (block_width + other_width) / 2.0
            and abs(y - other_y) < (block_depth + other_depth) / 2.0
            for other_x, other_y, other_width, other_depth in placed_blocks
        ):
            continue
        meshes.append(
            _rotated_box(
                (block_width, block_depth, block_thickness),
                (x, y, clearance + block_thickness / 2.0),
            )
        )
        placed_blocks.append((x, y, block_width, block_depth))
    return meshes, np.array([center_x, center_y, 0.0])


def mesh_hex_stepping_stones_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshHexSteppingStonesTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate hexagonally packed stepping stones over a configurable pit."""
    if cfg.pillar_radius_range[0] <= 0.0 or cfg.pillar_spacing_range[0] <= 0.0:
        raise ValueError("Pillar radius and spacing must be positive.")
    radius = _interpolate_decreasing_range(cfg.pillar_radius_range, difficulty)
    spacing_target = _interpolate_range(cfg.pillar_spacing_range, difficulty)
    spacing = max(spacing_target, 2.0 * radius + cfg.pillar_clearance)
    if cfg.stone_height_noise_type not in ("random", "difficulty"):
        raise ValueError(
            "stone_height_noise_type must be either 'random' or 'difficulty', "
            f"got '{cfg.stone_height_noise_type}'."
        )
    noise_min, noise_max = cfg.stone_height_noise_range
    if noise_min > noise_max or cfg.pit_depth_range[0] + noise_min < 0.0:
        raise ValueError("stone_height_noise_range must be ordered and keep every stone at or above the pit floor.")
    pit_depth = np.random.uniform(*cfg.pit_depth_range)
    if pit_depth <= 0.0:
        raise ValueError(f"pit_depth_range must contain positive values, got {cfg.pit_depth_range}.")
    width_min, width_max = cfg.center_platform_width_range
    if width_min <= 0.0 or width_min > width_max:
        raise ValueError(f"Invalid center_platform_width_range: {cfg.center_platform_width_range}.")
    center_platform_width = float(np.random.uniform(width_min, width_max))
    center_x, center_y = cfg.size[0] / 2.0, cfg.size[1] / 2.0
    meshes = [make_plane(cfg.size, height=-pit_depth, center_zero=False)]
    if cfg.border_width > 0.0:
        inner_size = (cfg.size[0] - 2.0 * cfg.border_width, cfg.size[1] - 2.0 * cfg.border_width)
        meshes.extend(make_border(cfg.size, inner_size, pit_depth, (center_x, center_y, -pit_depth / 2.0)))
    row_spacing = spacing * np.sqrt(3.0) / 2.0
    half_x = 0.5 * cfg.size[0]
    half_y = 0.5 * cfg.size[1]
    max_row = int(np.floor((half_y - cfg.edge_margin - radius) / row_spacing))
    max_x_extent = half_x - cfg.edge_margin - radius
    platform_half = 0.5 * center_platform_width + radius
    stone_positions: list[tuple[float, float]] = []
    for row in range(-max_row, max_row + 1):
        y = center_y + row * row_spacing
        offset = 0.5 if row % 2 else 0.0
        max_index = int(np.floor(max_x_extent / spacing - offset + 1e-9))
        if max_index < 0:
            continue
        index_min = -max_index - 1 if offset else -max_index
        for index in range(index_min, max_index + 1):
            x = center_x + (index + offset) * spacing
            if abs(x - center_x) <= platform_half and abs(y - center_y) <= platform_half:
                continue
            if not cfg.cross_shape or abs(y - center_y) <= platform_half:
                stone_positions.append((x, y))

    if cfg.cross_shape:
        symmetric_positions: set[tuple[float, float]] = set()
        for x, y in stone_positions:
            dx, dy = x - center_x, y - center_y
            for rotated_x, rotated_y in ((dx, dy), (-dy, dx), (-dx, -dy), (dy, -dx)):
                candidate_x, candidate_y = center_x + rotated_x, center_y + rotated_y
                if (
                    cfg.edge_margin + radius <= candidate_x <= cfg.size[0] - cfg.edge_margin - radius
                    and cfg.edge_margin + radius <= candidate_y <= cfg.size[1] - cfg.edge_margin - radius
                ):
                    symmetric_positions.add((round(candidate_x, 8), round(candidate_y, 8)))
        stone_positions = sorted(symmetric_positions)

    for x, y in stone_positions:
        height_offset = np.random.uniform(noise_min, noise_max)
        if cfg.stone_height_noise_type == "difficulty":
            height_offset *= np.clip(difficulty, 0.0, 1.0)
        meshes.append(
            trimesh.creation.cylinder(
                radius=radius,
                height=pit_depth + height_offset,
                sections=16,
                transform=trimesh.transformations.translation_matrix(
                    (x, y, (height_offset - pit_depth) / 2.0)
                ),
            )
        )
    platform_height = cfg.platform_thickness
    meshes.append(
        trimesh.creation.box(
            extents=(center_platform_width, center_platform_width, pit_depth + platform_height),
            transform=trimesh.transformations.translation_matrix(
                (center_x, center_y, (platform_height - pit_depth) / 2.0)
            ),
        )
    )
    return meshes, np.array([center_x, center_y, platform_height])


def mesh_pillar_forest_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshPillarForestTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate a randomly spaced forest of cylindrical pillars on a flat ground."""
    pillar_count = _sample_count(cfg.pillar_count_range, difficulty)
    if cfg.pillar_radius_range[0] <= 0.0 or cfg.pillar_height_range[0] <= 0.0:
        raise ValueError("Pillar radius and height ranges must be positive.")
    center_x, center_y = cfg.size[0] / 2.0, cfg.size[1] / 2.0
    meshes = [make_plane(cfg.size, height=0.0, center_zero=False)]
    pillars: list[tuple[float, float, float]] = []
    max_attempts = max(100, pillar_count * 50)
    for _ in range(max_attempts):
        if len(pillars) >= pillar_count:
            break
        radius = float(np.random.uniform(*cfg.pillar_radius_range))
        height = float(np.random.uniform(*cfg.pillar_height_range))
        x = float(np.random.uniform(cfg.edge_margin + radius, cfg.size[0] - cfg.edge_margin - radius))
        y = float(np.random.uniform(cfg.edge_margin + radius, cfg.size[1] - cfg.edge_margin - radius))
        if np.hypot(x - center_x, y - center_y) < cfg.center_clearance + radius:
            continue
        if any(
            np.hypot(x - other_x, y - other_y) < radius + other_radius + cfg.pillar_clearance
            for other_x, other_y, other_radius in pillars
        ):
            continue
        meshes.append(
            trimesh.creation.cylinder(
                radius=radius,
                height=height,
                sections=16,
                transform=trimesh.transformations.translation_matrix((x, y, height / 2.0)),
            )
        )
        pillars.append((x, y, radius))
    return meshes, np.array([center_x, center_y, 0.0])


def mesh_ring_platforms_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshRingPlatformsTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate concentric raised square platform rings."""
    if cfg.ring_width <= 0.0:
        raise ValueError("ring_width must be positive.")
    center_platform_width = np.random.uniform(*cfg.center_platform_width_range)
    if center_platform_width <= 0.0:
        raise ValueError("Sampled center_platform_width must be positive.")
    height = _interpolate_range(cfg.ring_platform_height_range, difficulty)
    gap = _interpolate_decreasing_range(cfg.ring_gap_range, difficulty)
    if gap < 0.0:
        raise ValueError("ring_gap_range must be non-negative.")
    center_x, center_y = cfg.size[0] / 2.0, cfg.size[1] / 2.0
    meshes = [make_plane(cfg.size, height=0.0, center_zero=False)]
    max_half_width = min(cfg.size) / 2.0 - cfg.border_width
    inner_half_width = center_platform_width / 2.0
    while inner_half_width + cfg.ring_width <= max_half_width:
        outer_half_width = inner_half_width + cfg.ring_width
        y_offset = outer_half_width - cfg.ring_width / 2.0
        meshes.append(
            _rotated_box(
                (2.0 * outer_half_width, cfg.ring_width, height),
                (center_x, center_y - y_offset, height / 2.0),
            )
        )
        meshes.append(
            _rotated_box(
                (2.0 * outer_half_width, cfg.ring_width, height),
                (center_x, center_y + y_offset, height / 2.0),
            )
        )
        x_offset = outer_half_width - cfg.ring_width / 2.0
        meshes.append(
            _rotated_box(
                (cfg.ring_width, 2.0 * inner_half_width, height),
                (center_x - x_offset, center_y, height / 2.0),
            )
        )
        meshes.append(
            _rotated_box(
                (cfg.ring_width, 2.0 * inner_half_width, height),
                (center_x + x_offset, center_y, height / 2.0),
            )
        )
        inner_half_width += cfg.ring_width + gap
    return meshes, np.array([center_x, center_y, 0.0])


def mesh_maze_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshMazeTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate a connected grid maze with metric wall geometry."""
    wall_height = _interpolate_range(cfg.wall_height_range, difficulty)
    if cfg.cell_size <= cfg.wall_thickness or cfg.cell_size <= 0.0:
        raise ValueError("cell_size must be greater than wall_thickness and positive.")
    usable_x = cfg.size[0] - 2.0 * cfg.border_width
    usable_y = cfg.size[1] - 2.0 * cfg.border_width
    columns = max(1, int(usable_x / cfg.cell_size))
    rows = max(1, int(usable_y / cfg.cell_size))
    maze_width = columns * cfg.cell_size
    maze_length = rows * cfg.cell_size
    origin_x = (cfg.size[0] - maze_width) / 2.0
    origin_y = (cfg.size[1] - maze_length) / 2.0
    terrain_center_x = cfg.size[0] / 2.0
    terrain_center_y = cfg.size[1] / 2.0
    visited = {(0, 0)}
    stack = [(0, 0)]
    passages: set[frozenset[tuple[int, int]]] = set()
    while stack:
        cell = stack[-1]
        x, y = cell
        candidates = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            next_cell = (x + dx, y + dy)
            if 0 <= next_cell[0] < columns and 0 <= next_cell[1] < rows and next_cell not in visited:
                candidates.append(next_cell)
        if not candidates:
            stack.pop()
            continue
        next_cell = candidates[np.random.randint(len(candidates))]
        passages.add(frozenset((cell, next_cell)))
        visited.add(next_cell)
        stack.append(next_cell)

    meshes = [make_plane(cfg.size, height=0.0, center_zero=False)]
    wall_thickness = cfg.wall_thickness
    for x in range(1, columns):
        for y in range(rows):
            if frozenset(((x - 1, y), (x, y))) in passages:
                continue
            meshes.append(
                _rotated_box(
                    (wall_thickness, cfg.cell_size, wall_height),
                    (origin_x + x * cfg.cell_size, origin_y + (y + 0.5) * cfg.cell_size, wall_height / 2.0),
                )
            )
    for x in range(columns):
        for y in range(1, rows):
            if frozenset(((x, y - 1), (x, y))) in passages:
                continue
            meshes.append(
                _rotated_box(
                    (cfg.cell_size, wall_thickness, wall_height),
                    (origin_x + (x + 0.5) * cfg.cell_size, origin_y + y * cfg.cell_size, wall_height / 2.0),
                )
            )
    meshes.extend(
        [
            _rotated_box((maze_width, wall_thickness, wall_height), (wall_x, wall_y, wall_height / 2.0))
            for wall_x, wall_y in (
                (origin_x + maze_width / 2.0, origin_y),
                (origin_x + maze_width / 2.0, origin_y + maze_length),
            )
        ]
    )
    meshes.extend(
        [
            _rotated_box(
                (wall_thickness, maze_length, wall_height),
                (origin_x, origin_y + maze_length / 2.0, wall_height / 2.0),
            ),
            _rotated_box(
                (wall_thickness, maze_length, wall_height),
                (origin_x + maze_width, origin_y + maze_length / 2.0, wall_height / 2.0),
            ),
        ]
    )
    return meshes, np.array([terrain_center_x, terrain_center_y, 0.0])
