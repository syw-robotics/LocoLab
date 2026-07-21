
"""LocoLab custom height-field terrains."""

from __future__ import annotations

import trimesh
from isaaclab.terrains.trimesh.utils import make_border
from typing import TYPE_CHECKING

import numpy as np


if TYPE_CHECKING:
    from . import locolab_mesh_terrains_cfg



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


def mesh_straight_stairs_terrain(
    difficulty: float, cfg: locolab_mesh_terrains_cfg.MeshStraightStairsTerrainCfg
) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    """Generate stairs terrain (up then down) in x direction."""
    # resolve the terrain configuration
    stair_height = cfg.stair_height_range[0] + difficulty * (cfg.stair_height_range[1] - cfg.stair_height_range[0])
    stair_width = cfg.stair_width + np.random.uniform(cfg.stair_width_noise_range[0], cfg.stair_width_noise_range[1])
    stair_length = (
        cfg.stair_length_range[1]
        if difficulty < cfg.easy_difficulty_threshold
        else np.random.uniform(cfg.stair_length_range[0], cfg.stair_length_range[1])
    )
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
    stair_length = (
        cfg.stair_length_range[1]
        if difficulty < cfg.easy_difficulty_threshold
        else np.random.uniform(cfg.stair_length_range[0], cfg.stair_length_range[1])
    )
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
