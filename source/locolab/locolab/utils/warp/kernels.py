# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

import warp as wp


@wp.kernel(enable_backward=False)
def points_penetrate_cylinder_kernel(
    points: wp.array(dtype=wp.vec3),
    cylinder_start: wp.array(dtype=wp.vec3),
    cylinder_end: wp.array(dtype=wp.vec3),
    cylinder_thinkness: wp.array(dtype=wp.float32),
    cell_offsets: wp.array(dtype=wp.int32),
    cell_indices: wp.array(dtype=wp.int32),
    grid_res: wp.vec3i,
    bbox_min: wp.vec3,
    cell_size: wp.vec3,
    penetrate_offset: wp.array(dtype=wp.vec3),
):
    """Compute the penetration depth of points into cylinders in a grid. Return the maximum depth for each point if it
    penetrates any cylinder.
    Args:
        points: Array of points to check for penetration. shape (N, 3) where N is the number of points.
        cylinders: Array of cylinders defined by start and end points and radius. shape (M, 7) where M is the number of cylinders.
        cell_offsets: Offsets for each grid cell in the flattened grid. shape (grid_res^3 + 1,)
        cell_indices: Indices of cylinders in each grid cell. shape (N, 8)
        grid_res: Resolution of the grid.
        bbox_min: Minimum coordinates of the bounding box for the grid. shape (3,)
        cell_size: Size of each grid cell. shape (3,)
        penetrate_offset: Output array to store the penetration offset from the surface of the cylinder to each point.
    """
    tid = wp.tid()
    p = points[tid]

    bbox_min_to_p = p - bbox_min
    ix = int(bbox_min_to_p[0] / cell_size[0])
    iy = int(bbox_min_to_p[1] / cell_size[1])
    iz = int(bbox_min_to_p[2] / cell_size[2])

    # Cylinders are inserted into every cell overlapping their radius-expanded AABB, so the
    # cell containing the query point is sufficient. Points outside the grid cannot hit.
    if ix < 0 or ix >= grid_res.x or iy < 0 or iy >= grid_res.y or iz < 0 or iz >= grid_res.z:
        return

    depth = float(0.0)
    penetrate_offset_ = wp.vec3(0.0, 0.0, 0.0)

    flat = ix * grid_res.y * grid_res.z + iy * grid_res.z + iz
    start = cell_offsets[flat]
    end = cell_offsets[flat + 1]

    for i in range(start, end):
        cid = cell_indices[i]
        a = cylinder_start[cid]
        b = cylinder_end[cid]
        r = cylinder_thinkness[cid]

        ab = b - a
        ab_len = wp.length(ab)
        if ab_len < 1.0e-8:
            continue
        ab_dir = ab / ab_len
        ap = p - a
        t = wp.dot(ap, ab_dir)

        if t < 0.0 or t > ab_len:
            continue

        proj = a + t * ab_dir
        dist = wp.length(p - proj)

        if dist < r:
            d = r - dist
            if d > depth:
                depth = d
                inv_dist = d / wp.max(dist, 1.0e-8)
                offset_ = (proj - p) * inv_dist
                penetrate_offset_.x = offset_.x
                penetrate_offset_.y = offset_.y
                penetrate_offset_.z = offset_.z

    if depth > 0.0:
        penetrate_offset[tid] = penetrate_offset_
