# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

import warp as wp


def launch_warp_on_torch_stream(kernel, *, dim: int, inputs: list, device) -> None:
    """Run a Warp kernel on the current PyTorch CUDA stream.

    Later PyTorch kernels on that stream then see these writes without a host synchronize.
    CPU devices have no PyTorch stream, so the launch runs on the Warp CPU device instead.
    """
    device_str = str(device)
    if device_str.startswith("cuda"):
        wp.launch(kernel, dim=dim, inputs=inputs, device=device_str, stream=wp.stream_from_torch())
    else:
        wp.launch(kernel, dim=dim, inputs=inputs, device=device_str)


@wp.func
def quat_apply_xyzw(qw: wp.float32, qx: wp.float32, qy: wp.float32, qz: wp.float32, v: wp.vec3) -> wp.vec3:
    """Rotate ``v`` by an xyzw quaternion. Matches ``isaaclab.utils.math.quat_apply`` after wxyz conversion."""
    qv = wp.vec3(qx, qy, qz)
    t = wp.cross(qv, v) * 2.0
    return v + qw * t + wp.cross(qv, t)


@wp.func
def cylinder_penetration_offset(
    p: wp.vec3,
    cylinder_start: wp.array(dtype=wp.vec3),
    cylinder_end: wp.array(dtype=wp.vec3),
    cylinder_thinkness: wp.array(dtype=wp.float32),
    cell_offsets: wp.array(dtype=wp.int32),
    cell_indices: wp.array(dtype=wp.int32),
    grid_res: wp.vec3i,
    bbox_min: wp.vec3,
    cell_size: wp.vec3,
) -> wp.vec3:
    """Deepest penetration of ``p`` into the cylinders of its grid cell. Zero when it misses."""
    bbox_min_to_p = p - bbox_min
    ix = int(bbox_min_to_p[0] / cell_size[0])
    iy = int(bbox_min_to_p[1] / cell_size[1])
    iz = int(bbox_min_to_p[2] / cell_size[2])

    # Cylinders are inserted into every cell overlapping their radius-expanded AABB, so the
    # cell containing the query point is sufficient. Points outside the grid cannot hit.
    if ix < 0 or ix >= grid_res.x or iy < 0 or iy >= grid_res.y or iz < 0 or iz >= grid_res.z:
        return wp.vec3(0.0, 0.0, 0.0)

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

    return penetrate_offset_


@wp.func
def deeper_penetration(hit: wp.vec3, existing: wp.vec3) -> wp.vec3:
    """Return whichever offset has the larger depth. Depth is the vector length."""
    if wp.length(hit) > wp.length(existing):
        return hit
    return existing


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
    """Merge this grid's hit with the offset already stored for each point."""
    tid = wp.tid()
    hit = cylinder_penetration_offset(
        points[tid],
        cylinder_start,
        cylinder_end,
        cylinder_thinkness,
        cell_offsets,
        cell_indices,
        grid_res,
        bbox_min,
        cell_size,
    )
    penetrate_offset[tid] = deeper_penetration(hit, penetrate_offset[tid])


@wp.kernel(enable_backward=False)
def points_penetrate_cylinder_selected_kernel(
    env_ids: wp.array(dtype=wp.int32),
    num_bodies: wp.int32,
    num_points: wp.int32,
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
    """Merge this grid's hit for ``env_ids`` only.

    ``points`` and ``penetrate_offset`` are flattened ``(num_envs, num_bodies, num_points)``.
    Thread ``tid`` walks selected environment, then body, then point.
    """
    tid = wp.tid()
    stride = num_bodies * num_points
    local_env = tid // stride
    point_in_env = tid - local_env * stride
    src = env_ids[local_env] * stride + point_in_env
    hit = cylinder_penetration_offset(
        points[src],
        cylinder_start,
        cylinder_end,
        cylinder_thinkness,
        cell_offsets,
        cell_indices,
        grid_res,
        bbox_min,
        cell_size,
    )
    penetrate_offset[src] = deeper_penetration(hit, penetrate_offset[src])


@wp.kernel(enable_backward=False)
def refresh_volume_points_kernel(
    env_ids: wp.array(dtype=wp.int32),
    num_bodies: wp.int32,
    body_poses: wp.array2d(dtype=wp.float32),
    body_vels: wp.array2d(dtype=wp.float32),
    pattern: wp.array(dtype=wp.vec3),
    pos_w: wp.array(dtype=wp.vec3),
    quat_w: wp.array(dtype=wp.vec4),
    vel_w: wp.array(dtype=wp.vec3),
    ang_vel_w: wp.array(dtype=wp.vec3),
    points_pos_w: wp.array(dtype=wp.vec3),
    points_vel_w: wp.array(dtype=wp.vec3),
):
    """Write body pose and per-point world position/velocity for the listed environments.

    ``body_poses`` rows are PhysX transforms ``(x, y, z, qx, qy, qz, qw)``. ``body_vels`` rows are
    ``(vx, vy, vz, wx, wy, wz)``. Both are flattened env-major over every environment.
    ``quat_w`` is stored ``(w, x, y, z)``.
    """
    # One thread per selected body. ``src`` is that body's row in the full env-major buffers.
    local = wp.tid()
    local_env = local // num_bodies
    body_i = local - local_env * num_bodies
    src = env_ids[local_env] * num_bodies + body_i

    origin = wp.vec3(body_poses[src, 0], body_poses[src, 1], body_poses[src, 2])
    qx = body_poses[src, 3]
    qy = body_poses[src, 4]
    qz = body_poses[src, 5]
    qw = body_poses[src, 6]
    lin = wp.vec3(body_vels[src, 0], body_vels[src, 1], body_vels[src, 2])
    ang = wp.vec3(body_vels[src, 3], body_vels[src, 4], body_vels[src, 5])

    pos_w[src] = origin
    quat_w[src] = wp.vec4(qw, qx, qy, qz)
    vel_w[src] = lin
    ang_vel_w[src] = ang

    p_count = pattern.shape[0]
    base = src * p_count
    for i in range(p_count):
        world = quat_apply_xyzw(qw, qx, qy, qz, pattern[i]) + origin
        points_pos_w[base + i] = world
        points_vel_w[base + i] = lin + wp.cross(ang, world - origin)
