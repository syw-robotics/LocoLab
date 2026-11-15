"""GPU scan, measurement, and publication kernels for the MID360."""

import warp as wp


@wp.kernel(enable_backward=False)
def scan(
    mask: wp.array(dtype=wp.bool),
    table: wp.array(dtype=wp.vec3),
    cursor: wp.array(dtype=wp.int32),
    slots: wp.array(dtype=wp.int32),
    pos: wp.array(dtype=wp.vec3),
    quat: wp.array(dtype=wp.quat),
    meshes: wp.array2d(dtype=wp.uint64),
    mesh_pos: wp.array2d(dtype=wp.vec3),
    mesh_quat: wp.array2d(dtype=wp.quat),
    reflectance: wp.array(dtype=float),
    noise: bool,
    seed: int,
    blind: float,
    range10: float,
    range80: float,
    angle_sigma: float,
    false_alarm_rate: float,
    resolution: float,
    points: wp.array2d(dtype=wp.vec3),
    world: wp.array2d(dtype=wp.vec3),
    valid: wp.array2d(dtype=wp.bool),
    reflectivity: wp.array2d(dtype=wp.uint8),
    hits: wp.array2d(dtype=wp.vec3),
):
    e, r = wp.tid()
    if not mask[e]:
        return
    direction = table[(cursor[e] + r) % table.shape[0]]
    direction_w = wp.quat_rotate(quat[e], direction)
    distance = float(range80)
    rho = float(0.0)
    found = bool(False)
    # Resolve the closest hit in one thread, keeping distance and normal consistent.
    for m in range(meshes.shape[1]):
        inverse = wp.transform_inverse(wp.transform(mesh_pos[e, m], mesh_quat[e, m]))
        local_dir = wp.transform_vector(inverse, direction_w)
        query = wp.mesh_query_ray(meshes[e, m], wp.transform_point(inverse, pos[e]), local_dir, distance)
        if query.result:
            found = True
            distance = query.t
            rho = reflectance[m] * wp.abs(wp.dot(query.normal, local_dir))
    exponent = wp.log(range80 / range10) / wp.log(8.0)
    limit = range10 * wp.pow(wp.max(rho / 0.1, 1.0e-9), exponent)
    ok = found and distance >= blind and distance <= limit
    strength = wp.round(rho * 150.0)
    rng = wp.rand_init(seed + e, cursor[e] + r)
    if noise:
        if wp.randf(rng) < false_alarm_rate:
            far = float(range10)
            if ok:
                far = distance
            distance = blind + wp.randf(rng) * (far - blind)
            strength = wp.floor(wp.randf(rng) * 10.0)
            ok = True
        sigma = 0.03 - 0.01 * wp.clamp((distance - 0.2) / 9.8, 0.0, 1.0)
        distance += wp.randn(rng) * sigma
        axis = wp.vec3(0.0, 0.0, 1.0)
        if wp.abs(direction[2]) > 0.99:
            axis = wp.vec3(1.0, 0.0, 0.0)
        tangent = wp.normalize(wp.cross(direction, axis))
        bitangent = wp.cross(direction, tangent)
        direction = wp.normalize(direction + angle_sigma * (wp.randn(rng) * tangent + wp.randn(rng) * bitangent))
    point = wp.vec3(0.0)
    point_w = wp.vec3(0.0)
    if ok:
        point = distance * direction
        point = (
            wp.vec3(wp.round(point[0] / resolution), wp.round(point[1] / resolution), wp.round(point[2] / resolution))
            * resolution
        )
        point_w = pos[e] + wp.quat_rotate(quat[e], point)
    s = slots[e] + r
    points[e, s] = point
    world[e, s] = point_w
    valid[e, s] = ok
    reflectivity[e, s] = wp.uint8(wp.clamp(strength, 0.0, 150.0))
    hits[e, r] = point_w if ok else wp.vec3(wp.inf)


@wp.kernel(enable_backward=False)
def publish(
    mask: wp.array(dtype=wp.bool),
    slots: wp.array(dtype=wp.int32),
    batch: int,
    points: wp.array2d(dtype=wp.vec3),
    world: wp.array2d(dtype=wp.vec3),
    valid: wp.array2d(dtype=wp.bool),
    refl: wp.array2d(dtype=wp.uint8),
    output_points: wp.array2d(dtype=wp.vec3),
    output_world: wp.array2d(dtype=wp.vec3),
    output_valid: wp.array2d(dtype=wp.bool),
    output_refl: wp.array2d(dtype=wp.uint8),
):
    e, r = wp.tid()
    if mask[e] and slots[e] + batch == points.shape[1]:
        output_points[e, r] = points[e, r]
        output_world[e, r] = world[e, r]
        output_valid[e, r] = valid[e, r]
        output_refl[e, r] = refl[e, r]


@wp.kernel(enable_backward=False)
def advance(
    mask: wp.array(dtype=wp.bool),
    slots: wp.array(dtype=wp.int32),
    cursor: wp.array(dtype=wp.int32),
    batch: int,
    frame_size: int,
    table_size: int,
    clock: wp.array(dtype=float),
    dt: float,
    pos: wp.array(dtype=wp.vec3),
    quat: wp.array(dtype=wp.quat),
    start_pos: wp.array(dtype=wp.vec3),
    start_quat: wp.array(dtype=wp.quat),
    output_pos: wp.array(dtype=wp.vec3),
    output_quat: wp.array(dtype=wp.quat),
    timebase: wp.array(dtype=float),
    frame_id: wp.array(dtype=wp.int32),
):
    e = wp.tid()
    if not mask[e]:
        return
    if slots[e] == 0:
        start_pos[e] = pos[e]
        start_quat[e] = quat[e]
    slots[e] += batch
    cursor[e] = (cursor[e] + batch) % (2147000000 - 2147000000 % table_size)
    if slots[e] == frame_size:
        output_pos[e] = start_pos[e]
        output_quat[e] = start_quat[e]
        timebase[e] = clock[e] - float(frame_size / batch) * dt
        frame_id[e] += 1
        slots[e] = 0


@wp.kernel(enable_backward=False)
def reset_frames(
    mask: wp.array(dtype=wp.bool),
    slots: wp.array(dtype=wp.int32),
    points: wp.array2d(dtype=wp.vec3),
    world: wp.array2d(dtype=wp.vec3),
    valid: wp.array2d(dtype=wp.bool),
    refl: wp.array2d(dtype=wp.uint8),
    frame_id: wp.array(dtype=wp.int32),
    timebase: wp.array(dtype=float),
    pos: wp.array(dtype=wp.vec3),
    quat: wp.array(dtype=wp.quat),
):
    e, r = wp.tid()
    if mask[e]:
        points[e, r] = wp.vec3(0.0)
        world[e, r] = wp.vec3(0.0)
        valid[e, r] = False
        refl[e, r] = wp.uint8(0)
        if r == 0:
            slots[e] = 0
            frame_id[e] = 0
            timebase[e] = 0.0
            pos[e] = wp.vec3(0.0)
            quat[e] = wp.quat_identity()


@wp.kernel(enable_backward=False)
def imu_noise(
    mask: wp.array(dtype=wp.bool),
    sample_index: wp.array(dtype=wp.int32),
    seed: int,
    gyro_sigma: float,
    accel_sigma: float,
    gyro: wp.array(dtype=wp.vec3),
    accel: wp.array(dtype=wp.vec3),
    accel_g: wp.array(dtype=wp.vec3),
):
    e = wp.tid()
    if mask[e]:
        sample_index[e] += 1
        rng = wp.rand_init(seed + e, sample_index[e])
        gyro[e] += gyro_sigma * wp.vec3(wp.randn(rng), wp.randn(rng), wp.randn(rng))
        accel[e] += (9.80665 * accel_sigma) * wp.vec3(wp.randn(rng), wp.randn(rng), wp.randn(rng))
        accel_g[e] = accel[e] / 9.80665


@wp.kernel(enable_backward=False)
def apply_mount(
    mask: wp.array(dtype=wp.bool),
    offset_pos: wp.vec3,
    offset_quat: wp.quat,
    pos: wp.array(dtype=wp.vec3),
    quat: wp.array(dtype=wp.quat),
):
    e = wp.tid()
    if mask[e]:
        pos[e] += wp.quat_rotate(quat[e], offset_pos)
        quat[e] = quat[e] * offset_quat
