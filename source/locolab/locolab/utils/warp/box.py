"""Static axis-aligned boxes with spatially indexed, in-place penetration queries."""

from collections import defaultdict

import numpy as np
import torch
import warp as wp

from .kernels import deeper_penetration, launch_warp_on_torch_stream


@wp.kernel(enable_backward=False)
def points_penetrate_box_kernel(
    points: wp.array(dtype=wp.vec3),
    out: wp.array(dtype=wp.vec3),
    env_ids: wp.array(dtype=wp.int32),
    stride: int,
    lower: wp.array(dtype=wp.vec3),
    upper: wp.array(dtype=wp.vec3),
    offsets: wp.array(dtype=wp.int32),
    indices: wp.array(dtype=wp.int32),
    origin: wp.vec3,
    cell_size: wp.vec3,
    resolution: wp.vec3i,
):
    tid = wp.tid()
    src = tid
    if stride > 0:
        src = env_ids[tid // stride] * stride + tid % stride
    p = points[src]
    cell = wp.cw_div(p - origin, cell_size)
    ix, iy, iz = int(wp.floor(cell[0])), int(wp.floor(cell[1])), int(wp.floor(cell[2]))
    if ix < 0 or iy < 0 or iz < 0 or ix >= resolution[0] or iy >= resolution[1] or iz >= resolution[2]:
        return
    flat = (ix * resolution[1] + iy) * resolution[2] + iz
    hit = wp.vec3(0.0)
    deepest = float(0.0)
    for j in range(offsets[flat], offsets[flat + 1]):
        box = indices[j]
        near_lower, near_upper = p - lower[box], upper[box] - p
        depth = float(1.0e30)
        axis = int(0)
        direction = float(1.0)
        for k in range(3):
            if near_lower[k] < depth:
                depth, axis, direction = near_lower[k], k, -1.0
            if near_upper[k] < depth:
                depth, axis, direction = near_upper[k], k, 1.0
        if depth > deepest:
            deepest = depth
            hit = wp.vec3(0.0)
            hit[axis] = direction * depth
    out[src] = deeper_penetration(hit, out[src])


class BoxSpatialGrid:
    """Each box is ``(xmin, ymin, zmin, xmax, ymax, zmax)``; queries visit one cell."""

    def __init__(self, bounds: np.ndarray, num_grid_cells: int = 64**3, device="cpu"):
        lower, upper = bounds[:, :3], bounds[:, 3:]
        origin = lower.min(axis=0)
        extent = upper.max(axis=0) - origin
        resolution = np.maximum(np.round(extent / extent.max() * num_grid_cells ** (1 / 3)), 1).astype(int)
        cell_size = extent / resolution
        grid = defaultdict(list)
        for i, (lo, hi) in enumerate(zip(lower, upper)):
            first = np.maximum(np.floor((lo - origin) / cell_size).astype(int), 0)
            last = np.minimum(np.floor((hi - origin) / cell_size).astype(int), resolution - 1)
            for x in range(first[0], last[0] + 1):
                for y in range(first[1], last[1] + 1):
                    for z in range(first[2], last[2] + 1):
                        grid[(x * resolution[1] + y) * resolution[2] + z].append(i)
        offsets = np.zeros(int(np.prod(resolution)) + 1, dtype=np.int32)
        indices = []
        for cell in range(len(offsets) - 1):
            indices.extend(grid.get(cell, ()))
            offsets[cell + 1] = len(indices)
        self._args = [
            wp.array(lower, dtype=wp.vec3, device=str(device)),
            wp.array(upper, dtype=wp.vec3, device=str(device)),
            wp.array(offsets, dtype=wp.int32, device=str(device)),
            wp.array(indices, dtype=wp.int32, device=str(device)),
            wp.vec3(*origin), wp.vec3(*cell_size), wp.vec3i(*resolution),
        ]
        self._empty_ids = torch.empty(0, dtype=torch.int32, device=device)
        self._offset_buf = None

    def _query(self, points, out, env_ids, stride, count):
        if count == 0:
            return
        launch_warp_on_torch_stream(
            points_penetrate_box_kernel,
            dim=count,
            inputs=[
                wp.from_torch(points.reshape(-1, 3), dtype=wp.vec3),
                wp.from_torch(out.reshape(-1, 3), dtype=wp.vec3),
                wp.from_torch(env_ids, dtype=wp.int32), stride, *self._args,
            ],
            device=points.device,
        )

    def get_points_penetration_offset(self, points: torch.Tensor, out: torch.Tensor | None = None):
        if out is None:
            if self._offset_buf is None or self._offset_buf.shape != points.shape:
                self._offset_buf = torch.empty_like(points)
            out = self._offset_buf.zero_()
        if out.shape != points.shape or not out.is_contiguous():
            raise ValueError("out must be contiguous and have the same shape as points.")
        self._query(points.contiguous(), out, self._empty_ids, 0, len(points))
        return out

    def accumulate_selected_penetration(self, points, out, env_ids, num_bodies, num_points):
        if not points.is_contiguous() or not out.is_contiguous():
            raise ValueError("points and out must be contiguous.")
        stride = num_bodies * num_points
        env_ids = env_ids.to(device=points.device, dtype=torch.int32).reshape(-1)
        self._query(points, out, env_ids, stride, env_ids.numel() * stride)
