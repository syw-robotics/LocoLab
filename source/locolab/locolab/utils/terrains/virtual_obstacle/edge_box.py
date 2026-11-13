"""Fill the vertical gaps between adjacent, axis-aligned floating treads."""

from collections import defaultdict

import numpy as np
import torch
from isaaclab.markers import VisualizationMarkers

from locolab.utils.warp.box import BoxSpatialGrid

from .edge_cylinder import GreedyconcatEdgeCylinder


class EdgeBox(GreedyconcatEdgeCylinder):
    def generate(self, mesh, device="cpu") -> None:
        if self.cfg.downward_extent < 0.0 or self.cfg.margin <= 0.0:
            raise ValueError("Edge boxes need downward_extent >= 0 and margin > 0.")
        mesh = mesh.copy()
        mesh.merge_vertices()
        edges = self._filter_edges_by_direction(self._upper_surface_edges(mesh))
        if edges.size:
            edges = self.process_edges(edges)
            if self.cfg.terrain_size is not None and edges.size:
                edges = self._filter_boundary_edges(edges)
        edges = np.asarray(edges, dtype=np.float32).reshape(-1, 6)
        directions = np.abs(edges[:, 3:] - edges[:, :3])
        if np.any(directions[:, 2] > 1e-5) or np.any(np.count_nonzero(directions > 1e-5, axis=1) != 1):
            raise ValueError("EdgeBox supports horizontal, axis-aligned terrain edges only.")
        bounds = self._box_bounds(edges)
        self.device = torch.device(device)
        self.edges_pyt = torch.as_tensor(edges, device=self.device)
        self.box_bounds_pyt = torch.as_tensor(bounds, device=self.device)
        self.boxes = BoxSpatialGrid(bounds, self.cfg.num_grid_cells, self.device) if len(bounds) else None
        print(f"Detected {len(bounds)} edge boxes after processing.")

    def _box_bounds(self, edges):
        """Connect each upper lip to the nearest lower lip sharing its xy boundary.

        Split partial overlaps so a neighboring tread only fills its own span. This
        runs once at terrain import; the rollout still queries the same box grid.
        """
        groups = defaultdict(list)
        for edge in edges:
            axis = int(np.argmax(np.abs(edge[3:5] - edge[:2])))
            groups[axis, round(float(edge[1 - axis]), 5)].append(edge)
        bounds = []
        for (axis, _), group in groups.items():
            group = np.asarray(group)
            starts = np.minimum(group[:, axis], group[:, axis + 3])
            ends = np.maximum(group[:, axis], group[:, axis + 3])
            heights = group[:, 2]
            for i, edge in enumerate(group):
                neighbors = (
                    (heights < heights[i] - 1e-5)
                    & (starts < ends[i] - 1e-5) & (ends > starts[i] + 1e-5)
                )
                cuts = np.unique(np.clip(
                    np.r_[starts[i], ends[i], starts[neighbors], ends[neighbors]], starts[i], ends[i]
                ))
                for left, right in zip(cuts[:-1], cuts[1:]):
                    if right - left <= 1e-5:
                        continue
                    midpoint = (left + right) * 0.5
                    below = neighbors & (starts < midpoint) & (ends > midpoint)
                    bottom = heights[i] - self.cfg.downward_extent
                    if np.any(below):
                        bottom = min(bottom, heights[below].max())
                    lower = np.minimum(edge[:3], edge[3:]).copy()
                    upper = np.maximum(edge[:3], edge[3:]).copy()
                    lower[axis], upper[axis] = left, right
                    lower[2] = bottom
                    bounds.append(np.r_[lower - self.cfg.margin, upper + self.cfg.margin])
        return np.asarray(bounds, dtype=np.float32).reshape(-1, 6)

    def get_points_penetration_offset(self, points, out=None):
        if getattr(self, "boxes", None) is not None:
            return self.boxes.get_points_penetration_offset(points, out=out)
        return torch.zeros_like(points) if out is None else out

    def accumulate_selected_penetration(self, points, out, env_ids, num_bodies, num_points):
        if getattr(self, "boxes", None) is not None:
            self.boxes.accumulate_selected_penetration(points, out, env_ids, num_bodies, num_points)

    def visualize(self):
        if not hasattr(self, "box_bounds_pyt") or self.box_bounds_pyt.numel() == 0:
            return
        if not hasattr(self, "_box_visualizer"):
            self._box_visualizer = VisualizationMarkers(self.cfg.visualizer)
        lower, upper = self.box_bounds_pyt[:, :3], self.box_bounds_pyt[:, 3:]
        self._box_visualizer.visualize(translations=(lower + upper) * 0.5, scales=upper - lower)
        self._box_visualizer.set_visibility(True)

    def disable_visualizer(self):
        if hasattr(self, "_box_visualizer"):
            self._box_visualizer.set_visibility(False)
