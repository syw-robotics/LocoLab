from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import MISSING

import torch
import trimesh
from isaaclab.markers import VisualizationMarkersCfg
from isaaclab.utils import configclass

MeshXyzRange = tuple[
    tuple[float | None, float | None],
    tuple[float | None, float | None],
    tuple[float | None, float | None],
]
"""Axis-aligned crop in a sub-terrain local frame, as ``((xmin, xmax), (ymin, ymax), (zmin, zmax))``.

Either side of an axis can be ``None`` to leave that bound open.
"""


@configclass
class VirtualObstacleCfg:
    """Configuration for a virtual obstacle."""

    class_type: type = MISSING
    """The class to use for the virtual obstacle."""

    visualizer: VisualizationMarkersCfg = MISSING
    """The visualizer configuration for the virtual obstacle."""

    debug_vis: bool = False
    """Whether to draw generated obstacles at terrain import time. Keep False for training."""

    terrain_names: list[str] | None = None
    """Sub-terrain names that should generate virtual obstacles.

    Combined with :attr:`terrain_mesh_xyz_ranges` keys. If both are None, all sub-terrains are used.
    """

    default_mesh_xyz_range: MeshXyzRange | None = None
    """Default local xyz crop applied to every selected sub-terrain.

    For generated terrains, the range is relative to each sub-terrain origin. Use
    :attr:`terrain_mesh_xyz_ranges` when different terrains need different crops.
    """

    terrain_mesh_xyz_ranges: dict[str, MeshXyzRange | None] | None = None
    """Per-terrain local xyz crops. Keys are sub-terrain names.

    A value of ``None`` keeps the whole mesh for that terrain. Names listed here are always
    selected, even if they are not in :attr:`terrain_names`. Terrains that only appear in
    :attr:`terrain_names` fall back to :attr:`default_mesh_xyz_range`.

    Example::

        terrain_names=["gap", "stairs_high", "climb"],
        default_mesh_xyz_range=((-3.9, 3.9), (-3.9, 3.9), (-1.0, 1.0)),
        terrain_mesh_xyz_ranges={
            "gap": ((-3.9, 3.9), (-1.5, 1.5), (-1.0, 0.6)),
            "stairs_high": ((-3.9, 3.9), (-3.9, 3.9), (-0.2, 1.8)),
            "climb": ((-2.0, 2.0), (-3.9, 3.9), (-0.5, 1.2)),
        }
    """

    def selected_terrain_names(self) -> list[str] | None:
        """Return the explicit sub-terrain names this obstacle applies to, or None for all."""
        names: list[str] = []
        if self.terrain_names:
            names.extend(self.terrain_names)
        if self.terrain_mesh_xyz_ranges:
            names.extend(self.terrain_mesh_xyz_ranges.keys())
        return names or None

    def mesh_xyz_range_for(self, terrain_name: str) -> MeshXyzRange | None:
        """Return the local xyz crop for one sub-terrain."""
        if self.terrain_mesh_xyz_ranges and terrain_name in self.terrain_mesh_xyz_ranges:
            return self.terrain_mesh_xyz_ranges[terrain_name]
        return self.default_mesh_xyz_range


class VirtualObstacleBase(ABC):
    def __init__(self, cfg: VirtualObstacleCfg):
        self.cfg = cfg

    @abstractmethod
    def generate(self, mesh: trimesh.Trimesh, device: torch.device | str = "cpu") -> None:
        """Generate the virtual obstacle mesh based on the provided terrain mesh.
        NOTE: This interface might be updated in the future to support more complex generation logic.

        Args:
            mesh (trimesh.Trimesh): The terrain mesh to generate the virtual obstacle from.

        """
        raise NotImplementedError("This method should be implemented by subclasses.")

    @abstractmethod
    def disable_visualizer(self) -> None:
        """Disable the visualizer for the virtual obstacle if there is one."""
        raise NotImplementedError("This method should be implemented by subclasses.")

    """
    Operations only after being generated.
    If called before generation, it should skip and print a warning.
    """

    @abstractmethod
    def visualize(self):
        """Visualize the virtual obstacle."""
        raise NotImplementedError("This method should be implemented by subclasses.")

    @abstractmethod
    def get_points_penetration_offset(self, points: torch.Tensor, out: torch.Tensor | None = None) -> torch.Tensor:
        """Get the penetration offset for the given points.

        Args:
            points: Shape (N, 3). The points to check for penetration.
            out: Optional (N, 3) buffer. When set, a hit is written only if it is deeper than the
                offset already stored there.

        Returns:
            Shape (N, 3). The penetration offsets for the points.
        """
        raise NotImplementedError("This method should be implemented by subclasses.")

    def accumulate_selected_penetration(
        self,
        points: torch.Tensor,
        out: torch.Tensor,
        env_ids: torch.Tensor,
        num_bodies: int,
        num_points: int,
    ) -> None:
        """Merge this obstacle into ``out`` for the environments in ``env_ids``.

        ``points`` and ``out`` have shape ``(num_envs, num_bodies, num_points, 3)``. The default
        gathers the selected rows. Cylinder obstacles replace this with an in-place warp query.
        """
        if env_ids.numel() == 0:
            return
        selected_points = points[env_ids].reshape(-1, 3).contiguous()
        selected_out = out[env_ids].reshape(-1, 3).contiguous()
        self.get_points_penetration_offset(selected_points, out=selected_out)
        out[env_ids] = selected_out.view(-1, num_bodies, num_points, 3)
