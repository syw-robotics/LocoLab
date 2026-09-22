# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""
Reference: https://github.com/project-instinct/instinct_rl.git
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING, Any

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
import isaaclab.utils.string as string_utils
import omni.physics.tensors.impl.api as physx
import torch
from isaaclab.markers import VisualizationMarkers
from isaaclab.sensors.sensor_base import SensorBase

from .volume_points_data import VolumePointsData

if TYPE_CHECKING:
    from .volume_points_cfg import VolumePointsCfg


class VolumePoints(SensorBase):
    """Volume Points sensor for detecting volume points in a simulation."""

    def __init__(self, cfg: VolumePointsCfg):
        super().__init__(cfg)

        # Initialize the volume points
        self._volume_points = None
        self._virtual_obstacles: dict = dict()
        self._enabled_env_mask: torch.Tensor | None = None
        self._enabled_env_ids: torch.Tensor | None = None
        self._pen_offset_tmp: torch.Tensor | None = None
        self._pen_depth_tmp: torch.Tensor | None = None

    """
    Properties
    """

    @property
    def data(self) -> VolumePointsData:
        # update sensors if needed
        self._update_outdated_buffers()
        # return the data
        return self._data

    @property
    def num_bodies(self) -> int:
        """Number of bodies with volume points sensors attached."""
        return self._num_bodies

    @property
    def body_names(self) -> list[str]:
        """Ordered names of bodies with volume points sensors attached."""
        prim_paths = self.body_physx_view.prim_paths[: self.num_bodies]
        return [path.split("/")[-1] for path in prim_paths]

    @property
    def body_physx_view(self) -> physx.RigidBodyView:
        """View for the rigid bodies captured (PhysX).

        Note:
            Use this view with caution. It requires handling of tensors in a specific way.
        """
        return self._body_physx_view

    """
    Operations
    """

    def register_virtual_obstacles(
        self,
        virtual_obstacles: dict[str, Any],
        enabled_env_mask: torch.Tensor | None = None,
    ) -> None:
        """Record virtual obstacles used for penetration queries.

        Typically called from a startup event. Pass ``enabled_env_mask`` so environments
        that are not on obstacle terrains skip the warp query during rollout.
        """
        self._virtual_obstacles.update(virtual_obstacles)
        self.set_enabled_env_mask(enabled_env_mask)

    def set_enabled_env_mask(self, enabled_env_mask: torch.Tensor | None) -> None:
        """Restrict penetration queries to a subset of environments.

        Terrain types are fixed at env construction, so the enabled index list is cached once.
        """
        if enabled_env_mask is None:
            self._enabled_env_mask = None
            self._enabled_env_ids = None
            return
        mask = enabled_env_mask.to(device=self.device, dtype=torch.bool)
        if mask.all():
            self._enabled_env_mask = None
            self._enabled_env_ids = None
            return
        self._enabled_env_mask = mask
        self._enabled_env_ids = torch.nonzero(mask, as_tuple=False).squeeze(-1)

    def reset(self, env_ids: Sequence[int] | None = None):
        # reset the timers and counters
        super().reset(env_ids)
        ...

    def find_bodies(self, name_keys: str | Sequence[str], preserve_order: bool = False) -> tuple[list[int], list[str]]:
        """Find bodies in the articulation based on the name keys.

        Args:
            name_keys: A regular expression or a list of regular expressions to match the body names.
            preserve_order: Whether to preserve the order of the name keys in the output. Defaults to False.

        Returns:
            A tuple of lists containing the body indices and names.
        """
        return string_utils.resolve_matching_names(name_keys, self.body_names, preserve_order)

    """
    Implementation
    """

    def _initialize_impl(self):
        super()._initialize_impl()
        # create simulation view
        self._physics_sim_view = physx.create_simulation_view(self._backend)
        self._physics_sim_view.set_subspace_roots("/")
        # check that only rigid bodies are selected
        leaf_pattern = self.cfg.prim_path.rsplit("/", 1)[-1]
        template_prim_path = self._parent_prims[0].GetPath().pathString
        body_names = list()
        for prim in sim_utils.find_matching_prims(template_prim_path + "/" + leaf_pattern):
            prim_path = prim.GetPath().pathString
            body_names.append(prim_path.rsplit("/", 1)[-1])
        if not body_names:
            raise RuntimeError(f"Sensor at path '{self.cfg.prim_path}' could not find any bodies.")

        # construct regex expression for the body names
        body_names_regex = r"(" + "|".join(body_names) + r")"
        body_names_regex = f"{self.cfg.prim_path.rsplit('/', 1)[0]}/{body_names_regex}"
        # convert regex expressions to glob expressions for PhysX
        body_names_glob = body_names_regex.replace(".*", "*")

        # create a rigid prim view for the sensor
        self._body_physx_view = self._physics_sim_view.create_rigid_body_view(body_names_glob)

        # resolve the true count of bodies
        self._num_bodies = self.body_physx_view.count // self._num_envs
        # check that volume points sensor succeeded
        if self._num_bodies != len(body_names):
            raise RuntimeError(
                "Failed to initialize volume points sensor for specified bodies."
                f"\n\tInput prim path    : {self.cfg.prim_path}"
                f"\n\tResolved prim paths: {body_names_regex}"
            )

        # initialize the volume points data
        self._volume_points_pattern: torch.Tensor = self.cfg.points_generator.func(self.cfg.points_generator).to(
            self.device
        )  # (P, 3)
        self._data: VolumePointsData = VolumePointsData.make_zero(
            num_envs=self._num_envs,
            num_bodies=self._num_bodies,
            point_num_each_body=self._volume_points_pattern.shape[0],
            device=self.device,
        )
        self._pen_offset_tmp = None
        self._pen_depth_tmp = None

    def _update_buffers_impl(self, env_ids: Sequence[int]):
        """Fills the buffers of the sensor data."""
        # default to all sensors
        if len(env_ids) == self._num_envs:
            env_ids = slice(None)

        active_ids = self._active_env_ids(env_ids)
        if isinstance(active_ids, torch.Tensor) and active_ids.numel() == 0:
            return

        self._refresh_volume_points(active_ids)
        self._refresh_penetration_offset(active_ids)

    def _refresh_volume_points(self, env_ids: Sequence[int] | None = None) -> None:
        """Refresh the volume points data. If env_ids is None, refresh all environments."""

        body_poses = self.body_physx_view.get_transforms().view(-1, self.num_bodies, 7)[env_ids]  # (N_, B, 7)
        body_vels = self.body_physx_view.get_velocities().view(-1, self.num_bodies, 6)[env_ids]  # (N_, B, 6)
        self._data.pos_w[env_ids] = body_poses[..., :3]  # (N_, B, 3)
        # convert quaternion from xyz to wxyz format
        self._data.quat_w[env_ids] = math_utils.convert_quat(body_poses[..., 3:], to="wxyz")  # (N_, B, 4)
        self._data.vel_w[env_ids] = body_vels[..., :3]  # (N_, B, 3)
        self._data.ang_vel_w[env_ids] = body_vels[..., 3:]  # (N_, B, 3)

        # calculate the volume points positions and velocities in world frame
        N_B = self._data.pos_w[env_ids].shape[0] * self._data.pos_w[env_ids].shape[1]  # (N_*B)
        points_pos_w = math_utils.transform_points(
            self._volume_points_pattern.unsqueeze(0).expand(N_B, -1, -1),  # (N_*B, P, 3)
            self._data.pos_w[env_ids].flatten(0, 1),  # (N_*B, 3)
            self._data.quat_w[env_ids].flatten(0, 1),  # (N_*B, 4)
        ).reshape(
            *self._data.pos_w[env_ids].shape[:2], self._data.point_num_each_body, 3
        )  # (N_, B, P, 3)
        self._data.points_pos_w[env_ids] = points_pos_w
        points_vel_w = self._data.points_vel_w[env_ids]
        points_vel_w.copy_(self._data.vel_w[env_ids].unsqueeze(-2).expand_as(points_vel_w))
        points_vel_w.add_(
            torch.linalg.cross(
                self._data.ang_vel_w[env_ids].unsqueeze(-2),
                points_pos_w - self._data.pos_w[env_ids].unsqueeze(-2),
                dim=-1,
            )
        )

    def _refresh_penetration_offset(self, env_ids: Sequence[int]) -> None:
        """Refresh the penetration depth data for the given environments."""

        penetration_offset_buf: torch.Tensor = self._data.penetration_offset[env_ids]
        penetration_offset_buf.zero_()
        if not self._virtual_obstacles:
            return

        query_points = self._data.points_pos_w[env_ids]
        query_shape = query_points.shape
        query_flat = query_points.reshape(-1, 3)
        best_offset, best_depth = self._penetration_scratch(query_shape, query_flat.device, query_flat.dtype)

        for virtual_obstacle in self._virtual_obstacles.values():
            penetration_offset = virtual_obstacle.get_points_penetration_offset(query_flat).view(query_shape)
            penetration_depth = torch.norm(penetration_offset, dim=-1)
            mask = penetration_depth > best_depth
            best_depth[mask] = penetration_depth[mask]
            best_offset[mask] = penetration_offset[mask]

        penetration_offset_buf.copy_(best_offset)

    def _active_env_ids(self, env_ids: Sequence[int] | slice) -> Sequence[int] | slice | torch.Tensor:
        if self._enabled_env_ids is None:
            return env_ids
        if isinstance(env_ids, slice) and env_ids == slice(None):
            return self._enabled_env_ids
        if isinstance(env_ids, torch.Tensor):
            env_ids_t = env_ids
        else:
            env_ids_t = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        return env_ids_t[self._enabled_env_mask[env_ids_t]]

    def _penetration_scratch(
        self, shape: torch.Size, device: torch.device, dtype: torch.dtype
    ) -> tuple[torch.Tensor, torch.Tensor]:
        if (
            self._pen_offset_tmp is None
            or self._pen_offset_tmp.shape != shape
            or self._pen_offset_tmp.device != device
            or self._pen_offset_tmp.dtype != dtype
        ):
            self._pen_offset_tmp = torch.zeros(shape, device=device, dtype=dtype)
            self._pen_depth_tmp = torch.zeros(shape[:-1], device=device, dtype=dtype)
        else:
            self._pen_offset_tmp.zero_()
            self._pen_depth_tmp.zero_()
        return self._pen_offset_tmp, self._pen_depth_tmp

    def _set_debug_vis_impl(self, debug_vis: bool):
        # set visibility of markers
        # note: parent only deals with callbacks. not their visibility
        if debug_vis:
            # create markers if necessary for the first tome
            if not hasattr(self, "points_visualizer"):
                self.points_visualizer = VisualizationMarkers(self.cfg.visualizer_cfg)
            # set their visibility to true
            self.points_visualizer.set_visibility(True)
        else:
            if hasattr(self, "points_visualizer"):
                self.points_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        # safely return if view becomes invalid
        # note: this invalidity happens because of isaac sim view callbacks
        if self.body_physx_view is None:
            return

        points = self._data.points_pos_w.view(-1, 3)  # (N_*B*P, 3)
        penetrated = torch.norm(self._data.penetration_offset.view(-1, 3), dim=-1) > 0.0  # (N_*B*P,)

        # add penetrated points if none
        if not torch.any(penetrated):
            points = torch.cat([points, torch.zeros_like(points[:1])], dim=0)
            penetrated = torch.cat([penetrated, torch.tensor([True], device=self.device)], dim=0)

        self.points_visualizer.visualize(
            translations=points,
            marker_indices=penetrated.long(),
        )

    """
    Internal simulation callbacks.
    """

    def _invalidate_initialize_callback(self, event):
        """Invalidates the scene elements."""
        # call parent
        super()._invalidate_initialize_callback(event)
        # set all existing views to None to invalidate them
        if hasattr(self, "points_visualizer"):
            delattr(self, "points_visualizer")
        self._physics_sim_view = None
        self._body_physx_view = None
