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

import weakref
from collections.abc import Sequence
from typing import TYPE_CHECKING, Any

import isaaclab.sim as sim_utils
import isaaclab.utils.string as string_utils
import omni.kit.app
import omni.physics.tensors.impl.api as physx
import torch
import warp as wp
from isaaclab.markers import VisualizationMarkers
from isaaclab.sensors.sensor_base import SensorBase

from locolab.utils.warp.kernels import launch_warp_on_torch_stream, refresh_volume_points_kernel

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
        self._enabled_env_ids_i32: torch.Tensor | None = None
        self._all_env_ids_i32: torch.Tensor | None = None

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
        that are not on obstacle terrains skip the penetration query. Poses are still refreshed.
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
            self._enabled_env_ids_i32 = None
            return
        mask = enabled_env_mask.to(device=self.device, dtype=torch.bool)
        if mask.all():
            self._enabled_env_mask = None
            self._enabled_env_ids = None
            self._enabled_env_ids_i32 = None
            return
        self._enabled_env_mask = mask
        self._enabled_env_ids = torch.nonzero(mask, as_tuple=False).squeeze(-1)
        self._enabled_env_ids_i32 = self._enabled_env_ids.to(dtype=torch.int32)

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
        self._all_env_ids_i32 = torch.arange(self._num_envs, device=self.device, dtype=torch.int32)
        self._volume_points_pattern = self._volume_points_pattern.contiguous()

    def _update_buffers_impl(self, env_ids: Sequence[int]):
        """Fills the buffers of the sensor data."""
        # default to all sensors
        if len(env_ids) == self._num_envs:
            env_ids = slice(None)

        # Link poses are needed for every selected body. The obstacle mask only
        # skips the penetration query, otherwise those markers freeze at spawn.
        self._refresh_volume_points(env_ids)

        active_ids = self._active_env_ids(env_ids)
        if isinstance(active_ids, torch.Tensor) and active_ids.numel() == 0:
            return
        self._refresh_penetration_offset(active_ids)

    def _refresh_volume_points(self, env_ids: Sequence[int] | slice | torch.Tensor | None = None) -> None:
        """Refresh body pose and per-point world position/velocity for the given environments."""
        env_ids_i32 = self._env_ids_i32(env_ids)
        if env_ids_i32.numel() == 0 or self.num_bodies == 0:
            return

        # PhysX stores one row per body, env-major:
        # pose (x, y, z, qx, qy, qz, qw), velocity (vx, vy, vz, wx, wy, wz).
        body_poses = self.body_physx_view.get_transforms().reshape(-1, 7).contiguous()
        body_vels = self.body_physx_view.get_velocities().reshape(-1, 6).contiguous()
        data = self._data
        env_ids_wp = wp.from_torch(env_ids_i32, dtype=wp.int32)
        body_poses_wp = wp.from_torch(body_poses)
        body_vels_wp = wp.from_torch(body_vels)
        pattern_wp = wp.from_torch(self._volume_points_pattern, dtype=wp.vec3)
        pos_w_wp = wp.from_torch(data.pos_w.view(-1, 3), dtype=wp.vec3)
        quat_w_wp = wp.from_torch(data.quat_w.view(-1, 4), dtype=wp.vec4)
        vel_w_wp = wp.from_torch(data.vel_w.view(-1, 3), dtype=wp.vec3)
        ang_vel_w_wp = wp.from_torch(data.ang_vel_w.view(-1, 3), dtype=wp.vec3)
        points_pos_w_wp = wp.from_torch(data.points_pos_w.view(-1, 3), dtype=wp.vec3)
        points_vel_w_wp = wp.from_torch(data.points_vel_w.view(-1, 3), dtype=wp.vec3)
        launch_warp_on_torch_stream(
            refresh_volume_points_kernel,
            dim=int(env_ids_i32.shape[0]) * self.num_bodies,
            inputs=[
                env_ids_wp,
                self.num_bodies,
                body_poses_wp,
                body_vels_wp,
                pattern_wp,
                pos_w_wp,
                quat_w_wp,
                vel_w_wp,
                ang_vel_w_wp,
                points_pos_w_wp,
                points_vel_w_wp,
            ],
            device=self.device,
        )

    def _refresh_penetration_offset(self, env_ids: Sequence[int] | slice | torch.Tensor | None) -> None:
        """Refresh penetration offsets for the given environments."""
        env_ids_i32 = self._env_ids_i32(env_ids)
        if env_ids_i32.numel() == 0:
            return

        # A missed point is left unchanged, so owned rows must start at zero.
        # Each obstacle then keeps the deeper hit already stored there.
        self._clear_penetration(env_ids_i32)
        if not self._virtual_obstacles:
            return

        for virtual_obstacle in self._virtual_obstacles.values():
            virtual_obstacle.accumulate_selected_penetration(
                self._data.points_pos_w,
                self._data.penetration_offset,
                env_ids_i32,
                self.num_bodies,
                self._data.point_num_each_body,
            )

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

    def _env_ids_i32(self, env_ids: Sequence[int] | slice | torch.Tensor | None) -> torch.Tensor:
        """int32 environment indices on the sensor device. ``slice(None)`` means every environment."""
        if env_ids is None or (isinstance(env_ids, slice) and env_ids == slice(None)):
            return self._all_env_ids_i32
        # The enabled-env list is fixed at startup, so reuse its int32 copy.
        if isinstance(env_ids, torch.Tensor) and env_ids is self._enabled_env_ids:
            return self._enabled_env_ids_i32
        if not isinstance(env_ids, torch.Tensor):
            env_ids = torch.as_tensor(env_ids, device=self.device)
        return env_ids.to(device=self.device, dtype=torch.int32).reshape(-1)

    def _clear_penetration(self, env_ids_i32: torch.Tensor) -> None:
        if env_ids_i32 is self._all_env_ids_i32:
            self._data.penetration_offset.zero_()
            return
        self._data.penetration_offset[env_ids_i32] = 0

    def set_debug_vis(self, debug_vis: bool) -> bool:
        """Draw markers on the pre-update stream so they share a frame with the links.

        ``SensorBase`` subscribes to post-update, which runs after the viewport has
        already drawn. The next ``sim.render()`` is ``render_interval`` physics steps
        later, so the spheres trail the foot meshes. Pre-update runs after
        ``SimulationContext.forward()`` flushes the link poses and before that draw.
        """
        if not self.has_debug_vis_implementation:
            return False
        self._set_debug_vis_impl(debug_vis)
        self._is_visualizing = debug_vis
        if debug_vis:
            if self._debug_vis_handle is None:
                app_interface = omni.kit.app.get_app_interface()
                self._debug_vis_handle = app_interface.get_pre_update_event_stream().create_subscription_to_pop(
                    lambda event, obj=weakref.proxy(self): obj._debug_vis_callback(event)
                )
        elif self._debug_vis_handle is not None:
            self._debug_vis_handle.unsubscribe()
            self._debug_vis_handle = None
        return True

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
        if not self._is_initialized or getattr(self, "_body_physx_view", None) is None:
            return

        # ``sim.render()`` runs before ``scene.update()``, so the cached buffer is still
        # the previous physics pose. Sample the rigid bodies that were just stepped.
        if str(self.device).startswith("cuda"):
            torch.cuda.set_device(self.device)
        self._refresh_volume_points(slice(None))
        wp.synchronize()

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
