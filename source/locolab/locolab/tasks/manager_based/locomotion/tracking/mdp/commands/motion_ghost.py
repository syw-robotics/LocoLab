# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Visual-only reference robot shared by viewport and browser debug views."""

from __future__ import annotations

import torch

from locolab.utils.viewer.assets import robot_mjcf_path
from locolab.utils.viewer.mjcf import MotionBodies
from locolab.utils.viewer.utils import as_numpy, xyzw_to_wxyz


class MotionGhost:
    """Load link meshes once and pose them from the command's reference frames."""

    color = (0.2, 0.65, 1.0)  # light blue
    opacity = 0.3  # transparent

    def __init__(self, command):
        robot = command.robot
        self.bodies = MotionBodies(robot_mjcf_path(robot.cfg))
        robot_joint_index = {name: index for index, name in enumerate(robot.joint_names)}
        self.joint_ids = [robot_joint_index[name] for name in self.bodies.joint_names]
        self.max_envs = command.cfg.ghost_max_envs
        # Flat tracking clips and scene origins are static. Copy them once so each
        # render only transfers the selected frame indices from the simulation device.
        motion = command.motion
        self.root_pos = as_numpy(motion.root_pos, slice(None)).copy()
        self.root_wxyz = xyzw_to_wxyz(as_numpy(motion.root_quat, slice(None)))
        self.joint_pos = as_numpy(motion.joint_pos, slice(None)).copy()
        self.default_joint_pos = as_numpy(robot.data.default_joint_pos, slice(None)).copy()
        self.origins = as_numpy(command._env.scene.env_origins, slice(None)).copy()

    def poses(self, command, selector=slice(None)):
        """Return world link poses for the selected environments, in wxyz order."""
        frames = command.time_steps[selector][:self.max_envs].detach().cpu().numpy()
        position = self.root_pos[frames] + self.origins[selector][:self.max_envs]
        orientation = self.root_wxyz[frames]
        # Preserve default positions for any robot joints omitted by the motion clip.
        joints = self.default_joint_pos[selector][:self.max_envs].copy()
        joints[:, command._joint_ids] = self.joint_pos[frames]
        return self.bodies.poses(position, orientation, joints[:, self.joint_ids])

    def create_markers(self, command):
        """Create instanced mesh markers without registering another simulated robot."""
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkersCfg

        from .motion_ghost_markers import MotionGhostMarkers

        markers = {
            f"mesh_{index}": sim_utils.MeshFileCfg(
                mesh=sim_utils.MeshFileCfg.TriangleMeshCfg(vertices=mesh.vertices, faces=mesh.faces),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=self.color, opacity=self.opacity),
            )
            for index, mesh in enumerate(self.bodies.meshes)
        }
        self.markers = MotionGhostMarkers(
            VisualizationMarkersCfg(prim_path=f"/Visuals/MotionGhost/{command.cfg.asset_name}", markers=markers), self
        )
        count = min(command.num_envs, self.max_envs)
        self.marker_indices = torch.arange(len(markers), dtype=torch.int32, device=command.device).repeat(count)
        self.environment_ids = torch.arange(count, dtype=torch.int32, device=command.device).repeat_interleave(
            len(markers)
        )

    def update_markers(self, command):
        if not self.markers.is_visible():
            return
        positions, wxyz = self.poses(command)
        links = self.bodies.mesh_links
        self.markers.visualize(
            translations=positions[:, links].reshape(-1, 3),
            orientations=wxyz[:, links][:, :, [1, 2, 3, 0]].reshape(-1, 4),
            marker_indices=self.marker_indices,
            environment_ids=self.environment_ids,
        )
