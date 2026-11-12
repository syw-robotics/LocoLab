# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Reference-motion geometry from mjviser and body poses from MuJoCo."""

from functools import cached_property
from pathlib import Path

import mujoco
import numpy as np
import trimesh

from ._viser import SO3, group_geoms_by_visual_compat, merge_geoms, merge_geoms_hull
from .assets import load_mjcf_model


def visual_meshes(model: mujoco.MjModel, *, hull: bool = False):
    """Use the same body-local visual geometry for the robot and its ghost."""
    by_body = {}
    for geom_id, body_id in enumerate(model.geom_bodyid):
        if body_id == 0 or model.geom_contype[geom_id] or model.geom_conaffinity[geom_id]:
            continue
        by_body.setdefault(int(body_id), []).append(geom_id)
    meshes, links = [], []
    for body_id, geom_ids in by_body.items():
        # Match mjviser's per-body, texture-compatible visual mesh grouping.
        for group in group_geoms_by_visual_compat(model, geom_ids):
            if hull:
                parts = []
                for geom_id in group:
                    part = merge_geoms_hull(model, [geom_id])
                    # Preserve visual primitives and meshes without a 3D hull.
                    if part is None:
                        part = merge_geoms(model, [geom_id])
                    # Hull mode uses viewer colors. Reset upstream color caches
                    # that may still refer to vertices removed during hull creation.
                    part.visual = trimesh.visual.ColorVisuals(mesh=part)
                    parts.append(part)
                mesh = trimesh.util.concatenate(parts)
            else:
                mesh = merge_geoms(model, group)
            if len(mesh.faces):
                meshes.append(mesh)
                links.append(body_id - 1)
    return meshes, np.asarray(links, dtype=np.int64)


class MotionBodies:
    """Load body-local meshes once; evaluate reference poses without simulation."""

    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)
        model = mujoco.MjModel.from_xml_path(str(self._path))
        self.model = model
        if model.nbody < 2:
            raise ValueError(f"{path} has no body to draw")
        # Skip MuJoCo's world body. Mesh indices and returned poses share this order.
        self.link_names = [model.body(i).name for i in range(1, model.nbody)]
        joint_ids = np.flatnonzero(np.isin(model.jnt_type, (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)))
        self.joint_names = [model.joint(int(i)).name for i in joint_ids]
        self._qpos_ids = model.jnt_qposadr[joint_ids].copy()

        self.meshes, self.mesh_links = visual_meshes(model)

    @cached_property
    def hull_meshes(self):
        """Build browser-only hulls on explicit request; keep viewport meshes intact."""
        model = load_mjcf_model(self._path)
        meshes, _ = visual_meshes(model, hull=True)
        return meshes

    def poses(
        self,
        root_position: np.ndarray,
        root_wxyz: np.ndarray,
        configuration: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return world link positions (N, L, 3) and wxyz orientations (N, L, 4)."""
        count = len(root_position)
        positions = np.empty((count, len(self.link_names), 3))
        orientations = np.empty((count, len(self.link_names), 4))
        # Keep data local to this call so viewport and browser can share the model.
        data = mujoco.MjData(self.model)
        for env_id in range(count):
            data.qpos[self._qpos_ids] = configuration[env_id]
            # Native FK handles joint anchors, reference angles and multiple joints
            # per body. No physics step or second simulated articulation is needed.
            mujoco.mj_kinematics(self.model, data)

            # Anchor the model's first body to the clip root. This also supports
            # fixed-base MJCFs and models with a nonzero default root transform.
            root_rotation = SO3(root_wxyz[env_id]) @ SO3(data.xquat[1]).inverse()
            positions[env_id] = root_position[env_id] + root_rotation.apply(data.xpos[1:] - data.xpos[1])
            orientations[env_id] = (root_rotation @ SO3(data.xquat[1:])).wxyz
        return positions, orientations
