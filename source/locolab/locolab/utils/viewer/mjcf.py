# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Pose an MJCF the way MuJoCo compiles it, for the Viser play viewer.

Visual geoms are the meshes (``contype == 0``). Collision geoms are the
primitives MuJoCo actually simulates, tessellated in the link frame.
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
import trimesh

from .urdf import JointMotion, RobotBodies

_CAPSULE = int(mujoco.mjtGeom.mjGEOM_CAPSULE)
_SPHERE = int(mujoco.mjtGeom.mjGEOM_SPHERE)
_BOX = int(mujoco.mjtGeom.mjGEOM_BOX)
_CYLINDER = int(mujoco.mjtGeom.mjGEOM_CYLINDER)
_MESH = int(mujoco.mjtGeom.mjGEOM_MESH)
_HINGE = int(mujoco.mjtJoint.mjJNT_HINGE)
_SLIDE = int(mujoco.mjtJoint.mjJNT_SLIDE)


def load_mjcf(path: str | Path) -> tuple[RobotBodies, list[str]]:
    """Return link meshes and the actuated joint names, in tree order."""
    xml = Path(path).expanduser().resolve()
    if not xml.is_file():
        raise FileNotFoundError(f"Robot MJCF not found: {xml}")
    model = mujoco.MjModel.from_xml_path(str(xml))
    if model.nbody < 2:
        raise ValueError(f"{xml} has no body to draw")

    link_names = [model.body(i).name for i in range(1, model.nbody)]
    count = len(link_names)
    parent = np.full(count, -1, dtype=np.int64)
    joint = np.full(count, -1, dtype=np.int64)
    for link, body_id in enumerate(range(1, model.nbody)):
        parent_body = int(model.body_parentid[body_id])
        if parent_body > 0:
            parent[link] = parent_body - 1
        if link > 0:
            joint[link] = link - 1

    motion, actuated = _link_motion(model, count)
    visuals, visual_links = _geoms(model, collision=False)
    collisions, collision_links = _geoms(model, collision=True)
    bodies = RobotBodies.from_kinematics(
        motion,
        link_names,
        parent,
        joint,
        visuals,
        visual_links,
        collisions,
        collision_links,
    )
    return bodies, actuated


def _link_motion(model: mujoco.MjModel, link_count: int) -> tuple[JointMotion, list[str]]:
    """One parent-to-child joint per non-root body, at MuJoCo's q=0 pose."""
    slots = link_count - 1
    rest_wxyz = np.zeros((slots, 4))
    rest_wxyz[:, 0] = 1.0
    rest_position = np.zeros((slots, 3))
    revolute_ids: list[int] = []
    revolute_rotation: list[np.ndarray] = []
    revolute_axis: list[np.ndarray] = []
    revolute_source: list[int] = []
    prismatic_ids: list[int] = []
    prismatic_rotation: list[np.ndarray] = []
    prismatic_translation: list[np.ndarray] = []
    prismatic_axis: list[np.ndarray] = []
    prismatic_source: list[int] = []
    actuated: list[str] = []

    for slot, body_id in enumerate(range(2, model.nbody)):
        rotation = _quat_to_matrix(model.body_quat[body_id])
        rest_position[slot] = model.body_pos[body_id]
        rest_wxyz[slot] = _matrix_to_wxyz(rotation)
        address = int(model.body_jntadr[body_id])
        number = int(model.body_jntnum[body_id])
        if number > 1:
            raise ValueError(
                f"Body '{model.body(body_id).name}' has {number} joints; "
                "the viewer poses one joint per link."
            )
        if number == 0:
            continue
        kind = int(model.jnt_type[address])
        if kind not in (_HINGE, _SLIDE):
            continue
        name = model.joint(address).name
        if not name:
            continue
        source = len(actuated)
        actuated.append(name)
        axis = np.asarray(model.jnt_axis[address], dtype=np.float64)
        if kind == _HINGE:
            revolute_ids.append(slot)
            revolute_rotation.append(rotation)
            revolute_axis.append(_unit(axis))
            revolute_source.append(source)
        else:
            prismatic_ids.append(slot)
            prismatic_rotation.append(rotation)
            prismatic_translation.append(rest_position[slot].copy())
            prismatic_axis.append(axis)
            prismatic_source.append(source)

    motion = JointMotion.from_parts(
        rest_wxyz,
        rest_position,
        np.asarray(revolute_ids, dtype=np.int64),
        _stack(revolute_rotation, (3, 3)),
        _stack(revolute_axis, (3,)),
        np.asarray(revolute_source, dtype=np.int64),
        np.asarray(prismatic_ids, dtype=np.int64),
        _stack(prismatic_rotation, (3, 3)),
        _stack(prismatic_translation, (3,)),
        _stack(prismatic_axis, (3,)),
        np.asarray(prismatic_source, dtype=np.int64),
    )
    return motion, actuated


def _geoms(model: mujoco.MjModel, *, collision: bool) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    meshes: list[trimesh.Trimesh] = []
    links: list[int] = []
    for geom_id in range(model.ngeom):
        body_id = int(model.geom_bodyid[geom_id])
        if body_id <= 0:
            continue
        is_collision = int(model.geom_contype[geom_id]) != 0
        if is_collision != collision:
            continue
        mesh = _geom_mesh(model, geom_id)
        if mesh is None or len(mesh.faces) == 0:
            continue
        _apply_geom_pose(mesh, model.geom_pos[geom_id], model.geom_quat[geom_id])
        if not collision:
            _paint(mesh, model.geom_rgba[geom_id])
        meshes.append(mesh)
        links.append(body_id - 1)
    return meshes, np.asarray(links, dtype=np.int64)


def _geom_mesh(model: mujoco.MjModel, geom_id: int) -> trimesh.Trimesh | None:
    kind = int(model.geom_type[geom_id])
    size = np.asarray(model.geom_size[geom_id], dtype=np.float64)
    if kind == _SPHERE:
        return trimesh.creation.icosphere(subdivisions=2, radius=float(size[0]))
    if kind == _CAPSULE:
        return trimesh.creation.capsule(radius=float(size[0]), height=float(size[1]) * 2.0)
    if kind == _CYLINDER:
        return trimesh.creation.cylinder(radius=float(size[0]), height=float(size[1]) * 2.0)
    if kind == _BOX:
        return trimesh.creation.box(extents=size[:3] * 2.0)
    if kind == _MESH:
        return _compiled_mesh(model, geom_id)
    return None


def _compiled_mesh(model: mujoco.MjModel, geom_id: int) -> trimesh.Trimesh | None:
    """Vertices in the mesh inertia frame. Geom pose puts them on the body.

    ``mesh_vert`` is already scaled. ``mesh_scale`` records that factor, and
    ``geom_size`` is the fitted bounding box, so neither is a second scale.
    """
    mesh_id = int(model.geom_dataid[geom_id])
    if mesh_id < 0:
        return None
    vert_adr = int(model.mesh_vertadr[mesh_id])
    vert_num = int(model.mesh_vertnum[mesh_id])
    face_adr = int(model.mesh_faceadr[mesh_id])
    face_num = int(model.mesh_facenum[mesh_id])
    vertices = np.asarray(model.mesh_vert[vert_adr : vert_adr + vert_num], dtype=np.float64).copy()
    faces = np.asarray(model.mesh_face[face_adr : face_adr + face_num], dtype=np.int64).copy()
    return trimesh.Trimesh(vertices=vertices, faces=faces, process=False)


def _apply_geom_pose(mesh: trimesh.Trimesh, pos, quat_wxyz) -> None:
    transform = np.eye(4)
    transform[:3, :3] = _quat_to_matrix(quat_wxyz)
    transform[:3, 3] = np.asarray(pos, dtype=np.float64)
    if not np.allclose(transform, np.eye(4)):
        mesh.apply_transform(transform)


def _paint(mesh: trimesh.Trimesh, rgba) -> None:
    color = np.clip(np.asarray(rgba, dtype=np.float64) * 255.0, 0.0, 255.0).astype(np.uint8)
    mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh, face_colors=np.tile(color, (len(mesh.faces), 1)))


def _quat_to_matrix(quat_wxyz) -> np.ndarray:
    w, x, y, z = (float(value) for value in quat_wxyz)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def _matrix_to_wxyz(rotation: np.ndarray) -> np.ndarray:
    from viser.transforms import SO3

    return np.asarray(SO3.from_matrix(rotation).wxyz, dtype=np.float64)


def _unit(axis: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12:
        return np.array([0.0, 0.0, 1.0])
    return axis / norm


def _stack(rows: list[np.ndarray], shape: tuple[int, ...]) -> np.ndarray:
    if not rows:
        return np.zeros((0, *shape))
    return np.stack(rows)
