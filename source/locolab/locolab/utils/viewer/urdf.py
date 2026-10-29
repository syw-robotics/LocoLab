# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Load the spawned URDF and pose its links without copying the meshes.

:class:`RobotBodies` keeps visual and collision meshes in each link's frame.
The viewer instances those meshes, so a second environment costs a pose,
not another copy of the vertices.
"""

from __future__ import annotations

from functools import partial
from pathlib import Path

import numpy as np
import trimesh
import yourdfpy
from viser.transforms import SO3

from .utils import rotate_wxyz


def load_robot(asset_path: str | Path) -> tuple[RobotBodies, list[str]]:
    """Load the file the task actually spawns.

    ``.xml`` / ``.mjcf`` use MuJoCo geoms: visual meshes and the collision
    primitives (capsules, spheres, boxes). Anything else is a URDF.
    """
    path = Path(asset_path)
    if path.suffix.lower() in {".xml", ".mjcf"}:
        from .mjcf import load_mjcf

        return load_mjcf(path)
    urdf = load_urdf(path)
    return RobotBodies(urdf), list(urdf.actuated_joint_names)


def load_urdf(urdf_path: str | Path) -> yourdfpy.URDF:
    """Load a URDF and resolve ``package://`` or relative mesh paths."""
    path = Path(urdf_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Robot URDF not found: {path}")
    handler = partial(yourdfpy.filename_handler_magic, dir=str(path.parent))
    return yourdfpy.URDF.load(
        path,
        filename_handler=handler,
        build_scene_graph=True,
        load_meshes=True,
        build_collision_scene_graph=True,
        load_collision_meshes=True,
    )


def urdf_joint_ids(isaac_joint_names: list[str], urdf_joint_names: tuple[str, ...] | list[str]) -> np.ndarray:
    """Indices that reorder Isaac ``joint_pos`` into the URDF actuated order."""
    index = {name: i for i, name in enumerate(isaac_joint_names)}
    missing = [name for name in urdf_joint_names if name not in index]
    if missing:
        raise KeyError("URDF actuated joints are missing from the Isaac articulation: " + ", ".join(missing))
    return np.asarray([index[name] for name in urdf_joint_names], dtype=np.int64)


class JointMotion:
    """Parent-to-child pose of each URDF joint, without yourdfpy's scene graph."""

    def __init__(self, urdf: yourdfpy.URDF) -> None:
        joints = list(urdf.joint_map.values())
        actuated = {name: index for index, name in enumerate(urdf.actuated_joint_names)}
        count = len(joints)
        self.num_joints = count
        self.rest_wxyz = np.zeros((count, 4))
        self.rest_wxyz[:, 0] = 1.0
        self.rest_position = np.zeros((count, 3))
        revolute_ids: list[int] = []
        revolute_rotation: list[np.ndarray] = []
        revolute_axis: list[np.ndarray] = []
        revolute_source: list[int] = []
        revolute_scale: list[float] = []
        revolute_offset: list[float] = []
        prismatic_ids: list[int] = []
        prismatic_rotation: list[np.ndarray] = []
        prismatic_translation: list[np.ndarray] = []
        prismatic_axis: list[np.ndarray] = []
        prismatic_source: list[int] = []
        prismatic_scale: list[float] = []
        prismatic_offset: list[float] = []

        for index, joint in enumerate(joints):
            origin = _joint_origin(joint)
            axis = _joint_axis(joint)
            command = _joint_command(joint, actuated)
            if isinstance(command, float):
                angle = command
                moving = False
                source, scale, offset = 0, 1.0, 0.0
            else:
                source, scale, offset = command
                angle = offset
                moving = True
            rotation = origin[:3, :3]
            translation = origin[:3, 3].copy()
            if joint.type in ("revolute", "continuous"):
                unit = _unit_axis(axis)
                rotation = _revolute_rotation(rotation, unit, angle)
                if moving:
                    revolute_ids.append(index)
                    revolute_rotation.append(origin[:3, :3])
                    revolute_axis.append(unit)
                    revolute_source.append(source)
                    revolute_scale.append(scale)
                    revolute_offset.append(offset)
            elif joint.type == "prismatic":
                translation = translation + origin[:3, :3] @ (angle * axis)
                if moving:
                    prismatic_ids.append(index)
                    prismatic_rotation.append(origin[:3, :3])
                    prismatic_translation.append(origin[:3, 3].copy())
                    prismatic_axis.append(axis)
                    prismatic_source.append(source)
                    prismatic_scale.append(scale)
                    prismatic_offset.append(offset)
            self.rest_wxyz[index] = SO3.from_matrix(rotation).wxyz
            self.rest_position[index] = translation

        self.revolute_ids = np.asarray(revolute_ids, dtype=np.int64)
        self.prismatic_ids = np.asarray(prismatic_ids, dtype=np.int64)
        self._revolute_rotation = _stack_rows(revolute_rotation, (3, 3))
        self._revolute_axis = _stack_rows(revolute_axis, (3,))
        self._revolute_source = np.asarray(revolute_source, dtype=np.int64)
        self._revolute_scale = np.asarray(revolute_scale, dtype=np.float64)
        self._revolute_offset = np.asarray(revolute_offset, dtype=np.float64)
        self._prismatic_rotation = _stack_rows(prismatic_rotation, (3, 3))
        self._prismatic_translation = _stack_rows(prismatic_translation, (3,))
        self._prismatic_axis = _stack_rows(prismatic_axis, (3,))
        self._prismatic_source = np.asarray(prismatic_source, dtype=np.int64)
        self._prismatic_scale = np.asarray(prismatic_scale, dtype=np.float64)
        self._prismatic_offset = np.asarray(prismatic_offset, dtype=np.float64)

    @classmethod
    def from_parts(
        cls,
        rest_wxyz: np.ndarray,
        rest_position: np.ndarray,
        revolute_ids: np.ndarray,
        revolute_rotation: np.ndarray,
        revolute_axis: np.ndarray,
        revolute_source: np.ndarray,
        prismatic_ids: np.ndarray,
        prismatic_rotation: np.ndarray,
        prismatic_translation: np.ndarray,
        prismatic_axis: np.ndarray,
        prismatic_source: np.ndarray,
    ) -> JointMotion:
        """Build motion from arrays already in the parent-to-child convention."""
        self = cls.__new__(cls)
        self.num_joints = int(len(rest_wxyz))
        self.rest_wxyz = np.asarray(rest_wxyz, dtype=np.float64)
        self.rest_position = np.asarray(rest_position, dtype=np.float64)
        self.revolute_ids = np.asarray(revolute_ids, dtype=np.int64)
        self.prismatic_ids = np.asarray(prismatic_ids, dtype=np.int64)
        self._revolute_rotation = np.asarray(revolute_rotation, dtype=np.float64)
        self._revolute_axis = np.asarray(revolute_axis, dtype=np.float64)
        self._revolute_source = np.asarray(revolute_source, dtype=np.int64)
        self._revolute_scale = np.ones(len(self.revolute_ids), dtype=np.float64)
        self._revolute_offset = np.zeros(len(self.revolute_ids), dtype=np.float64)
        self._prismatic_rotation = np.asarray(prismatic_rotation, dtype=np.float64)
        self._prismatic_translation = np.asarray(prismatic_translation, dtype=np.float64)
        self._prismatic_axis = np.asarray(prismatic_axis, dtype=np.float64)
        self._prismatic_source = np.asarray(prismatic_source, dtype=np.int64)
        self._prismatic_scale = np.ones(len(self.prismatic_ids), dtype=np.float64)
        self._prismatic_offset = np.zeros(len(self.prismatic_ids), dtype=np.float64)
        return self

    def revolute_wxyz(self, configuration: np.ndarray) -> np.ndarray:
        """Orientations, shape ``(N, M, 4)``, for moving revolute joints."""
        angles = configuration[:, self._revolute_source] * self._revolute_scale + self._revolute_offset
        return _rotated_wxyz(self._revolute_rotation, self._revolute_axis, angles)

    def prismatic_positions(self, configuration: np.ndarray) -> np.ndarray:
        """Translations, shape ``(N, P, 3)``, for moving prismatic joints."""
        distance = configuration[:, self._prismatic_source] * self._prismatic_scale + self._prismatic_offset
        delta = distance[:, :, None] * self._prismatic_axis
        rotated = np.matmul(self._prismatic_rotation[None], delta[..., None])[..., 0]
        return self._prismatic_translation + rotated


def _joint_origin(joint) -> np.ndarray:
    origin = np.eye(4) if joint.origin is None else np.asarray(joint.origin, dtype=np.float64)
    if origin.shape != (4, 4):
        return np.eye(4)
    return origin


def _joint_axis(joint) -> np.ndarray:
    if joint.axis is None:
        return np.zeros(3)
    vector = np.asarray(joint.axis, dtype=np.float64).reshape(-1)
    axis = np.zeros(3)
    axis[: min(3, vector.size)] = vector[:3]
    return axis


def _unit_axis(axis: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12:
        return np.array([0.0, 0.0, 1.0])
    return axis / norm


def _joint_command(joint, actuated: dict[str, int]) -> tuple[int, float, float] | float:
    """Return ``(cfg index, scale, offset)`` or a constant angle in radians or meters."""
    if joint.mimic is not None:
        scale = 1.0 if joint.mimic.multiplier is None else float(joint.mimic.multiplier)
        offset = 0.0 if joint.mimic.offset is None else float(joint.mimic.offset)
        source = actuated.get(joint.mimic.joint)
        if source is None or scale == 0.0:
            return offset
        return source, scale, offset
    if joint.type in ("revolute", "continuous", "prismatic") and joint.name in actuated:
        return actuated[joint.name], 1.0, 0.0
    return 0.0


def _revolute_rotation(origin_rotation: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    """Match ``origin @ rotation_matrix(angle, axis)`` from yourdfpy."""
    cosine = float(np.cos(angle))
    sine = float(np.sin(angle))
    outer = np.outer(axis, axis)
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return origin_rotation @ (cosine * np.eye(3) + (1.0 - cosine) * outer + sine * skew)


def _rotated_wxyz(origin_rotation: np.ndarray, axis: np.ndarray, angles: np.ndarray) -> np.ndarray:
    """Batched form of :func:`_revolute_rotation`. ``angles`` has shape ``(N, M)``."""
    cosine = np.cos(angles)[:, :, None, None]
    sine = np.sin(angles)[:, :, None, None]
    outer = axis[:, :, None] * axis[:, None, :]
    x, y, z = axis.T
    skew = np.zeros((len(axis), 3, 3))
    skew[:, 0, 1] = -z
    skew[:, 0, 2] = y
    skew[:, 1, 0] = z
    skew[:, 1, 2] = -x
    skew[:, 2, 0] = -y
    skew[:, 2, 1] = x
    rotation = cosine * np.eye(3) + (1.0 - cosine) * outer + sine * skew
    return SO3.from_matrix(np.matmul(origin_rotation[None], rotation)).wxyz


def _stack_rows(rows: list[np.ndarray], shape: tuple[int, ...]) -> np.ndarray:
    if not rows:
        return np.zeros((0, *shape))
    return np.stack(rows)


class RobotBodies:
    """Visual and collision meshes, plus the world pose of every link.

    Meshes are baked into the link frame, including a fixed geometry origin.
    :meth:`poses` composes the root pose with each joint's local pose.
    """

    def __init__(self, urdf: yourdfpy.URDF) -> None:
        self.motion = JointMotion(urdf)
        joints = list(urdf.joint_map.values())
        parent_name = {joint.child: joint.parent for joint in joints}
        joint_index = {joint.child: index for index, joint in enumerate(joints)}
        children: dict[str, list[str]] = {}
        for child, parent in parent_name.items():
            children.setdefault(parent, []).append(child)
        order = [urdf.base_link]
        cursor = 0
        while cursor < len(order):
            order.extend(children.get(order[cursor], ()))
            cursor += 1
        self.link_names = order
        index = {name: link for link, name in enumerate(order)}
        count = len(order)
        self.parent = np.full(count, -1, dtype=np.int64)
        self.joint = np.full(count, -1, dtype=np.int64)
        for name, link in index.items():
            if name not in parent_name or parent_name[name] not in index:
                continue
            self.parent[link] = index[parent_name[name]]
            self.joint[link] = joint_index[name]

        self.meshes, self.mesh_links = _meshes_in_links(urdf.scene, index)
        self.collision_meshes, self.collision_mesh_links = _meshes_in_links(urdf.collision_scene, index)

    @classmethod
    def from_kinematics(
        cls,
        motion: JointMotion,
        link_names: list[str],
        parent: np.ndarray,
        joint: np.ndarray,
        meshes: list[trimesh.Trimesh],
        mesh_links: np.ndarray,
        collision_meshes: list[trimesh.Trimesh],
        collision_mesh_links: np.ndarray,
    ) -> RobotBodies:
        """Assemble a body graph that :meth:`poses` can drive without a URDF."""
        self = cls.__new__(cls)
        self.motion = motion
        self.link_names = link_names
        self.parent = parent
        self.joint = joint
        self.meshes = meshes
        self.mesh_links = mesh_links
        self.collision_meshes = collision_meshes
        self.collision_mesh_links = collision_mesh_links
        return self

    def poses(
        self,
        root_position: np.ndarray,
        root_wxyz: np.ndarray,
        configuration: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """World position ``(N, L, 3)`` and wxyz ``(N, L, 4)`` of each link."""
        position = np.asarray(root_position, dtype=np.float64)
        orientation = np.asarray(root_wxyz, dtype=np.float64)
        configuration = np.asarray(configuration, dtype=np.float64)
        count = position.shape[0]
        joints = self.motion.num_joints
        local_wxyz = np.broadcast_to(self.motion.rest_wxyz, (count, joints, 4)).copy()
        local_position = np.broadcast_to(self.motion.rest_position, (count, joints, 3)).copy()
        if len(self.motion.revolute_ids):
            local_wxyz[:, self.motion.revolute_ids] = self.motion.revolute_wxyz(configuration)
        if len(self.motion.prismatic_ids):
            local_position[:, self.motion.prismatic_ids] = self.motion.prismatic_positions(configuration)

        world_wxyz = np.zeros((count, len(self.link_names), 4))
        world_position = np.zeros((count, len(self.link_names), 3))
        world_wxyz[:, 0] = orientation
        world_position[:, 0] = position
        for link in range(1, len(self.link_names)):
            parent = int(self.parent[link])
            joint = int(self.joint[link])
            if parent < 0 or joint < 0:
                world_wxyz[:, link, 0] = 1.0
                continue
            parent_wxyz = world_wxyz[:, parent]
            world_wxyz[:, link] = _quat_mul(parent_wxyz, local_wxyz[:, joint])
            world_position[:, link] = world_position[:, parent] + rotate_wxyz(parent_wxyz, local_position[:, joint])
        return world_position, world_wxyz


def _meshes_in_links(scene, link_index: dict[str, int]) -> tuple[list[trimesh.Trimesh], np.ndarray]:
    meshes: list[trimesh.Trimesh] = []
    mesh_links: list[int] = []
    if scene is not None:
        for geom_name, mesh in scene.geometry.items():
            baked = _mesh_in_link(scene, link_index, geom_name, mesh)
            if baked is None:
                continue
            link, posed = baked
            meshes.append(posed)
            mesh_links.append(link)
    return meshes, np.asarray(mesh_links, dtype=np.int64)


def _mesh_in_link(scene, link_index: dict[str, int], geom_name: str, mesh: trimesh.Trimesh):
    """Bake a geometry into its link frame. Return ``(link, mesh)`` or None."""
    if len(mesh.faces) == 0:
        return None
    transforms = scene.graph.transforms
    name = geom_name
    chain: list[tuple[str, str]] = []
    seen: set[str] = set()
    while name not in link_index:
        if name in seen:
            return None
        seen.add(name)
        parent = transforms.parents.get(name)
        if parent is None:
            return None
        chain.append((parent, name))
        name = parent
    transform = np.eye(4)
    for parent, child in chain:
        edge = transforms.edge_data.get((parent, child))
        if edge is None:
            return None
        transform = np.asarray(edge["matrix"], dtype=np.float64) @ transform
    posed = mesh.copy()
    if not np.allclose(transform, np.eye(4)):
        posed.apply_transform(transform)
    return link_index[name], posed


def _quat_mul(parent: np.ndarray, local: np.ndarray) -> np.ndarray:
    """Hamilton product. ``R(parent @ local)`` matches :class:`viser.transforms.SO3`."""
    aw, ax, ay, az = parent[..., 0], parent[..., 1], parent[..., 2], parent[..., 3]
    bw, bx, by, bz = local[..., 0], local[..., 1], local[..., 2], local[..., 3]
    return np.stack(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ),
        axis=-1,
    )

