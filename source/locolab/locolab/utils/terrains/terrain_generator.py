# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.


from __future__ import annotations

import logging
import os
from typing import TYPE_CHECKING

import numpy as np
import torch
import trimesh

from isaaclab.terrains.trimesh.utils import make_border
from isaaclab.terrains.utils import color_meshes_by_height, find_flat_patches
from isaaclab.utils.dict import dict_to_md5_hash
from isaaclab.utils.io import dump_yaml
from isaaclab.utils.timer import Timer
from isaaclab.utils.warp import convert_to_warp_mesh

if TYPE_CHECKING:
    from isaaclab.terrains.sub_terrain_cfg import (
        FlatPatchSamplingCfg,
        SubTerrainBaseCfg,
    )

    from .terrain_generator_cfg import TerrainGeneratorCfg

# import logger
logger = logging.getLogger(__name__)


def _make_terrain_seam(
    mesh_a: trimesh.Trimesh, mesh_b: trimesh.Trimesh, axis: int, position: float
) -> trimesh.Trimesh | None:
    """Join the exposed heights of two tile edges without closing the space above them."""
    def boundary_edges(mesh):
        triangles = mesh.triangles[mesh.face_normals[:, 2] > 1e-6]
        on_edge = np.isclose(triangles[:, :, axis], position, atol=1e-6, rtol=0.0)
        selected = on_edge.sum(axis=1) == 2
        edges = triangles[selected][on_edge[selected]].reshape(-1, 2, 3)[:, :, [1 - axis, 2]]
        edges = edges[np.abs(edges[:, 1, 0] - edges[:, 0, 0]) > 1e-6]
        return edges

    def heights(edges, points):
        midpoints = 0.5 * (points[:-1] + points[1:])
        start, end = edges[:, 0], edges[:, 1]
        slopes = (end[:, 1] - start[:, 1]) / (end[:, 0] - start[:, 0])
        covered = (midpoints[:, None] >= np.minimum(start[:, 0], end[:, 0])) & (
            midpoints[:, None] <= np.maximum(start[:, 0], end[:, 0])
        )
        z = start[:, 1] + (midpoints[:, None] - start[:, 0]) * slopes
        highest = np.where(covered, z, -np.inf).argmax(axis=1)
        result = start[highest, 1, None] + (
            np.column_stack((points[:-1], points[1:])) - start[highest, 0, None]
        ) * slopes[highest, None]
        result[~covered.any(axis=1)] = np.nan
        return result

    edges_a, edges_b = boundary_edges(mesh_a), boundary_edges(mesh_b)
    if len(edges_a) == 0 or len(edges_b) == 0:
        return None
    points = np.unique(np.concatenate((edges_a[:, :, 0].ravel(), edges_b[:, :, 0].ravel())))
    za, zb = heights(edges_a, points), heights(edges_b, points)
    difference = za - zb
    crossing = difference[:, 0] * difference[:, 1] < 0.0
    if crossing.any():
        fraction = difference[crossing, 0] / (difference[crossing, 0] - difference[crossing, 1])
        points = np.unique(np.r_[points, points[:-1][crossing] + np.diff(points)[crossing] * fraction])
        za, zb = heights(edges_a, points), heights(edges_b, points)
    selected = np.isfinite(za).all(axis=1) & np.isfinite(zb).all(axis=1)
    selected &= (np.abs(za - zb) > 1e-6).any(axis=1) & (np.diff(points) > 1e-6)
    if not selected.any():
        return None
    spans = np.column_stack((points[:-1], points[1:]))[selected]
    za, zb = za[selected], zb[selected]
    vertices = np.zeros((len(spans), 4, 3))
    vertices[:, :, axis] = position
    vertices[:, :, 1 - axis] = spans[:, [0, 1, 1, 0]]
    vertices[:, :, 2] = np.column_stack((za[:, 0], za[:, 1], zb[:, 1], zb[:, 0]))
    faces = (np.array([[0, 1, 2], [0, 2, 3]]) + 4 * np.arange(len(spans))[:, None, None]).reshape(-1, 3)
    seam = trimesh.Trimesh(vertices=vertices.reshape(-1, 3), faces=faces, process=False)
    seam.update_faces(seam.area_faces > 1e-12)
    seam.faces = np.vstack((seam.faces, seam.faces[:, ::-1]))
    return seam


class TerrainGenerator:
    r"""Terrain generator to handle different terrain generation functions.

    The terrains are represented as meshes. These are obtained either from height fields or by using the
    `trimesh <https://trimsh.org/trimesh.html>`__ library. The height field representation is more
    flexible, but it is less computationally and memory efficient than the trimesh representation.

    All terrain generation functions take in the argument :obj:`difficulty` which determines the complexity
    of the terrain. The difficulty is a number between 0 and 1, where 0 is the easiest and 1 is the hardest.
    In most cases, the difficulty is used for linear interpolation between different terrain parameters.
    For example, in a pyramid stairs terrain the step height is interpolated between the specified minimum
    and maximum step height.

    Each sub-terrain has a corresponding configuration class that can be used to specify the parameters
    of the terrain. The configuration classes are inherited from the :class:`SubTerrainBaseCfg` class
    which contains the common parameters for all terrains.

    If a curriculum is used, the terrains are generated based on their difficulty parameter.
    The difficulty is varied linearly over the number of rows (i.e. along x) with a small random value
    added to the difficulty to ensure that the columns with the same sub-terrain type are not exactly
    the same. The difficulty parameter for a sub-terrain at a given row is calculated as:

    .. math::

        \text{difficulty} = \frac{\text{row_id} + \eta}{\text{num_rows}} \times (\text{upper} - \text{lower}) + \text{lower}

    where :math:`\eta\sim\mathcal{U}(0, 1)` is a random perturbation to the difficulty, and
    :math:`(\text{lower}, \text{upper})` is the range of the difficulty parameter, specified using the
    :attr:`~TerrainGeneratorCfg.difficulty_range` parameter.

    If a curriculum is not used, the terrains are generated randomly. In this case, the difficulty parameter
    is randomly sampled from the specified range, given by the :attr:`~TerrainGeneratorCfg.difficulty_range` parameter:

    .. math::

        \text{difficulty} \sim \mathcal{U}(\text{lower}, \text{upper})

    If the :attr:`~TerrainGeneratorCfg.flat_patch_sampling` is specified for a sub-terrain, flat patches are sampled
    on the terrain. These can be used for spawning robots, targets, etc. The sampled patches are stored
    in the :obj:`flat_patches` dictionary. The key specifies the intention of the flat patches and the
    value is a tensor containing the flat patches for each sub-terrain.

    If the flag :attr:`~TerrainGeneratorCfg.use_cache` is set to True, the terrains are cached based on their
    sub-terrain configurations. This means that if the same sub-terrain configuration is used
    multiple times, the terrain is only generated once and then reused. This is useful when
    generating complex sub-terrains that take a long time to generate.

    .. attention::

        The terrain generation has its own seed parameter. This is set using the :attr:`TerrainGeneratorCfg.seed`
        parameter. If the seed is not set and the caching is disabled, the terrain generation may not be
        completely reproducible.

    """

    terrain_mesh: trimesh.Trimesh
    """A single trimesh.Trimesh object for all the generated sub-terrains."""
    terrain_meshes: list[trimesh.Trimesh]
    """List of trimesh.Trimesh objects for all the generated sub-terrains."""
    terrain_mesh_origins: list[np.ndarray]
    """List of sub-terrain origins in the same frame as ``terrain_meshes``."""
    terrain_mesh_names: list[str]
    """List of sub-terrain names matching ``terrain_meshes`` and ``terrain_mesh_origins``."""
    terrain_origins: np.ndarray
    """The origin of each sub-terrain. Shape is (num_rows, num_cols, 3)."""
    terrain_indices: np.ndarray
    """The index of each sub-terrain. Shape is (num_rows, num_cols) with the index being an integer."""
    flat_patches: dict[str, torch.Tensor]
    """A dictionary of sampled valid (flat) patches for each sub-terrain.

    The dictionary keys are the names of the flat patch sampling configurations. This maps to a
    tensor containing the flat patches for each sub-terrain. The shape of the tensor is
    (num_rows, num_cols, num_patches, 3).

    For instance, the key "root_spawn" maps to a tensor containing the flat patches for spawning an asset.
    Similarly, the key "target_spawn" maps to a tensor containing the flat patches for setting targets.
    """

    def __init__(self, cfg: TerrainGeneratorCfg, device: str = "cpu"):
        """Initialize the terrain generator.

        Args:
            cfg: Configuration for the terrain generator.
            device: The device to use for the flat patches tensor.
        """
        # check inputs
        if len(cfg.sub_terrains) == 0:
            raise ValueError("No sub-terrains specified! Please add at least one sub-terrain.")
        # store inputs
        self.cfg = cfg
        self.device = device

        # set common values to all sub-terrains config
        from isaaclab.terrains.height_field import (
            HfTerrainBaseCfg,  # prevent circular import
        )
        from locolab.utils.terrains.locolab_terrains.locolab_mesh_terrains_cfg import MeshRoughTerrainCfg

        for sub_cfg in self.cfg.sub_terrains.values():
            # size of all terrains
            sub_cfg.size = self.cfg.size
            # params for height field terrains
            if isinstance(sub_cfg, HfTerrainBaseCfg):
                sub_cfg.horizontal_scale = self.cfg.horizontal_scale
                sub_cfg.vertical_scale = self.cfg.vertical_scale
                sub_cfg.slope_threshold = self.cfg.slope_threshold
            elif isinstance(sub_cfg, MeshRoughTerrainCfg):
                # Same roughness sampling scales as HF; stair/box XY stays metric.
                sub_cfg.horizontal_scale = self.cfg.horizontal_scale
                sub_cfg.vertical_scale = self.cfg.vertical_scale

        # throw a warning if the cache is enabled but the seed is not set
        if self.cfg.use_cache and self.cfg.seed is None:
            logger.warning(
                "Cache is enabled but the seed is not set. The terrain generation will not be reproducible."
                " Please set the seed in the terrain generator configuration to make the generation reproducible."
            )

        # if the seed is not set, we assume there is a global seed set and use that.
        # this ensures that the terrain is reproducible if the seed is set at the beginning of the program.
        if self.cfg.seed is not None:
            seed = self.cfg.seed
        else:
            seed = np.random.get_state()[1][0]
        # set the seed for reproducibility
        # note: we create a new random number generator to avoid affecting the global state
        #  in the other places where random numbers are used.
        self.np_rng = np.random.default_rng(seed)

        # buffer for storing valid patches
        self.flat_patches = {}
        # create a list of all sub-terrains
        self.terrain_meshes = list()
        self._terrain_meshes_by_cell = {}
        self.terrain_mesh_origins = list()
        self.terrain_mesh_names = list()  # sub-terrain name for each mesh
        self.terrain_origins = np.zeros((self.cfg.num_rows, self.cfg.num_cols, 3))
        self.terrain_indices = np.zeros((self.cfg.num_rows, self.cfg.num_cols), dtype=np.int32)

        # parse configuration and add sub-terrains
        # create terrains based on curriculum or randomly
        if self.cfg.curriculum:
            with Timer("[INFO] Generating terrains based on curriculum took"):
                self._generate_curriculum_terrains()
        else:
            with Timer("[INFO] Generating terrains randomly took"):
                self._generate_random_terrains()
        # add a border around the terrains
        self._add_terrain_border()
        # combine all the sub-terrains into a single mesh
        for mesh in self.terrain_meshes:
            mesh.face_attributes.setdefault("exclude_virtual_obstacle", np.zeros(len(mesh.faces), dtype=bool))
        self.terrain_mesh = trimesh.util.concatenate(self.terrain_meshes)

        # color the terrain mesh
        if self.cfg.color_scheme == "height":
            self.terrain_mesh = color_meshes_by_height(self.terrain_mesh)
        elif self.cfg.color_scheme == "random":
            self.terrain_mesh.visual.vertex_colors = self.np_rng.choice(
                range(256), size=(len(self.terrain_mesh.vertices), 4)
            )
        elif self.cfg.color_scheme == "none":
            pass
        else:
            raise ValueError(f"Invalid color scheme: {self.cfg.color_scheme}.")

        # offset the entire terrain and origins so that it is centered
        # -- terrain mesh
        transform = np.eye(4)
        transform[:2, -1] = -self.cfg.size[0] * self.cfg.num_rows * 0.5, -self.cfg.size[1] * self.cfg.num_cols * 0.5
        self.terrain_mesh.apply_transform(transform)
        # -- terrain origins
        self.terrain_origins += transform[:3, -1]
        # -- valid patches
        terrain_origins_torch = torch.tensor(self.terrain_origins, dtype=torch.float, device=self.device).unsqueeze(2)
        for name, value in self.flat_patches.items():
            self.flat_patches[name] = value + terrain_origins_torch

    def __str__(self):
        """Return a string representation of the terrain generator."""
        msg = "Terrain Generator:"
        msg += f"\n\tSeed: {self.cfg.seed}"
        msg += f"\n\tNumber of rows: {self.cfg.num_rows}"
        msg += f"\n\tNumber of columns: {self.cfg.num_cols}"
        msg += f"\n\tSub-terrain size: {self.cfg.size}"
        msg += f"\n\tSub-terrain types: {list(self.cfg.sub_terrains.keys())}"
        msg += f"\n\tCurriculum: {self.cfg.curriculum}"
        msg += f"\n\tDifficulty range: {self.cfg.difficulty_range}"
        msg += f"\n\tColor scheme: {self.cfg.color_scheme}"
        msg += f"\n\tUse cache: {self.cfg.use_cache}"
        if self.cfg.use_cache:
            msg += f"\n\tCache directory: {self.cfg.cache_dir}"

        return msg

    """
    Terrain generator functions.
    """

    def _generate_random_terrains(self):
        """Add terrains based on randomly sampled difficulty parameter."""
        # normalize the proportions of the sub-terrains
        proportions = np.array([sub_cfg.proportion for sub_cfg in self.cfg.sub_terrains.values()])
        proportions /= np.sum(proportions)
        # create a list of all terrain configs
        sub_terrains_cfgs = list(self.cfg.sub_terrains.values())

        # randomly sample sub-terrains
        for index in range(self.cfg.num_rows * self.cfg.num_cols):
            # coordinate index of the sub-terrain
            (sub_row, sub_col) = np.unravel_index(index, (self.cfg.num_rows, self.cfg.num_cols))
            # randomly sample terrain index
            sub_index = self.np_rng.choice(len(proportions), p=proportions)
            # randomly sample difficulty parameter
            difficulty = self.np_rng.uniform(*self.cfg.difficulty_range)
            # generate terrain
            mesh, origin = self._get_terrain_mesh(difficulty, sub_terrains_cfgs[sub_index])
            # add to sub-terrains
            self._add_sub_terrain(mesh, origin, sub_row, sub_col, sub_terrains_cfgs[sub_index], sub_index)

    def _generate_curriculum_terrains(self):
        """Add terrains based on the difficulty parameter."""
        # normalize the proportions of the sub-terrains
        proportions = np.array([sub_cfg.proportion for sub_cfg in self.cfg.sub_terrains.values()])
        proportions /= np.sum(proportions)

        # find the sub-terrain index for each column
        # we generate the terrains based on their proportion (not randomly sampled)
        sub_indices = []
        for index in range(self.cfg.num_cols):
            sub_index = np.min(np.where(index / self.cfg.num_cols + 0.001 < np.cumsum(proportions))[0])
            sub_indices.append(sub_index)
        sub_indices = np.array(sub_indices, dtype=np.int32)
        # create a list of all terrain configs
        sub_terrains_cfgs = list(self.cfg.sub_terrains.values())

        # curriculum-based sub-terrains
        for sub_col in range(self.cfg.num_cols):
            for sub_row in range(self.cfg.num_rows):
                # vary the difficulty parameter linearly over the number of rows
                # note: based on the proportion, multiple columns can have the same sub-terrain type.
                #  Thus to increase the diversity along the rows, we add a small random value to the difficulty.
                #  This ensures that the terrains are not exactly the same. For example, if the
                #  the row index is 2 and the number of rows is 10, the nominal difficulty is 0.2.
                #  We add a small random value to the difficulty to make it between 0.2 and 0.3.
                lower, upper = self.cfg.difficulty_range
                difficulty = (sub_row + self.np_rng.uniform()) / self.cfg.num_rows
                difficulty = lower + (upper - lower) * difficulty
                # generate terrain
                mesh, origin = self._get_terrain_mesh(difficulty, sub_terrains_cfgs[sub_indices[sub_col]])
                # add to sub-terrains
                self._add_sub_terrain(
                    mesh, origin, sub_row, sub_col, sub_terrains_cfgs[sub_indices[sub_col]], sub_indices[sub_col]
                )

    """
    Internal helper functions.
    """

    def _add_terrain_border(self):
        """Add a surrounding border over all the sub-terrains into the terrain meshes."""
        # border parameters
        border_size = (
            self.cfg.num_rows * self.cfg.size[0] + 2 * self.cfg.border_width,
            self.cfg.num_cols * self.cfg.size[1] + 2 * self.cfg.border_width,
        )
        inner_size = (self.cfg.num_rows * self.cfg.size[0], self.cfg.num_cols * self.cfg.size[1])
        border_center = (
            self.cfg.num_rows * self.cfg.size[0] / 2,
            self.cfg.num_cols * self.cfg.size[1] / 2,
            -self.cfg.border_height / 2,
        )
        # border mesh
        border_meshes = make_border(border_size, inner_size, height=abs(self.cfg.border_height), position=border_center)
        border = trimesh.util.concatenate(border_meshes)
        # update the faces to have minimal triangles
        selector = ~(np.asarray(border.triangles)[:, :, 2] < -0.1).any(1)
        border.update_faces(selector)
        self._add_terrain_seams(border)
        # add the border to the list of meshes
        self.terrain_meshes.append(border)

    def _add_terrain_seams(self, border: trimesh.Trimesh):
        """Fill actual height differences between neighboring tiles and the outer border."""
        seams = []
        for cell, mesh in self._terrain_meshes_by_cell.items():
            for axis in (0, 1):
                for side in (0, 1):
                    if side == 0 and cell[axis] > 0:
                        continue  # Each internal boundary is joined from its lower-index tile.
                    neighbor = list(cell)
                    neighbor[axis] += 2 * side - 1
                    other = self._terrain_meshes_by_cell.get(tuple(neighbor))
                    if other is None:
                        if self.cfg.border_width <= 0.0:
                            continue
                        other = border
                    seam = _make_terrain_seam(mesh, other, axis, (cell[axis] + side) * self.cfg.size[axis])
                    if seam is not None:
                        seams.append(seam)
        if seams:
            self.terrain_meshes.append(trimesh.util.concatenate(seams))

    def _add_sub_terrain(
        self,
        mesh: trimesh.Trimesh,
        origin: np.ndarray,
        row: int,
        col: int,
        sub_terrain_cfg: SubTerrainBaseCfg,
        sub_terrain_index: int,
    ):
        """Add input sub-terrain to the list of sub-terrains.

        This function adds the input sub-terrain mesh to the list of sub-terrains and updates the origin
        of the sub-terrain in the list of origins. It also samples flat patches if specified.

        Args:
            mesh: The mesh of the sub-terrain.
            origin: The origin of the sub-terrain.
            row: The row index of the sub-terrain.
            col: The column index of the sub-terrain.
        """
        # sample flat patches if specified
        if sub_terrain_cfg.flat_patch_sampling is not None:
            logger.info(f"Sampling flat patches for sub-terrain at (row, col):  ({row}, {col})")
            # convert the mesh to warp mesh
            wp_mesh = convert_to_warp_mesh(mesh.vertices, mesh.faces, device=self.device)
            # sample flat patches based on each patch configuration for that sub-terrain
            for name, patch_cfg in sub_terrain_cfg.flat_patch_sampling.items():
                patch_cfg: FlatPatchSamplingCfg
                # create the flat patches tensor (if not already created)
                if name not in self.flat_patches:
                    self.flat_patches[name] = torch.zeros(
                        (self.cfg.num_rows, self.cfg.num_cols, patch_cfg.num_patches, 3), device=self.device
                    )
                # add the flat patches to the tensor
                self.flat_patches[name][row, col] = find_flat_patches(
                    wp_mesh=wp_mesh,
                    origin=origin,
                    num_patches=patch_cfg.num_patches,
                    patch_radius=patch_cfg.patch_radius,
                    x_range=patch_cfg.x_range,
                    y_range=patch_cfg.y_range,
                    z_range=patch_cfg.z_range,
                    max_height_diff=patch_cfg.max_height_diff,
                )

        # transform the mesh to the correct position
        transform = np.eye(4)
        transform[0:2, -1] = (row + 0.5) * self.cfg.size[0], (col + 0.5) * self.cfg.size[1]
        mesh.apply_transform(transform)
        # add mesh to the list
        self.terrain_meshes.append(mesh)
        self._terrain_meshes_by_cell[row, col] = mesh
        self.terrain_mesh_origins.append(origin + transform[:3, -1])
        sub_terrain_name = list(self.cfg.sub_terrains.keys())[sub_terrain_index]
        self.terrain_mesh_names.append(sub_terrain_name)
        # add origin to the list
        self.terrain_origins[row, col] = origin + transform[:3, -1]
        # add terrain index to the list
        self.terrain_indices[row, col] = sub_terrain_index

    def _get_terrain_mesh(self, difficulty: float, cfg: SubTerrainBaseCfg) -> tuple[trimesh.Trimesh, np.ndarray]:
        """Generate a sub-terrain mesh based on the input difficulty parameter.

        If caching is enabled, the sub-terrain is cached and loaded from the cache if it exists.
        The cache is stored in the cache directory specified in the configuration.

        .. Note:
            This function centers the 2D center of the mesh and its specified origin such that the
            2D center becomes :math:`(0, 0)` instead of :math:`(size[0] / 2, size[1] / 2).

        Args:
            difficulty: The difficulty parameter.
            cfg: The configuration of the sub-terrain.

        Returns:
            The sub-terrain mesh and origin.
        """
        # copy the configuration
        cfg = cfg.copy()
        # add other parameters to the sub-terrain configuration
        cfg.difficulty = float(difficulty)
        cfg.seed = self.cfg.seed
        # generate hash for the sub-terrain
        sub_terrain_hash = dict_to_md5_hash(cfg.to_dict())
        # generate the file name
        sub_terrain_cache_dir = os.path.join(self.cfg.cache_dir, sub_terrain_hash)
        sub_terrain_obj_filename = os.path.join(sub_terrain_cache_dir, "mesh.obj")
        sub_terrain_csv_filename = os.path.join(sub_terrain_cache_dir, "origin.csv")
        sub_terrain_meta_filename = os.path.join(sub_terrain_cache_dir, "cfg.yaml")
        sub_terrain_exclusion_filename = os.path.join(sub_terrain_cache_dir, "exclude_virtual_obstacle.npy")

        # check if hash exists - if true, load the mesh and origin and return
        if self.cfg.use_cache and os.path.exists(sub_terrain_obj_filename) and os.path.exists(sub_terrain_exclusion_filename):
            # load existing mesh
            mesh = trimesh.load_mesh(sub_terrain_obj_filename, process=False)
            mesh.face_attributes["exclude_virtual_obstacle"] = np.load(sub_terrain_exclusion_filename)
            origin = np.loadtxt(sub_terrain_csv_filename, delimiter=",")
            # return the generated mesh
            return mesh, origin

        # generate the terrain
        meshes, origin = cfg.function(difficulty, cfg)
        for mesh in meshes:
            mesh.face_attributes.setdefault("exclude_virtual_obstacle", np.zeros(len(mesh.faces), dtype=bool))
        mesh = trimesh.util.concatenate(meshes)
        # offset mesh such that they are in their center
        transform = np.eye(4)
        transform[0:2, -1] = -cfg.size[0] * 0.5, -cfg.size[1] * 0.5
        mesh.apply_transform(transform)
        # change origin to be in the center of the sub-terrain
        origin += transform[0:3, -1]

        # if caching is enabled, save the mesh and origin
        if self.cfg.use_cache:
            # create the cache directory
            os.makedirs(sub_terrain_cache_dir, exist_ok=True)
            # save the data
            mesh.export(sub_terrain_obj_filename)
            np.save(sub_terrain_exclusion_filename, mesh.face_attributes["exclude_virtual_obstacle"])
            np.savetxt(sub_terrain_csv_filename, origin, delimiter=",", header="x,y,z")
            dump_yaml(sub_terrain_meta_filename, cfg)
        # return the generated mesh
        return mesh, origin
