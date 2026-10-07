"""Configuration classes for LocoLab mesh terrains."""

from __future__ import annotations

from dataclasses import MISSING
from typing import Literal

from isaaclab.terrains import SubTerrainBaseCfg
from isaaclab.utils import configclass

from . import locolab_mesh_terrains
from .locolab_hf_terrains_cfg import PoleParamsCfg, RoughnessParamsCfg


@configclass
class MeshRoughTerrainCfg(RoughnessParamsCfg, PoleParamsCfg, SubTerrainBaseCfg):
    """Mesh terrain with optional Perlin roughness and obstacle poles.

    Roughness and poles use the same parameters as :class:`HfRoughTerrainCfg`.
    The generator fills :attr:`horizontal_scale` and :attr:`vertical_scale` from
    :class:`~locolab.utils.terrains.terrain_generator_cfg.TerrainGeneratorCfg`.
    They control noise sampling and tread tessellation density, not box sizes.
    Stair/box footprints stay metric; only vertex z is displaced. Poles are
    cylinders or square prisms that sit on the local mesh surface.
    """

    apply_roughness: float = 0.0
    """Probability of applying roughness to a generated sub-terrain. Must be within [0, 1]."""

    horizontal_scale: float = 0.1
    """Sampling resolution of the roughness field along x and y (in m)."""

    vertical_scale: float = 0.005
    """Sampling resolution of the roughness field along z (in m)."""


@configclass
class MeshRepeatedBoxesTerrainCfg(SubTerrainBaseCfg):
    """Repeated boxes with independently randomized length and width."""

    function = locolab_mesh_terrains.mesh_random_size_repeated_boxes_terrain

    num_objects_range: tuple[int, int] = MISSING
    num_objects_type: Literal["random", "difficulty"] = "difficulty"
    box_height_range: tuple[float, float] = MISSING
    box_length_range: tuple[float, float] = MISSING
    box_width_range: tuple[float, float] = MISSING
    angle_range: tuple[float, float] = (0.0, 0.0)
    angle_type: Literal["random", "difficulty"] = "random"
    angle_degrees: bool = True
    center_platform_width: float = 1.0
    platform_height: float = -1.0
    abs_height_noise: tuple[float, float] = (0.0, 0.0)
    rel_height_noise: tuple[float, float] = (1.0, 1.0)


@configclass
class MeshPyramidStairsTerrainCfg(MeshRoughTerrainCfg):
    """Isaac Lab pyramid stairs generated as metric meshes, with optional roughness."""

    function = locolab_mesh_terrains.mesh_pyramid_stairs_terrain

    stair_height_range: tuple[float, float] = MISSING
    """The minimum and maximum stair height (in m). Scales with difficulty."""

    stair_width: float = MISSING
    """The nominal tread width (in m). Not quantized by height-field scales."""

    stair_width_noise_range: tuple[float, float] = (0.0, 0.0)
    """Range for discrete noise added to :attr:`stair_width` for each sub-terrain (in m)."""

    stair_width_noise_step: float = 0.1
    """Smallest increment used when sampling stair-width noise (in m)."""

    center_platform_width: float = 1.0
    """The width of the square platform at the center of the terrain (in m)."""

    border_width: float = 0.0
    """The width of the flat border around the terrain (in m)."""

    holes: bool = False
    """Whether to leave holes outside the pyramid stairs."""

    inverted: bool = False
    """Whether to generate inverted pyramid stairs."""


@configclass
class MeshInvertedPyramidStairsTerrainCfg(MeshPyramidStairsTerrainCfg):
    """Inverted pyramid stairs generated as metric meshes, with optional roughness."""

    inverted: bool = True


@configclass
class MeshFloatingPyramidStairsTerrainCfg(MeshRoughTerrainCfg):
    """Pyramid stairs whose treads are floating slabs instead of solid blocks.

    The center platform and the stair treads use the same heights as
    :class:`MeshPyramidStairsTerrainCfg`. Each tread is a box of height
    :attr:`stair_thickness` measured downward from that tread surface. The open
    gap under a tread, down to the next lower tread, is
    ``stair_height - stair_thickness``. Keep the thickness below the stair height
    to leave that gap open.

    :attr:`stair_type` selects the footprint. ``pyramid`` builds full concentric
    rings. ``cross`` builds four flights of width :attr:`center_platform_width`, one
    on each side of the center platform. ``random`` picks one of those two
    footprints with equal probability for each generated sub-terrain.

    Ascending stairs sit above a ground plane at ``z = 0``. Inverted stairs keep
    the outer border at ``z = 0`` and add a pit floor one step below the center slab.
    """

    function = locolab_mesh_terrains.mesh_floating_pyramid_stairs_terrain

    stair_height_range: tuple[float, float] = MISSING
    """The minimum and maximum stair height (in m). Scales with difficulty."""

    stair_width: float = MISSING
    """The nominal tread width (in m). Not quantized by height-field scales."""

    stair_width_noise_range: tuple[float, float] = (0.0, 0.0)
    """Uniform noise added to :attr:`stair_width` for each sub-terrain (in m)."""

    stair_thickness: float = 0.05
    """Vertical thickness of every floating tread and the center platform (in m)."""

    center_platform_width: tuple[float, float] = MISSING
    """Range of square center platform widths, sampled once per sub-terrain (in m).

    When :attr:`stair_type` is ``cross`` this is also the width of each of the four stair flights.
    """

    border_width: float = 0.0
    """The width of the flat border around the terrain (in m)."""

    stair_type: Literal["pyramid", "cross", "random"] = "pyramid"
    """``pyramid`` fills concentric rings. ``cross`` keeps four equal-width flights.

    ``random`` samples one of those two footprints uniformly for each sub-terrain.
    """

    inverted: bool = False
    """Whether to generate inverted pyramid stairs."""

    extend_center_platform_under_stairs: bool = True
    """Extend the low floor beneath the center platform across the stair footprint when inverted."""

    floor_thickness: float = 0.1
    """Thickness of the low floor when inverted, in meters."""


@configclass
class MeshInvertedFloatingPyramidStairsTerrainCfg(MeshFloatingPyramidStairsTerrainCfg):
    """Inverted floating pyramid stairs, with optional roughness."""

    inverted: bool = True


@configclass
class MeshStraightGapTerrainCfg(MeshRoughTerrainCfg):
    """Straight corridor along x with a gap sequence on each side.

    This is the mesh counterpart to :class:`HfStraightGapTerrainCfg`. The height-field
    version snaps every x and y span to ``horizontal_scale`` and the pit depth to
    ``vertical_scale``. This version places boxes in meters, so the sampled widths,
    offsets, and depths are the collision dimensions.
    Roughness and poles use the shared mesh parameters and are applied only on the
    walkable tops.

    Each side packs as many gaps as fit at the difficulty-scaled gap width.
    Consecutive gaps are separated by an island. The center platform width is
    sampled along x. A side with two gaps looks like::

        landing | gap | island | gap | center
    """

    function = locolab_mesh_terrains.mesh_straight_gap_terrain

    gap_width_range: tuple[float, float] = MISSING
    """The minimum and maximum gap width in meters. Scales with difficulty."""

    gap_depth_range: tuple[float, float] = MISSING
    """The minimum and maximum gap depth in meters."""

    gap_depth_type: Literal["difficulty", "random"] = "difficulty"
    """How gap depth is sampled. Must be ``"difficulty"`` or ``"random"``."""

    center_platform_width_range: tuple[float, float] = MISSING
    """The center platform width along x, in meters."""

    island_width_range: tuple[float, float] = (1.0, 2.0)
    """Island size in meters. Each island samples x and y independently from this range.

    The minimum is a hard lower bound on each island's x-width. Landings and the
    center platform also sample their y-width from this range.
    Each x-edge landing reserves half of a sample from this range along x, so two
    neighboring gap tiles join into about one island width. The painted landing
    then extends from the tile edge to the first gap.
    """

    island_y_offset_range: tuple[float, float] = (0.0, 0.0)
    """Lateral offset of each island center relative to the corridor, in meters.

    Sampled independently per island. Positive is +y. Landings and the center
    platform stay on the corridor. Unused when that side has only one gap.
    """

    island_height_offset_range: tuple[float, float] = (0.0, 0.0)
    """Height of each island relative to the landings and center, in meters.

    Sampled independently per island. Unused when that side has only one gap.
    """

    border_width: float = 0.05
    """Thickness of the perimeter wall, in meters.

    The wall rises from the pit floor to z=0 so the tile edge meets neighboring
    terrain. Set to 0 to leave the pit open at the border.
    """


@configclass
class MeshHurdleTerrainCfg(SubTerrainBaseCfg):
    """Rectangular hurdles generated directly as metric mesh boxes.

    This is the mesh counterpart to :class:`HfHurdleTerrainCfg`.  The HF
    version first rasterizes the layout onto horizontal and vertical grids and
    then converts that height field to a mesh, so its physical hurdle width and
    height can differ from the configured values by up to one sampling step.
    This version creates explicit ``trimesh`` boxes in meters and therefore
    preserves the configured collision dimensions exactly.  Keep the HF class
    when height-field sampling or roughness compatibility is required; use this
    class for contact-sensitive hurdle and parkour tasks.
    """

    function = locolab_mesh_terrains.mesh_hurdle_terrain

    hurdle_width_range: tuple[float, float] = MISSING
    """The minimum and maximum hurdle width in meters."""

    hurdle_height_range: tuple[float, float] = MISSING
    """The minimum and maximum hurdle height in meters."""

    spacing_range: tuple[float, float] = MISSING
    """Flat spacing sampled once per terrain, in meters.

    The sampled value is the gap between consecutive hurdle rings. The first
    ring is flush with the center platform, so this gap starts outside that
    ring. At least half of it is left between the outermost ring and the nearer
    tile border. Ring count is the largest number that still fits.
    """

    center_platform_width_range: tuple[float, float] = MISSING
    """Width of the center square platform, sampled uniformly, in meters.

    The innermost hurdle ring is placed on the edge of this platform.
    """


@configclass
class MeshStraightClimbTerrainCfg(MeshRoughTerrainCfg):
    """Climb walls along x, with the wall count chosen to fill the tile.

    Walls are explicit ``trimesh`` boxes in meters, so the sampled thickness,
    length, spacing, and height are the collision dimensions.

    Layout along x, for two pairs::

        margin | wall | spacing | wall | center | wall | spacing | wall | margin

    The first full wall on each side is flush with the center platform. Spacing
    is the flat gap between later walls. Pair count is however many full walls
    fit while leaving at least half of the sampled spacing before each x border.
    If that margin is still at least one spacing plus half a wall, each x
    border independently adds a wall of thickness ``wall_width / 2`` with
    probability :attr:`apply_edge_half_wall`.
    """

    function = locolab_mesh_terrains.mesh_straight_climb_terrain

    wall_width_range: tuple[float, float] = MISSING
    """Minimum and maximum wall thickness along x, in meters.

    Thickness decreases from the maximum to the minimum as difficulty rises,
    matching :attr:`MeshHurdleTerrainCfg.hurdle_width_range`. Narrower walls
    can leave room for more pairs.
    """

    wall_height_range: tuple[float, float] = MISSING
    """Minimum and maximum wall height, in meters. Increases with difficulty."""

    wall_height_noise_range: tuple[float, float] = (0.0, 0.0)
    """Uniform noise added to the difficulty-scaled wall height, in meters.

    Sampled once per sub-terrain and shared by every wall, including a border
    half-wall. The result must stay non-negative.
    """

    wall_length_range: tuple[float, float] = MISSING
    """Minimum and maximum wall length along y, in meters. Sampled uniformly.

    The sampled length is shared by every wall and is centered on the tile.
    Values above the terrain size along y are clipped to that size.
    """

    spacing_range: tuple[float, float] = MISSING
    """Flat gap between adjacent walls, sampled once per sub-terrain, in meters.

    The first full wall on each side stays on the center-platform edge, so this
    gap starts outside that wall. At least half of it is left between the
    outermost full wall and each x border. Pair count is the largest number that
    still fits. A half-width border wall also needs one extra sample of this
    spacing between itself and the outermost full wall.
    """

    apply_edge_half_wall: float = 0.0
    """Probability of placing a half-width wall on one x border.

    Each border is sampled independently, and only when the margin can hold the
    half wall plus one spacing. ``0`` never places one. ``1`` places one on
    every border that fits. Defaults to 0.
    """

    center_platform_width_range: tuple[float, float] = MISSING
    """Clear width of the center ground along x, sampled uniformly, in meters.

    The inner face of the first wall on each side lies on the edge of this span.
    """


@configclass
class MeshStraightStairsTerrainCfg(SubTerrainBaseCfg):
    """Stairs that rise to a center platform inside a flat border.

    The flat margin outside the finished stairs is :attr:`border_width` on
    every side, the same role as :class:`MeshPyramidStairsTerrainCfg`. Treads
    keep :attr:`stair_width` and run up to that margin. The flight spans the
    full inner width along y.
    """

    function = locolab_mesh_terrains.mesh_straight_stairs_terrain

    stair_width: float = MISSING
    """The fixed width of each stair in y direction (in m)."""

    stair_width_noise_range: tuple[float, float] = (0.0, 0.0)
    """Range for discrete noise added to :attr:`stair_width` for each sub-terrain (in m)."""

    stair_width_noise_step: float = 0.1
    """Smallest increment used when sampling stair-width noise (in m)."""

    stair_height_range: tuple[float, float] = MISSING
    """The minimum and maximum height of each stair (in m). Scales with difficulty."""

    stair_length_range: tuple[float, float] = MISSING
    """Minimum and maximum tread length along y, in meters.

    The flight is extended to the inner edge of :attr:`border_width`, so this
    range does not leave a wider flat margin than the border.
    """

    num_stairs_range: tuple[int, int] = MISSING
    """Inclusive minimum and exclusive maximum stair count on each side.

    The generator uses the largest count in this range that fits between the
    border and the sampled platform. The count is reduced when that flight
    would pass the inner edge of the border.
    """

    center_platform_width_range: tuple[float, float] = MISSING
    """Minimum and maximum width of the center platform, in meters.

    The sampled width is kept. Leftover shorter than two stair widths is added
    so the outer step meets the border. If the stair-count range stops before
    the inner length is filled, the platform takes that unused length.
    """

    border_width: float = 0.0
    """Flat ground outside the finished stairs, in meters.

    For these ascending stairs it is the margin past the outer step on every
    side, matching :class:`MeshPyramidStairsTerrainCfg`. Inverted stairs use
    the same value as the flat ring before the first drop. Zero puts the first
    step on the tile boundary.
    """


@configclass
class MeshInvertedStraightStairsTerrainCfg(MeshStraightStairsTerrainCfg):
    """Stairs that descend from a flat outer ring to a low center platform.

    The ring at the tile edge is :attr:`border_width` and is not a stair tread.
    The first drop starts on its inner edge. Spare length widens the center
    platform. Along y the ring is at least :attr:`border_width`.
    """

    function = locolab_mesh_terrains.mesh_inverted_straight_stairs_terrain


@configclass
class MeshCorridorTerrainCfg(MeshRoughTerrainCfg):
    """Two walls with centered openings whose dimensions stay in meters."""

    function = locolab_mesh_terrains.mesh_corridor_terrain

    center_platform_width_range: tuple[float, float] = MISSING
    """Range of clear ground widths between the inner wall faces along x, in meters."""

    wall_height_range: tuple[float, float] = MISSING
    """Minimum and maximum wall height, sampled once per sub-terrain in meters."""

    wall_width_range: tuple[float, float] = MISSING
    """Minimum and maximum wall thickness along x, sampled in meters."""

    wall_spacing_range: tuple[float, float] = MISSING
    """Opening width along y, decreasing from maximum to minimum with difficulty."""

    border_width: float = 0.0
    """Flat margin around the walls, in meters."""


@configclass
class MeshCircularDoorsTerrainCfg(SubTerrainBaseCfg):
    """Concentric circular walls with evenly spaced door openings."""

    function = locolab_mesh_terrains.mesh_circular_doors_terrain

    inner_radius: float = 1.0
    """Radius of the innermost wall centerline, in meters."""

    inner_radius_noise_range: tuple[float, float] = (0.0, 0.0)
    """Additive random range applied to :attr:`inner_radius`, in meters."""

    ring_spacing: float = 1.0
    """Distance between neighboring wall centerlines, in meters."""

    ring_spacing_noise_range: tuple[float, float] = (0.0, 0.0)
    """Additive random range applied to :attr:`ring_spacing`, in meters."""

    wall_height_range: tuple[float, float] = (1.0, 1.8)
    """Minimum and maximum wall height, in meters. Increases with difficulty."""

    wall_thickness: float = 0.12
    """Radial thickness of each wall, in meters."""

    wall_arc_length: float = 1.5
    """Target length of each solid wall arc between doors, in meters."""

    wall_arc_length_noise_range: tuple[float, float] = (0.0, 0.0)
    """Additive random range applied to :attr:`wall_arc_length`, in meters."""

    door_width: float = 1.0
    """Target width of each door opening along the ring, in meters."""

    door_width_noise_range: tuple[float, float] = (0.0, 0.0)
    """Additive random range applied to :attr:`door_width`, in meters."""

    door_frame_width: float = 0.12
    """Tangential thickness of each door-frame post, in meters."""

    wall_panel_length: float = 0.4
    """Maximum length of one straight wall panel used to approximate an arc."""

    border_width: float = 0.25
    """Flat clearance from the outer ring to the terrain edge, in meters."""


@configclass
class MeshRandomCeilingObstaclesTerrainCfg(SubTerrainBaseCfg):
    """Ground plane with randomly placed overhead blocks."""

    function = locolab_mesh_terrains.mesh_ceiling_obstacles_terrain

    block_num_range: tuple[int, int] = (8, 24)
    """Target block count range, increasing with difficulty. Non-overlap may reduce the placed count."""

    block_width_range: tuple[float, float] = (0.8, 1.8)
    """Minimum and maximum block side lengths along x and y, in meters."""

    block_thickness_range: tuple[float, float] = (0.10, 0.30)
    """Minimum and maximum vertical block thickness, in meters."""

    clearance_height_range: tuple[float, float] = (1.0, 1.8)
    """Lowest and highest clearance from ground to block bottoms, in meters."""

    center_clearance: float = 1.0
    """Half-width of the spawn area kept clear of overhead blocks, in meters."""

    edge_margin: float = 0.25
    """Minimum distance from a block edge to the terrain boundary, in meters."""


@configclass
class MeshHexSteppingStonesTerrainCfg(SubTerrainBaseCfg):
    """Hexagonally packed stepping stones over a configurable pit."""

    function = locolab_mesh_terrains.mesh_hex_stepping_stones_terrain

    pillar_radius_range: tuple[float, float] = (0.12, 0.22)
    """Minimum and maximum pillar radius. Pillars become smaller with difficulty."""

    pillar_spacing_range: tuple[float, float] = (0.45, 0.90)
    """Minimum and maximum pillar center spacing. Spacing increases with difficulty."""

    pillar_clearance: float = 0.05
    """Additional gap between neighboring pillar footprints, in meters."""

    pit_depth_range: tuple[float, float] = (0.5, 1.0)
    """Range from which the pit depth is sampled for each terrain, in meters."""

    stone_height_noise_range: tuple[float, float] = (-0.1, 0.1)
    """Per-stone height offset from the nominal top at z=0, in meters."""

    stone_height_noise_type: str = "random"
    """Height noise mode: ``"random"`` uses the full range; ``"difficulty"`` scales it by difficulty."""

    center_platform_width_range: tuple[float, float] = (1.8, 1.8)
    """Range of square center platform widths sampled per sub-terrain, in meters."""

    cross_shape: bool = False
    """If True, keep only the horizontal and vertical stepping-stone arms through the center."""

    platform_thickness: float = 0.05
    """Height of the center platform top above the nominal stone tops, in meters."""

    border_width: float = 0.15
    """Width of the perimeter wall surrounding the pit, in meters."""

    edge_margin: float = 0.1
    """Clearance between pillars and the inner terrain boundary, in meters."""


@configclass
class MeshPillarForestTerrainCfg(SubTerrainBaseCfg):
    """Randomly spaced cylindrical pillars on a flat ground plane."""

    function = locolab_mesh_terrains.mesh_pillar_forest_terrain

    pillar_count_range: tuple[int, int] = (12, 28)
    """Minimum and maximum number of pillars. Count increases with difficulty."""

    pillar_radius_range: tuple[float, float] = (0.10, 0.20)
    """Minimum and maximum pillar radius, sampled independently per pillar."""

    pillar_height_range: tuple[float, float] = (0.5, 1.5)
    """Minimum and maximum pillar height, sampled independently per pillar."""

    pillar_clearance: float = 0.10
    """Minimum horizontal clearance between neighboring pillar surfaces, in meters."""

    center_clearance: float = 1.0
    """Radius of the clear center area around the terrain origin, in meters."""

    edge_margin: float = 0.20
    """Minimum distance between a pillar surface and the terrain boundary, in meters."""


@configclass
class MeshRingPlatformsTerrainCfg(SubTerrainBaseCfg):
    """Concentric raised square platform rings around a central open area."""

    function = locolab_mesh_terrains.mesh_ring_platforms_terrain

    center_platform_width_range: tuple[float, float] = (1.2, 1.2)
    """Minimum and maximum width of the open square inside the innermost ring, in meters."""

    ring_width: float = 0.35
    """Radial width of each raised ring, in meters."""

    ring_gap_range: tuple[float, float] = (0.4, 0.8)
    """Minimum and maximum flat gap between neighboring rings. Gaps narrow with difficulty."""

    ring_platform_height_range: tuple[float, float] = (0.10, 0.35)
    """Minimum and maximum ring platform height, in meters."""

    border_width: float = 0.25
    """Flat clearance from the outer ring to the terrain edge, in meters."""


@configclass
class MeshMazeTerrainCfg(SubTerrainBaseCfg):
    """Connected grid maze made from metric wall boxes."""

    function = locolab_mesh_terrains.mesh_maze_terrain

    cell_size: float = 1.5
    """Center-to-center size of one maze cell, in meters."""

    wall_thickness: float = 0.15
    """Thickness of maze walls, in meters."""

    wall_height_range: tuple[float, float] = (0.8, 1.8)
    """Minimum and maximum maze wall height, in meters."""

    border_width: float = 0.25
    """Clearance from the maze boundary to the terrain edge, in meters."""
