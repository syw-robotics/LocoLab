"""LocoLab custom height-field terrains."""

from __future__ import annotations

import numpy as np

from isaaclab.terrains.height_field.utils import height_field_to_mesh

from . import locolab_hf_terrains_cfg
from .utils import _finalize_height_field


@height_field_to_mesh
def flat_rough_terrain(difficulty: float, cfg: locolab_hf_terrains_cfg.HfFlatRoughTerrainCfg) -> np.ndarray:
    """Generate flat terrain with LocoLab fractal Perlin roughness."""
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)
    hf_raw = np.zeros((width_pixels, length_pixels), dtype=np.int16)
    return _finalize_height_field(cfg, hf_raw, difficulty)


@height_field_to_mesh
def pyramid_sloped_rough_terrain(difficulty: float, cfg: locolab_hf_terrains_cfg.HfPyramidSlopedRoughTerrainCfg) -> np.ndarray:
    """Generate a terrain with a truncated pyramid structure with optional roughness.

    The terrain is a pyramid-shaped sloped surface with a slope of :obj:`slope` that trims into a flat platform
    at the center. The slope is defined as the ratio of the height change along the x axis to the width along the
    x axis. For example, a slope of 1.0 means that the height changes by 1 unit for every 1 unit of width.

    If the :obj:`cfg.inverted` flag is set to :obj:`True`, the terrain is inverted such that
    the platform is at the bottom.

    Roughness is randomly applied to the terrain surface according to :obj:`cfg.apply_roughness`.

    Args:
        difficulty: The difficulty of the terrain. This is a value between 0 and 1.
        cfg: The configuration for the terrain.

    Returns:
        The height field of the terrain as a 2D numpy array with discretized heights.
        The shape of the array is (width, length), where width and length are the number of points
        along the x and y axis, respectively.
    """
    # resolve terrain configuration
    if cfg.inverted:
        slope = -cfg.slope_range[0] - difficulty * (cfg.slope_range[1] - cfg.slope_range[0])
    else:
        slope = cfg.slope_range[0] + difficulty * (cfg.slope_range[1] - cfg.slope_range[0])

    # switch parameters to discrete units
    # -- horizontal scale
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)
    # -- height
    # we want the height to be 1/2 of the width since the terrain is a pyramid
    height_max = int(slope * cfg.size[0] / 2 / cfg.vertical_scale)
    # -- center of the terrain
    center_x = int(width_pixels / 2)
    center_y = int(length_pixels / 2)

    # create a meshgrid of the terrain
    x = np.arange(0, width_pixels)
    y = np.arange(0, length_pixels)
    xx, yy = np.meshgrid(x, y, sparse=True)
    # offset the meshgrid to the center of the terrain
    xx = (center_x - np.abs(center_x - xx)) / center_x
    yy = (center_y - np.abs(center_y - yy)) / center_y
    # reshape the meshgrid to be 2D
    xx = xx.reshape(width_pixels, 1)
    yy = yy.reshape(1, length_pixels)
    # create a sloped surface
    hf_raw = np.zeros((width_pixels, length_pixels))
    hf_raw = height_max * xx * yy

    # create a flat platform at the center of the terrain
    center_platform_width = int(cfg.center_platform_width / cfg.horizontal_scale / 2)
    # get the height of the platform at the corner of the platform
    x_pf = width_pixels // 2 - center_platform_width
    y_pf = length_pixels // 2 - center_platform_width
    z_pf = hf_raw[x_pf, y_pf]
    hf_raw = np.clip(hf_raw, min(0, z_pf), max(0, z_pf))

    hf_raw = _finalize_height_field(cfg, hf_raw, difficulty)

    # round off the heights to the nearest vertical step
    return np.rint(hf_raw).astype(np.int16)


@height_field_to_mesh
def discrete_obstacles_terrain(
    difficulty: float, cfg: locolab_hf_terrains_cfg.HfDiscreteObstaclesTerrainCfg
) -> np.ndarray:
    """Generate legged_gym-style discrete rectangular obstacles."""
    # resolve terrain configuration
    obstacle_height = cfg.obstacle_height_range[0] + difficulty * (
        cfg.obstacle_height_range[1] - cfg.obstacle_height_range[0]
    )

    # switch parameters to discrete units
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)
    obstacle_height = int(obstacle_height / cfg.vertical_scale)
    obstacle_width_min = int(cfg.obstacle_width_range[0] / cfg.horizontal_scale)
    obstacle_width_max = int(cfg.obstacle_width_range[1] / cfg.horizontal_scale)
    center_platform_width = int(cfg.center_platform_width / cfg.horizontal_scale)

    if obstacle_height <= 0:
        raise ValueError(f"Obstacle height must be positive. Got: {obstacle_height * cfg.vertical_scale}.")
    if obstacle_width_min <= 0:
        raise ValueError(f"Minimum obstacle width must be positive. Got: {cfg.obstacle_width_range[0]}.")
    if obstacle_width_max < obstacle_width_min:
        raise ValueError(
            "Maximum obstacle width must be greater than or equal to the minimum obstacle width:"
            f" {cfg.obstacle_width_range[1]} < {cfg.obstacle_width_range[0]}."
        )

    # create discrete ranges for the legged_gym-style obstacles
    obstacle_width_range = np.arange(obstacle_width_min, obstacle_width_max + 1, 4)
    obstacle_length_range = np.arange(obstacle_width_min, obstacle_width_max + 1, 4)
    if len(obstacle_width_range) == 0:
        obstacle_width_range = np.array([obstacle_width_min])
    if len(obstacle_length_range) == 0:
        obstacle_length_range = np.array([obstacle_width_min])

    height_range = np.array([-obstacle_height, -obstacle_height // 2, obstacle_height // 2, obstacle_height], dtype=np.int16)
    hf_raw = np.zeros((width_pixels, length_pixels), dtype=np.int16)

    for _ in range(cfg.num_obstacles):
        width = int(np.random.choice(obstacle_width_range))
        length = int(np.random.choice(obstacle_length_range))
        width = min(width, width_pixels)
        length = min(length, length_pixels)

        x_start_range = np.arange(0, max(width_pixels - width, 0) + 1, 4)
        y_start_range = np.arange(0, max(length_pixels - length, 0) + 1, 4)
        x_start = int(np.random.choice(x_start_range))
        y_start = int(np.random.choice(y_start_range))

        hf_raw[x_start : x_start + width, y_start : y_start + length] = np.random.choice(height_range)

    # Keep the center platform free of discrete obstacles before adding surface roughness.
    center_platform_width = min(center_platform_width, width_pixels, length_pixels)
    x1 = (width_pixels - center_platform_width) // 2
    x2 = (width_pixels + center_platform_width) // 2
    y1 = (length_pixels - center_platform_width) // 2
    y2 = (length_pixels + center_platform_width) // 2
    hf_raw[x1:x2, y1:y2] = 0

    hf_raw = _finalize_height_field(cfg, hf_raw, difficulty)

    return np.rint(hf_raw).astype(np.int16)


@height_field_to_mesh
def gap_terrain(
    difficulty: float, cfg: locolab_hf_terrains_cfg.HfGapTerrainCfg
) -> np.ndarray:
    """Generate rectangular gap terrain."""
    # 0------x_11         x_1-----x_2          x_22---end
    # --------- gap_size ----------- gap_size --------
    #          ----------           ----------
    # 0------y11         y_1-----y_2          y_22---end
    # --------- gap_size ----------- gap_size --------
    #          ----------           ----------

    # resolve terrain configuration
    gap_width = (cfg.gap_width_range[1] - cfg.gap_width_range[0]) * difficulty + cfg.gap_width_range[0]
    gap_width_pixels = int(gap_width / cfg.horizontal_scale)
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)

    center_x_pixels = 0.5 * cfg.size[0] / cfg.horizontal_scale
    center_y_pixels = 0.5 * cfg.size[1] / cfg.horizontal_scale
    center_platform_width = (cfg.center_platform_width_range[1] - cfg.center_platform_width_range[0]) * np.random.random() + cfg.center_platform_width_range[0]
    half_platform_width_pixels = int(0.5 * center_platform_width / cfg.horizontal_scale)

    # x direction
    x1 = int(center_x_pixels - half_platform_width_pixels)
    x2 = int(center_x_pixels + half_platform_width_pixels)
    x11 = x1 - gap_width_pixels
    x22 = x2 + gap_width_pixels

    # y direction
    y1 = int(center_y_pixels - half_platform_width_pixels)
    y2 = int(center_y_pixels + half_platform_width_pixels)
    y11 = y1 - gap_width_pixels
    y22 = y2 + gap_width_pixels

    # height for x1-x2 platform: -0.10m to 0.10m
    platform_height = (
        cfg.platform_height_range[1] - cfg.platform_height_range[0]
    ) * np.random.random() + cfg.platform_height_range[0]
    platform_height_pixels = int(platform_height / cfg.vertical_scale)

    # gap depth
    if cfg.gap_depth_type == "difficulty":
        gap_depth = (cfg.gap_depth_range[1] - cfg.gap_depth_range[0]) * difficulty + cfg.gap_depth_range[0]
    elif cfg.gap_depth_type == "random":
        gap_depth = np.random.uniform(cfg.gap_depth_range[0], cfg.gap_depth_range[1])
    else:
        raise ValueError(f"cfg.gap_depth_type' must be 'difficulty' or 'random'. Current value is `{cfg.gap_depth_type}`.")
    gap_depth_pixels = int(gap_depth / cfg.vertical_scale)

    # write height field array
    hf_raw = np.zeros((width_pixels, length_pixels))
    hf_raw[:, :] = -gap_depth_pixels
    hf_raw[:x11, ::] = 0
    hf_raw[x1:x2, y1:y2] = platform_height_pixels
    hf_raw[x11:x22, :y11] = 0
    hf_raw[x11:x22, y22:] = 0
    hf_raw[x22:, :] = 0

    hf_raw = _finalize_height_field(cfg, hf_raw, difficulty)

    # round off the heights to the nearest vertical step
    return np.rint(hf_raw).astype(np.int16)


@height_field_to_mesh
def double_gap_terrain(
    difficulty: float, cfg: locolab_hf_terrains_cfg.HfDoubleGapTerrainCfg
) -> np.ndarray:
    """Generate double rectangular gap terrain."""
    # 0------x_33         x_3-----x_11          x_1------x_2         x_22-----x_4          x_4---end
    # --------- gap_size ----------- gap_size -------------- gap_size ----------- gap_size --------
    #          ----------           ----------              ----------           ----------
    # 0------y_33         y_3-----y_11          y_1------y_2         y_22-----y_4          y_44---end
    # --------- gap_size ----------- gap_size -------------- gap_size ----------- gap_size --------
    #          ----------           ----------              ----------           ----------

    # resolve terrain configuration
    gap_width = (cfg.gap_width_range[1] - cfg.gap_width_range[0]) * difficulty + cfg.gap_width_range[0]
    gap_width_pixels = int(gap_width / cfg.horizontal_scale)
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)

    center_x_pixels = 0.5 * cfg.size[0] / cfg.horizontal_scale
    center_y_pixels = 0.5 * cfg.size[1] / cfg.horizontal_scale
    center_platform_width = (cfg.center_platform_width_range[1] - cfg.center_platform_width_range[0]) * np.random.random() + cfg.center_platform_width_range[0]
    half_platform_width_pixels = int(0.5 * center_platform_width / cfg.horizontal_scale)

    gap_in_between_width = (cfg.gap_in_between_width_range[1] - cfg.gap_in_between_width_range[0]) * np.random.random() + cfg.gap_in_between_width_range[0]
    gap_in_between_width_pixels = int(gap_in_between_width / cfg.horizontal_scale)

    # x direction
    x1 = int(center_x_pixels - half_platform_width_pixels)
    x2 = int(center_x_pixels + half_platform_width_pixels)
    x11 = x1 - gap_width_pixels
    x22 = x2 + gap_width_pixels
    x3 = x11 - gap_in_between_width_pixels
    x4 = x22 + gap_in_between_width_pixels
    x33 = x3 - gap_width_pixels
    x44 = x4 + gap_width_pixels

    # y direction
    y1 = int(center_y_pixels - half_platform_width_pixels)
    y2 = int(center_y_pixels + half_platform_width_pixels)
    y11 = y1 - gap_width_pixels
    y22 = y2 + gap_width_pixels
    y3 = y11 - gap_in_between_width_pixels
    y4 = y22 + gap_in_between_width_pixels
    y33 = y3 - gap_width_pixels
    y44 = y4 + gap_width_pixels

    # height for x1-x2 platform: -0.10m to 0.10m
    platform_height = (
        cfg.platform_height_range[1] - cfg.platform_height_range[0]
    ) * np.random.random() + cfg.platform_height_range[0]
    platform_height_pixels = int(platform_height / cfg.vertical_scale)

    # gap depth
    if cfg.gap_depth_type == "difficulty":
        gap_depth = (cfg.gap_depth_range[1] - cfg.gap_depth_range[0]) * difficulty + cfg.gap_depth_range[0]
    elif cfg.gap_depth_type == "random":
        gap_depth = np.random.uniform(cfg.gap_depth_range[0], cfg.gap_depth_range[1])
    else:
        raise ValueError(f"cfg.gap_depth_type' must be 'difficulty' or 'random'. Current value is `{cfg.gap_depth_type}`.")
    gap_depth_pixels = int(gap_depth / cfg.vertical_scale)

    # write height field array
    hf_raw = np.zeros((width_pixels, length_pixels))
    hf_raw[:, :] = -gap_depth_pixels
    hf_raw[:x33, ::] = 0
    hf_raw[x44:, ::] = 0
    hf_raw[x33:x44, :y33] = 0
    hf_raw[x33:x44, y44:] = 0

    hf_raw[x3:x11, y3:y4] = 0
    hf_raw[x22:x4, y3:y4] = 0
    hf_raw[x11:x22, y3:y11] = 0
    hf_raw[x11:x22, y22:y4] = 0

    hf_raw[x1:x2, y1:y2] = platform_height_pixels

    hf_raw = _finalize_height_field(cfg, hf_raw, difficulty)

    # round off the heights to the nearest vertical step
    return np.rint(hf_raw).astype(np.int16)


@height_field_to_mesh
def straight_gap_terrain(
    difficulty: float, cfg: locolab_hf_terrains_cfg.HfStraightGapTerrainCfg
) -> np.ndarray:
    """Generate a y-limited corridor with independently placed islands along x.

    Gap width, platform width, island x-width, y spans, and lateral offsets are
    truncated to ``horizontal_scale`` cells. Gap depth is truncated to
    ``vertical_scale``. Roughness is added only on painted tops (landings, center,
    islands), not the pit.
    """
    # landing | gap | island | ... | gap | center | gap | ... | island | gap | landing
    num_gaps = cfg.num_gaps_per_side_range
    if isinstance(num_gaps, int):
        num_min = num_max = num_gaps
    elif len(num_gaps) == 1:
        num_min = num_max = int(num_gaps[0])
    elif len(num_gaps) == 2:
        num_min, num_max = int(num_gaps[0]), int(num_gaps[1])
    else:
        raise ValueError(f"Invalid num_gaps_per_side_range: {cfg.num_gaps_per_side_range}.")
    if num_min < 1 or num_min > num_max:
        raise ValueError(f"Invalid num_gaps_per_side_range: {cfg.num_gaps_per_side_range}.")
    num_gaps = int(np.random.randint(num_min, num_max + 1))

    gap_width = (cfg.gap_width_range[1] - cfg.gap_width_range[0]) * difficulty + cfg.gap_width_range[0]
    gap_width_pixels = max(int(gap_width / cfg.horizontal_scale), 1)
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)

    island_w_min, island_w_max = cfg.island_width_range
    if island_w_min <= 0.0 or island_w_min > island_w_max:
        raise ValueError(f"Invalid island_width_range: {cfg.island_width_range}.")
    offset_min, offset_max = cfg.island_y_offset_range
    if offset_min > offset_max:
        raise ValueError(f"Invalid island_y_offset_range: {cfg.island_y_offset_range}.")
    height_min, height_max = cfg.island_height_offset_range
    if height_min > height_max:
        raise ValueError(f"Invalid island_height_offset_range: {cfg.island_height_offset_range}.")

    num_islands = num_gaps - 1

    # sample islands
    def _sample_islands() -> list[tuple[int, float, float, float]]:
        # (x-width in pixels, y-width in meters, y-offset in meters, height in meters), inner first.
        return [
            (
                max(int(np.random.uniform(island_w_min, island_w_max) / cfg.horizontal_scale), 1),
                float(np.random.uniform(island_w_min, island_w_max)),
                float(np.random.uniform(offset_min, offset_max)),
                float(np.random.uniform(height_min, height_max)),
            )
            for _ in range(num_islands)
        ]

    left_islands = _sample_islands() if num_islands > 0 else []
    right_islands = _sample_islands() if num_islands > 0 else []

    def _sample_edge_landing_x_pixels() -> int:
        return max(int(0.5 * np.random.uniform(island_w_min, island_w_max) / cfg.horizontal_scale), 1)

    center_platform_width = (cfg.center_platform_width_range[1] - cfg.center_platform_width_range[0]) * np.random.random() + cfg.center_platform_width_range[0]
    center_x_pixels = 0.5 * cfg.size[0] / cfg.horizontal_scale
    center_y_pixels = 0.5 * cfg.size[1] / cfg.horizontal_scale
    half_platform_x_pixels = max(int(0.5 * center_platform_width / cfg.horizontal_scale), 1)
    inner_left = max(int(center_x_pixels - half_platform_x_pixels), 0)
    inner_right = min(int(center_x_pixels + half_platform_x_pixels), width_pixels)
    if inner_right <= inner_left:
        mid = int(center_x_pixels)
        inner_left = max(mid - 1, 0)
        inner_right = min(mid + 1, width_pixels)

    # fit islands to the gap
    def _fit_side(
        budget: int,
        side_gaps: int,
        side_gap_width: int,
        islands: list[tuple[int, float, float, float]],
    ) -> tuple[int, int, list[tuple[int, float, float, float]]]:
        # Keep the sampled layout if it fits. Otherwise drop gaps, then cap gap and island x-widths.
        budget = max(int(budget), 0)
        side_gap_width = max(int(side_gap_width), 1)
        islands = list(islands)
        side_gaps = max(int(side_gaps), 0)
        if budget < 1 or side_gaps < 1:
            return 0, 0, []
        # 1px gap + 1px island is the smallest layout that still has `side_gaps` pits.
        while side_gaps > 1 and side_gaps + (side_gaps - 1) > budget:
            side_gaps -= 1
            islands = islands[: side_gaps - 1]
        islands = islands[: max(side_gaps - 1, 0)]
        side_gap_width = min(side_gap_width, max((budget - len(islands)) // side_gaps, 1))
        # Shrink island x-widths from the outside so they occupy the leftover budget.
        remain = max(budget - side_gaps * side_gap_width, 0)
        if remain <= 0 or not islands:
            islands = []
        else:
            widths = [max(int(width), 1) for width, *_ in islands]
            overflow = sum(widths) - remain
            if overflow > 0:
                for i in range(len(widths) - 1, -1, -1):
                    take = min(overflow, widths[i])
                    widths[i] -= take
                    overflow -= take
                    if overflow <= 0:
                        break
            islands = [
                (width, y_width, y_offset, height)
                for width, (_, y_width, y_offset, height) in zip(widths, islands)
                if width > 0
            ]
        return side_gaps, side_gap_width, islands

    left_n, left_gap, left_islands = _fit_side(
        max(inner_left - _sample_edge_landing_x_pixels(), 0), num_gaps, gap_width_pixels, left_islands
    )
    right_n, right_gap, right_islands = _fit_side(
        max(width_pixels - inner_right - _sample_edge_landing_x_pixels(), 0),
        num_gaps,
        gap_width_pixels,
        right_islands,
    )
    outer_left = inner_left - (left_n * left_gap + sum(width for width, *_ in left_islands))
    outer_right = inner_right + (right_n * right_gap + sum(width for width, *_ in right_islands))

    if cfg.gap_depth_type == "difficulty":
        gap_depth = (cfg.gap_depth_range[1] - cfg.gap_depth_range[0]) * difficulty + cfg.gap_depth_range[0]
    elif cfg.gap_depth_type == "random":
        gap_depth = np.random.uniform(cfg.gap_depth_range[0], cfg.gap_depth_range[1])
    else:
        raise ValueError(
            f"cfg.gap_depth_type must be 'difficulty' or 'random'. Current value is `{cfg.gap_depth_type}`."
        )
    gap_depth_pixels = int(gap_depth / cfg.vertical_scale)

    hf_raw = np.full((width_pixels, length_pixels), -gap_depth_pixels, dtype=np.float64)
    walkable = np.zeros((width_pixels, length_pixels), dtype=bool)

    # paint the height field
    def _paint(x1: int, x2: int, y_offset_m: float, y_width_m: float, height: float) -> None:
        offset_pixels = int(y_offset_m / cfg.horizontal_scale)
        half_y_pixels = max(int(0.5 * y_width_m / cfg.horizontal_scale), 1)
        y1 = int(center_y_pixels + offset_pixels - half_y_pixels)
        y2 = int(center_y_pixels + offset_pixels + half_y_pixels)
        y1 = max(y1, 0)
        y2 = min(y2, length_pixels)
        if x2 > x1 and y2 > y1:
            hf_raw[x1:x2, y1:y2] = height
            walkable[x1:x2, y1:y2] = True

    left_landing_y = float(np.random.uniform(island_w_min, island_w_max))
    right_landing_y = float(np.random.uniform(island_w_min, island_w_max))
    center_y_width = float(np.random.uniform(island_w_min, island_w_max))
    _paint(0, outer_left, 0.0, left_landing_y, 0.0)
    _paint(inner_left, inner_right, 0.0, center_y_width, 0.0)
    _paint(outer_right, width_pixels, 0.0, right_landing_y, 0.0)
    for inner, sign, gap_pixels, islands in (
        (inner_left, -1, left_gap, left_islands),
        (inner_right, 1, right_gap, right_islands),
    ):
        cursor = inner
        for x_width_pixels, y_width_m, y_offset_m, height_m in islands:
            cursor += sign * gap_pixels
            island_start = cursor
            cursor += sign * x_width_pixels
            lo, hi = (cursor, island_start) if sign < 0 else (island_start, cursor)
            _paint(lo, hi, y_offset_m, y_width_m, int(height_m / cfg.vertical_scale))

    hf_raw = _finalize_height_field(cfg, hf_raw, difficulty, mask=walkable)
    return np.rint(hf_raw).astype(np.int16)


@height_field_to_mesh
def straight_climb_terrain(
    difficulty: float, cfg: locolab_hf_terrains_cfg.HfStraightClimbTerrainCfg
) -> np.ndarray:
    """Generate two climb walls along x with a sampled span in y."""
    wall_height = (cfg.wall_height_range[1] - cfg.wall_height_range[0]) * difficulty + cfg.wall_height_range[0]
    wall_height_pixels = int(wall_height / cfg.vertical_scale)
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)

    center_x = width_pixels // 2
    offset_pixels = int(np.random.uniform(1.0, 2.0) / cfg.horizontal_scale)
    wall_width_pixels = max(int(np.random.uniform(*cfg.wall_width_range) / cfg.horizontal_scale), 1)
    right_x1 = np.clip(center_x + offset_pixels, 0, width_pixels)
    right_x2 = np.clip(center_x + offset_pixels + wall_width_pixels, 0, width_pixels)
    left_x2 = np.clip(center_x - offset_pixels, 0, width_pixels)
    left_x1 = np.clip(center_x - offset_pixels - wall_width_pixels, 0, width_pixels)

    center_y = length_pixels // 2
    half_length_pixels = max(int(0.5 * np.random.uniform(*cfg.wall_length_range) / cfg.horizontal_scale), 1)
    y1 = max(center_y - half_length_pixels, 0)
    y2 = min(center_y + half_length_pixels, length_pixels)

    hf_raw = np.zeros((width_pixels, length_pixels), dtype=np.float64)
    if right_x2 > right_x1 and y2 > y1:
        hf_raw[right_x1:right_x2, y1:y2] = wall_height_pixels
    if left_x2 > left_x1 and y2 > y1:
        hf_raw[left_x1:left_x2, y1:y2] = wall_height_pixels

    hf_raw = _finalize_height_field(cfg, hf_raw, difficulty)
    return np.rint(hf_raw).astype(np.int16)


@height_field_to_mesh
def hurdle_terrain(
    difficulty: float, cfg: locolab_hf_terrains_cfg.HfHurdleTerrainCfg
) -> np.ndarray:
    """Generate rectangular gap terrain."""
    # 0------x_11         x_1-----x_2          x_22---end
    #          ----------           ----------
    # --------- hurdle_size ----------- hurdle_size --------
    # 0------y11         y_1-----y_2          y_22---end
    #          ----------           ----------
    # --------- hurdle_size ----------- hurdle_size --------

    # resolve terrain configuration
    # hurdle width from wide to narrow
    hurdle_width = - (cfg.hurdle_width_range[1] - cfg.hurdle_width_range[0]) * difficulty + cfg.hurdle_width_range[1] + cfg.hurdle_width_range[0]
    hurdle_width_pixels = int(hurdle_width / cfg.horizontal_scale)
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)

    center_x_pixels = 0.5 * cfg.size[0] / cfg.horizontal_scale
    center_y_pixels = 0.5 * cfg.size[1] / cfg.horizontal_scale
    center_platform_width = (cfg.center_platform_width_range[1] - cfg.center_platform_width_range[0]) * np.random.random() + cfg.center_platform_width_range[0]
    half_platform_width_pixels = int(0.5 * center_platform_width / cfg.horizontal_scale)

    # x direction
    x1 = int(center_x_pixels - half_platform_width_pixels)
    x2 = int(center_x_pixels + half_platform_width_pixels)
    x11 = x1 - hurdle_width_pixels
    x22 = x2 + hurdle_width_pixels

    # y direction
    y1 = int(center_y_pixels - half_platform_width_pixels)
    y2 = int(center_y_pixels + half_platform_width_pixels)
    y11 = y1 - hurdle_width_pixels
    y22 = y2 + hurdle_width_pixels

    # hurdle height from short to tall
    hurdle_height = (cfg.hurdle_height_range[1] - cfg.hurdle_height_range[0]) * difficulty + cfg.hurdle_height_range[0]
    hurdle_height_pixels = int(hurdle_height / cfg.vertical_scale)

    # write height field array
    hf_raw = np.zeros((width_pixels, length_pixels))
    hf_raw[:, :] = 0
    hf_raw[x11:x1, y11:y22] = hurdle_height_pixels
    hf_raw[x2:x22, y11:y22] = hurdle_height_pixels
    hf_raw[x1:x2, y11:y1] = hurdle_height_pixels
    hf_raw[x1:x2, y2:y22] = hurdle_height_pixels

    hf_raw = _finalize_height_field(cfg, hf_raw, difficulty)

    # round off the heights to the nearest vertical step
    return np.rint(hf_raw).astype(np.int16)


@height_field_to_mesh
def pyramid_stairs_terrain(difficulty: float, cfg: locolab_hf_terrains_cfg.HfPyramidStairsTerrainCfg) -> np.ndarray:
    """Generate pyramid stairs, snapping tread width to the height-field grid."""
    noise_min, noise_max = cfg.stair_width_noise_range
    if cfg.stair_width <= 0.0:
        raise ValueError(f"stair_width must be positive, got {cfg.stair_width}.")
    if noise_min > noise_max:
        raise ValueError(f"Invalid stair_width_noise_range: {cfg.stair_width_noise_range}.")
    stair_width = float(cfg.stair_width + np.random.uniform(noise_min, noise_max))
    if stair_width <= 0.0:
        raise ValueError(
            f"Sampled stair width must be positive, got {stair_width} from "
            f"{cfg.stair_width} + {cfg.stair_width_noise_range}."
        )

    step_height = cfg.stair_height_range[0] + difficulty * (cfg.stair_height_range[1] - cfg.stair_height_range[0])
    if cfg.inverted:
        step_height *= -1
    # switch parameters to discrete units
    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)
    step_width_pixels = max(int(stair_width / cfg.horizontal_scale), 1)
    step_height = int(step_height / cfg.vertical_scale)
    center_platform_width = int(cfg.center_platform_width / cfg.horizontal_scale)

    # Keep the center at least ``center_platform_width``. Fit as many equal treads as possible
    # and leave the leftover as a flat outer border, matching the mesh pyramid stairs.
    if step_width_pixels > 0:
        num_steps = min(
            (width_pixels - center_platform_width) // (2 * step_width_pixels),
            (length_pixels - center_platform_width) // (2 * step_width_pixels),
        )
    else:
        num_steps = 0
    num_steps = max(int(num_steps), 0)

    remain_x = width_pixels - center_platform_width - 2 * num_steps * step_width_pixels
    remain_y = length_pixels - center_platform_width - 2 * num_steps * step_width_pixels
    start_x = max(remain_x // 2, 0)
    start_y = max(remain_y // 2, 0)
    stop_x = width_pixels - remain_x + start_x
    stop_y = length_pixels - remain_y + start_y

    hf_raw = np.zeros((width_pixels, length_pixels))
    current_step_height = 0
    for _ in range(num_steps):
        start_x += step_width_pixels
        stop_x -= step_width_pixels
        start_y += step_width_pixels
        stop_y -= step_width_pixels
        if stop_x <= start_x or stop_y <= start_y:
            break
        current_step_height += step_height
        hf_raw[start_x:stop_x, start_y:stop_y] = current_step_height

    hf_raw = _finalize_height_field(cfg, hf_raw, difficulty)
    return np.rint(hf_raw).astype(np.int16)
