"""Shared roughness sampling used by height-field and mesh terrains."""

from __future__ import annotations

from typing import Any

import numpy as np


def _sample_perlin_noise(shape: tuple[int, int], cell_size: float) -> np.ndarray:
    """Sample 2-D gradient Perlin noise directly on the target grid."""
    x, y = np.meshgrid(np.arange(shape[0]) / cell_size, np.arange(shape[1]) / cell_size, indexing="ij")
    x_cell = np.floor(x).astype(np.int32)
    y_cell = np.floor(y).astype(np.int32)
    x -= x_cell
    y -= y_cell

    angles = np.random.uniform(0.0, 2.0 * np.pi, size=(x_cell.max() + 2, y_cell.max() + 2))
    gradients = np.stack((np.cos(angles), np.sin(angles)), axis=-1)

    def dot_grid(offset_x: int, offset_y: int) -> np.ndarray:
        gradient = gradients[x_cell + offset_x, y_cell + offset_y]
        return (x - offset_x) * gradient[..., 0] + (y - offset_y) * gradient[..., 1]

    fade_x = 6.0 * x**5 - 15.0 * x**4 + 10.0 * x**3
    fade_y = 6.0 * y**5 - 15.0 * y**4 + 10.0 * y**3
    noise_x0 = dot_grid(0, 0) * (1.0 - fade_x) + dot_grid(1, 0) * fade_x
    noise_x1 = dot_grid(0, 1) * (1.0 - fade_x) + dot_grid(1, 1) * fade_x
    return np.sqrt(2.0) * (noise_x0 * (1.0 - fade_y) + noise_x1 * fade_y) * 0.5 + 0.5


def _sample_fractal_perlin_noise(
    shape: tuple[int, int], horizontal_scale: float, downsampled_scale: float
) -> np.ndarray:
    """Sample InstinctLab-style two-octave Perlin noise, centered around zero."""
    noise = np.zeros(shape, dtype=np.float64)
    amplitude = 1.0
    amplitude_sum = 0.0
    cell_size = 2.0 * downsampled_scale / horizontal_scale

    for _ in range(2):
        noise += amplitude * _sample_perlin_noise(shape, cell_size)
        amplitude_sum += amplitude
        amplitude *= 0.25
        cell_size /= 2.0

    noise -= noise.mean()
    return np.clip(noise / (0.5 * amplitude_sum), -1.0, 1.0)


def _roughness_grid_shape(cfg: Any) -> tuple[int, int]:
    return int(cfg.size[0] / cfg.horizontal_scale), int(cfg.size[1] / cfg.horizontal_scale)


def _resolve_downsampled_scale(cfg: Any) -> float:
    if cfg.downsampled_scale is None:
        return cfg.horizontal_scale
    if cfg.downsampled_scale < cfg.horizontal_scale:
        raise ValueError(
            "Downsampled scale must be larger than or equal to the horizontal scale:"
            f" {cfg.downsampled_scale} < {cfg.horizontal_scale}."
        )
    return cfg.downsampled_scale


def _resolve_roughness_strength(cfg: Any, difficulty: float) -> float:
    roughness_type = getattr(cfg, "roughness_type", "fixed")
    if roughness_type == "difficulty":
        return float(np.clip(difficulty, 0.0, 1.0))
    if roughness_type == "random":
        strengths = getattr(cfg, "roughness_strengths", None)
        if strengths is None:
            strengths = getattr(cfg, "random_strengths", (0.2, 0.4, 0.6, 0.8, 1.0))
        if len(strengths) == 0:
            raise ValueError("roughness_strengths must contain at least one value.")
        return float(np.random.choice(strengths))
    if roughness_type == "fixed":
        return 1.0
    raise ValueError(
        f"Invalid roughness type: {roughness_type}. Must be one of 'difficulty', 'random', or 'fixed'."
    )


def should_apply_roughness(cfg: Any) -> bool:
    """Return whether this sub-terrain should receive roughness."""
    probability = float(getattr(cfg, "apply_roughness", 0.0))
    if not 0.0 <= probability <= 1.0:
        raise ValueError(f"Roughness probability must be within [0, 1]. Got: {probability}.")
    if probability == 0.0:
        return False
    return probability >= 1.0 or np.random.random() < probability


def generate_roughness_height_field(
    cfg: Any, difficulty: float = 0.0, shape: tuple[int, int] | None = None
) -> np.ndarray:
    """Sample the shared Perlin roughness as an integer height field.

    Values are in ``vertical_scale`` units so height-field terrains can add them
    directly. Mesh terrains convert back to meters with ``* cfg.vertical_scale``.
    """
    width_pixels, length_pixels = shape if shape is not None else _roughness_grid_shape(cfg)
    strength = _resolve_roughness_strength(cfg, difficulty)
    height_min = int(cfg.noise_range[0] / cfg.vertical_scale)
    height_max = int(cfg.noise_range[1] / cfg.vertical_scale)
    height_step = int(cfg.noise_step / cfg.vertical_scale)
    if height_min > height_max:
        raise ValueError(f"Noise range must be ordered. Got: {cfg.noise_range}.")
    if height_step <= 0:
        raise ValueError(
            f"Noise step must be greater than or equal to the vertical scale: {cfg.noise_step} < {cfg.vertical_scale}."
        )
    if strength == 0.0:
        return np.zeros((width_pixels, length_pixels), dtype=np.int16)

    downsampled_scale = _resolve_downsampled_scale(cfg) * np.exp2(np.random.random())
    noise = _sample_fractal_perlin_noise((width_pixels, length_pixels), cfg.horizontal_scale, downsampled_scale)
    noise = np.where(
        noise < 0.0,
        noise * abs(min(height_min, 0)),
        noise * max(height_max, 0),
    )
    noise *= strength
    noise = np.rint(noise / height_step) * height_step
    scaled_min = int(np.rint(min(height_min * strength, height_max * strength) / height_step)) * height_step
    scaled_max = int(np.rint(max(height_min * strength, height_max * strength) / height_step)) * height_step
    if scaled_min > scaled_max:
        return np.zeros((width_pixels, length_pixels), dtype=np.int16)
    return np.clip(noise, scaled_min, scaled_max).astype(np.int16)


def sample_height_field(
    height_field: np.ndarray, x: np.ndarray, y: np.ndarray, horizontal_scale: float
) -> np.ndarray:
    """Bilinear-sample a grid whose ``(i, j)`` cell sits at ``(i * scale, j * scale)``."""
    rows, cols = height_field.shape
    if rows < 2 or cols < 2:
        return np.zeros(np.shape(x), dtype=np.float64)

    fx = np.clip(np.asarray(x, dtype=np.float64) / horizontal_scale, 0.0, rows - 1.0)
    fy = np.clip(np.asarray(y, dtype=np.float64) / horizontal_scale, 0.0, cols - 1.0)
    x0 = np.floor(fx).astype(np.intp)
    y0 = np.floor(fy).astype(np.intp)
    x1 = np.minimum(x0 + 1, rows - 1)
    y1 = np.minimum(y0 + 1, cols - 1)
    tx = fx - x0
    ty = fy - y0
    v00 = height_field[x0, y0]
    v10 = height_field[x1, y0]
    v01 = height_field[x0, y1]
    v11 = height_field[x1, y1]
    return v00 * (1.0 - tx) * (1.0 - ty) + v10 * tx * (1.0 - ty) + v01 * (1.0 - tx) * ty + v11 * tx * ty


def apply_perlin_noise(
    cfg: Any, hf_raw: np.ndarray, difficulty: float = 0.0, mask: np.ndarray | None = None
) -> np.ndarray:
    """Add shared Perlin roughness onto a height-field array.

    If ``mask`` is given, only those cells are updated. The noise field is still
    sampled on the full grid so neighboring walkable patches stay spatially coherent.
    """
    noise = generate_roughness_height_field(cfg, difficulty, shape=hf_raw.shape)
    if mask is None:
        hf_raw += noise
    else:
        hf_raw[mask] += noise[mask]
    return hf_raw


def _maybe_apply_roughness(
    cfg: Any, hf_raw: np.ndarray, difficulty: float, mask: np.ndarray | None = None
) -> np.ndarray:
    """Apply roughness according to the configured probability."""
    if not should_apply_roughness(cfg):
        return hf_raw
    return apply_perlin_noise(cfg, hf_raw, difficulty, mask=mask)


def should_apply_poles(cfg: Any) -> bool:
    """Return whether this sub-terrain should receive obstacle poles."""
    probability = float(getattr(cfg, "apply_poles", 0.0))
    if not 0.0 <= probability <= 1.0:
        raise ValueError(f"Pole probability must be within [0, 1]. Got: {probability}.")
    if probability == 0.0:
        return False
    return probability >= 1.0 or np.random.random() < probability


def _pole_height_meters(cfg: Any) -> float:
    height_min, height_max = cfg.pole_height_range
    if height_min <= 0.0 or height_min > height_max:
        raise ValueError(f"Invalid pole_height_range: {cfg.pole_height_range}.")
    return float(np.random.uniform(height_min, height_max))


def _validate_positive_range(name: str, value: tuple[float, float]) -> tuple[float, float]:
    lo, hi = value
    if lo <= 0.0 or lo > hi:
        raise ValueError(f"Invalid {name}: {value}.")
    return float(lo), float(hi)


def _sample_pole_kind_and_size(cfg: Any) -> tuple[str, float]:
    """Return ``("cylinder", radius)`` or ``("box", side)``."""
    cylinder_prob = float(getattr(cfg, "cylinder_probability", 0.5))
    if not 0.0 <= cylinder_prob <= 1.0:
        raise ValueError(f"cylinder_probability must be within [0, 1]. Got: {cylinder_prob}.")
    if np.random.random() < cylinder_prob:
        lo, hi = _validate_positive_range("cylinder_radius_range", cfg.cylinder_radius_range)
        return "cylinder", float(np.random.uniform(lo, hi))
    lo, hi = _validate_positive_range("box_side_range", cfg.box_side_range)
    return "box", float(np.random.uniform(lo, hi))


def _pole_xy_half(kind: str, size: float) -> float:
    return float(size) if kind == "cylinder" else 0.5 * float(size)


def _pole_cover_mask(
    kind: str,
    size: float,
    x_m: float,
    y_m: float,
    x1: int,
    x2: int,
    y1: int,
    y2: int,
    h_scale: float,
) -> np.ndarray:
    """Boolean cover of shape ``(x2 - x1, y2 - y1)`` in height-field pixels."""
    ix = (np.arange(x1, x2) + 0.5) * h_scale
    iy = (np.arange(y1, y2) + 0.5) * h_scale
    dx = ix[:, None] - x_m
    dy = iy[None, :] - y_m
    if kind == "cylinder":
        return dx * dx + dy * dy <= size * size
    half = 0.5 * size
    return (np.abs(dx) <= half) & (np.abs(dy) <= half)


def _pole_pixel_window(
    x_m: float, y_m: float, half: float, rows: int, cols: int, h_scale: float
) -> tuple[int, int, int, int] | None:
    x1 = int(np.floor((x_m - half) / h_scale))
    x2 = int(np.ceil((x_m + half) / h_scale))
    y1 = int(np.floor((y_m - half) / h_scale))
    y2 = int(np.ceil((y_m + half) / h_scale))
    x1 = max(x1, 0)
    y1 = max(y1, 0)
    x2 = min(x2, rows)
    y2 = min(y2, cols)
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, x2, y1, y2


def _sample_pole_placements(
    cfg: Any,
    size_xy: tuple[float, float],
    mask: np.ndarray | None = None,
    horizontal_scale: float | None = None,
) -> list[tuple[float, float, str, float, float]]:
    """Sample poles as ``(x, y, kind, size, height)`` uniformly in the sub-terrain.

    ``kind`` is ``"cylinder"`` (size is radius) or ``"box"`` (size is side length).
    """
    num_min, num_max = cfg.num_poles_range
    if num_min < 0 or num_min > num_max:
        raise ValueError(f"Invalid num_poles_range: {cfg.num_poles_range}.")

    num_poles = int(np.random.randint(int(num_min), int(num_max) + 1))
    margin = float(getattr(cfg, "pole_edge_margin", 0.0))
    clear = float(getattr(cfg, "keep_center_clear", 0.0))
    separation = float(getattr(cfg, "min_pole_separation", 0.0))
    cx, cy = 0.5 * size_xy[0], 0.5 * size_xy[1]
    h_scale = float(horizontal_scale) if horizontal_scale is not None else float(getattr(cfg, "horizontal_scale", 0.1))
    placed: list[tuple[float, float, str, float, float]] = []

    for _ in range(max(num_poles, 0)):
        kind, size = _sample_pole_kind_and_size(cfg)
        half = _pole_xy_half(kind, size)
        lo_x, hi_x = margin + half, size_xy[0] - margin - half
        lo_y, hi_y = margin + half, size_xy[1] - margin - half
        if lo_x >= hi_x or lo_y >= hi_y:
            break
        for _attempt in range(200):
            x = float(np.random.uniform(lo_x, hi_x))
            y = float(np.random.uniform(lo_y, hi_y))
            if clear > 0.0 and (x - cx) ** 2 + (y - cy) ** 2 < clear**2:
                continue
            if any(
                (x - px) ** 2 + (y - py) ** 2 < (separation + half + _pole_xy_half(pk, ps)) ** 2
                for px, py, pk, ps, _ in placed
            ):
                continue
            if mask is not None:
                window = _pole_pixel_window(x, y, half, mask.shape[0], mask.shape[1], h_scale)
                if window is None:
                    continue
                x1, x2, y1, y2 = window
                cover = _pole_cover_mask(kind, size, x, y, x1, x2, y1, y2, h_scale)
                if not bool(np.any(cover)) or not bool(np.all(mask[x1:x2, y1:y2][cover])):
                    continue
            placed.append((x, y, kind, size, _pole_height_meters(cfg)))
            break
    return placed


def apply_poles_to_height_field(
    cfg: Any, hf_raw: np.ndarray, mask: np.ndarray | None = None
) -> np.ndarray:
    """Stamp cylindrical or square poles onto a height field, sitting on the local surface."""
    width_m, height_m = cfg.size
    rows, cols = hf_raw.shape
    h_scale = float(cfg.horizontal_scale)
    v_scale = float(cfg.vertical_scale)
    placements = _sample_pole_placements(cfg, (width_m, height_m), mask=mask, horizontal_scale=h_scale)
    for x_m, y_m, kind, size, height in placements:
        half = _pole_xy_half(kind, size)
        window = _pole_pixel_window(x_m, y_m, half, rows, cols, h_scale)
        if window is None:
            continue
        x1, x2, y1, y2 = window
        cover = _pole_cover_mask(kind, size, x_m, y_m, x1, x2, y1, y2, h_scale)
        if not bool(np.any(cover)):
            continue
        patch = hf_raw[x1:x2, y1:y2]
        if mask is not None and not bool(np.all(mask[x1:x2, y1:y2][cover])):
            continue
        base = float(np.max(patch[cover]))
        patch[cover] = base + max(int(height / v_scale), 1)
    return hf_raw


def _maybe_apply_poles(
    cfg: Any, hf_raw: np.ndarray, mask: np.ndarray | None = None
) -> np.ndarray:
    """Apply poles according to the configured probability."""
    if not should_apply_poles(cfg):
        return hf_raw
    return apply_poles_to_height_field(cfg, hf_raw, mask=mask)


def _finalize_height_field(
    cfg: Any, hf_raw: np.ndarray, difficulty: float, mask: np.ndarray | None = None
) -> np.ndarray:
    """Apply shared surface details: roughness then optional poles."""
    hf_raw = _maybe_apply_roughness(cfg, hf_raw, difficulty, mask=mask)
    return _maybe_apply_poles(cfg, hf_raw, mask=mask)
