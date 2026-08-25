"""Utilities for LocoLab height-field terrains."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from isaaclab.terrains.height_field import hf_terrains_cfg

    from . import locolab_hf_terrains_cfg


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


def apply_perlin_noise(
    cfg: hf_terrains_cfg.HfRandomUniformTerrainCfg, hf_raw: np.ndarray, difficulty: float = 0.0
) -> np.ndarray:
    if cfg.downsampled_scale is None:
        downsampled_scale = cfg.horizontal_scale
    elif cfg.downsampled_scale < cfg.horizontal_scale:
        raise ValueError(
            "Downsampled scale must be larger than or equal to the horizontal scale:"
            f" {cfg.downsampled_scale} < {cfg.horizontal_scale}."
        )
    else:
        downsampled_scale = cfg.downsampled_scale

    width_pixels = int(cfg.size[0] / cfg.horizontal_scale)
    length_pixels = int(cfg.size[1] / cfg.horizontal_scale)

    roughness_type = getattr(cfg, "roughness_type", "fixed")
    if roughness_type == "difficulty":
        strength = float(np.clip(difficulty, 0.0, 1.0))
    elif roughness_type == "random":
        strength = np.random.choice((0.4, 0.6, 0.8, 1.0))
    elif roughness_type == "fixed":
        strength = 1.0
    else:
        raise ValueError(
            f"Invalid roughness type: {roughness_type}. Must be one of 'difficulty', 'random', or 'fixed'."
        )

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
        return hf_raw

    downsampled_scale *= np.exp2(np.random.random())
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
        return hf_raw
    noise = np.clip(noise, scaled_min, scaled_max).astype(np.int16)

    hf_raw += noise
    return hf_raw


def _maybe_apply_roughness(
    cfg: locolab_hf_terrains_cfg.HfRoughTerrainCfg, hf_raw: np.ndarray, difficulty: float
) -> np.ndarray:
    """Apply roughness according to the configured probability."""
    probability = float(cfg.apply_roughness)
    if not 0.0 <= probability <= 1.0:
        raise ValueError(f"Roughness probability must be within [0, 1]. Got: {probability}.")
    if probability == 0.0 or (probability < 1.0 and np.random.random() >= probability):
        return hf_raw
    return apply_perlin_noise(cfg, hf_raw, difficulty)
