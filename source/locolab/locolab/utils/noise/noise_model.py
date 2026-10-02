# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from __future__ import annotations

import random
from typing import TYPE_CHECKING, Sequence

import torch
import torch.nn.functional as F
from isaacsim.core.utils.torch.maths import torch_rand_float
from torchvision.transforms import GaussianBlur
from locolab.utils.buffers import AsyncDelayBuffer
from .corruption import (
    DEPTH_CORRUPTION_MODE_NAMES,
    apply_active_depth_corruption,
    apply_selected_depth_corruption_modes,
)

if TYPE_CHECKING:
    from .noise_cfg import (
        BlindSpotNoiseCfg,
        CropAndResizeCfg,
        DepthArtifactNoiseCfg,
        DepthContourNoiseCfg,
        DepthCorruptionNoiseCfg,
        DepthNormalizationCfg,
        DepthSkyArtifactNoiseCfg,
        DepthSteroNoiseCfg,
        GaussianBlurNoiseCfg,
        ImageNoiseCfg,
        LatencyNoiseCfg,
        RandomGaussianNoiseCfg,
        RandomSideMaskingCfg,
        RangeBasedGaussianNoiseCfg,
        StereoTooCloseNoiseCfg,
    )


class ImageNoiseModel:
    """This serves as an example of a noise model for images.
    It should be replaced with a specific noise model implementation.
    """

    def __init__(self, cfg: ImageNoiseCfg, num_envs: int = 1, device: str | torch.device = "cpu"):
        """Initialize the noise model with the configuration.

        Args:
            cfg: The configuration for the noise model.
            num_envs: The number of environments (default is 1).
        """
        self.cfg = cfg
        self.num_envs = num_envs
        self.device = device

    def __call__(self, data: torch.Tensor, cfg: ImageNoiseCfg, env_ids: torch.Tensor | Sequence[int]) -> torch.Tensor:
        """Apply noise to the image data.

        Args:
            data: The image data in shape (N, H, W, C).
            cfg: The configuration for the noise model.
            env_ids: The environment IDs for the current image sensor.

        Returns:
            The noisy image data.
        """
        return data

    def reset(self, env_ids: Sequence[int] | None = None):
        """Reset the noise model state if needed.

        Args:
            env_ids: The environment IDs to reset. If None, reset all environments.
        """
        pass


def depth_contour_noise(
    data: torch.Tensor,
    cfg: DepthContourNoiseCfg,
    env_ids: torch.Tensor | Sequence[int],
) -> torch.Tensor:
    """Apply depth contour noise to the image data.

    Args:
        data: The image data in shape (N, H, W, C).
        cfg: The configuration for the depth contour noise.
        env_ids: The environment IDs for the current image sensor.

    Returns:
        The noisy image data with depth contour noise applied.
    """
    device = cfg.device
    # build the contour detection kernel
    contour_detection_kernel = torch.zeros(
        (8, 1, 3, 3),
        dtype=torch.float32,
        device=device,
    )
    # empirical values to be more sensitive to vertical edges
    contour_detection_kernel[0, :, 1, 1] = 0.5
    contour_detection_kernel[0, :, 0, 0] = -0.5
    contour_detection_kernel[1, :, 1, 1] = 0.1
    contour_detection_kernel[1, :, 0, 1] = -0.1
    contour_detection_kernel[2, :, 1, 1] = 0.5
    contour_detection_kernel[2, :, 0, 2] = -0.5
    contour_detection_kernel[3, :, 1, 1] = 1.2
    contour_detection_kernel[3, :, 1, 0] = -1.2
    contour_detection_kernel[4, :, 1, 1] = 1.2
    contour_detection_kernel[4, :, 1, 2] = -1.2
    contour_detection_kernel[5, :, 1, 1] = 0.5
    contour_detection_kernel[5, :, 2, 0] = -0.5
    contour_detection_kernel[6, :, 1, 1] = 0.1
    contour_detection_kernel[6, :, 2, 1] = -0.1
    contour_detection_kernel[7, :, 1, 1] = 0.5
    contour_detection_kernel[7, :, 2, 2] = -0.5

    # Ensure data is in (N, C, H, W) format for conv2d
    original_shape = data.shape
    if data.dim() == 4 and data.shape[-1] == 1:
        # Convert from (N, H, W, C) to (N, C, H, W)
        data = data.permute(0, 3, 1, 2)
    elif data.dim() == 3:
        # Add channel dimension: (N, H, W) -> (N, 1, H, W)
        data = data.unsqueeze(1)

    mask = (
        F.max_pool2d(
            torch.abs(F.conv2d(data, contour_detection_kernel, padding=1)).max(dim=1, keepdim=True)[0],
            kernel_size=cfg.maxpool_kernel_size,
            stride=1,
            padding=int(cfg.maxpool_kernel_size / 2),
        )
        > cfg.contour_threshold
    )

    data = data.clone()
    data[mask] = 0.0

    # Convert back to original shape
    if len(original_shape) == 4 and original_shape[-1] == 1:
        # Convert from (N, C, H, W) back to (N, H, W, C)
        data = data.permute(0, 2, 3, 1)
    elif len(original_shape) == 3:
        # Remove channel dimension: (N, 1, H, W) -> (N, H, W)
        data = data.squeeze(1)

    return data


def depth_artifact_noise(
    data: torch.Tensor,
    cfg: DepthArtifactNoiseCfg,
    env_ids: torch.Tensor | Sequence[int],
) -> torch.Tensor:
    """Add depth artifacts to the image data."""
    return _add_depth_artifacts(
        data,
        artifacts_prob=cfg.artifacts_prob,
        artifacts_height_mean_std=cfg.artifacts_height_mean_std,
        artifacts_width_mean_std=cfg.artifacts_width_mean_std,
        device=cfg.device,
        noise_value=cfg.noise_value,
        noise_value_choices=cfg.noise_value_choices,
    )


def _create_depth_artifact_templates(height: int, width: int, device: str | torch.device):
    """Create the fixed source patch and normalized grid for one image resolution."""
    source_template = torch.zeros((1, 1, 25, 25), device=device)
    source_template[:, :, 1:24, 1:24] = 1.0

    grid_template = torch.zeros((1, height, width, 2), device=device)
    grid_template[..., 0] = torch.linspace(-1, 1, width, device=device).view(1, 1, width)
    grid_template[..., 1] = torch.linspace(-1, 1, height, device=device).view(1, height, 1)
    return source_template, grid_template


class DepthArtifactNoiseModel(ImageNoiseModel):
    """Depth-artifact noise with reusable sampling templates."""

    def __init__(self, cfg: DepthArtifactNoiseCfg, num_envs: int, device: str | torch.device):
        super().__init__(cfg, num_envs, device)
        self._templates: dict[tuple[int, int], tuple[torch.Tensor, torch.Tensor]] = {}

    def __call__(
        self, data: torch.Tensor, cfg: DepthArtifactNoiseCfg, env_ids: torch.Tensor | Sequence[int]
    ) -> torch.Tensor:
        return _add_depth_artifacts(
            data,
            artifacts_prob=cfg.artifacts_prob,
            artifacts_height_mean_std=cfg.artifacts_height_mean_std,
            artifacts_width_mean_std=cfg.artifacts_width_mean_std,
            device=cfg.device,
            noise_value=cfg.noise_value,
            noise_value_choices=cfg.noise_value_choices,
            templates=self._templates,
        )


def depth_stero_noise(
    data: torch.Tensor,
    cfg: DepthSteroNoiseCfg,
    env_ids: torch.Tensor | Sequence[int],
) -> torch.Tensor:
    """Simulate the noise from the depth limit of the stereo camera."""
    device = cfg.device
    N, H, W, _ = data.shape
    far_mask = data > cfg.stero_far_distance
    too_close_mask = data < cfg.stero_min_distance
    near_mask = (~far_mask) & (~too_close_mask)

    # add noise to the fart points
    far_noise = torch_rand_float(0.0, cfg.stero_far_noise_std, (N, H * W), device=device).view(N, H, W, 1)
    far_noise = far_noise * far_mask
    data += far_noise

    # add noise to near points
    near_noise = torch_rand_float(0.0, cfg.stero_near_noise_std, (N, H * W), device=device).view(N, H, W, 1)
    near_noise = near_noise * near_mask
    data += near_noise

    # add artifacts to the too close points
    vertical_block_mask = too_close_mask.sum(dim=-3, keepdim=True) > (too_close_mask.shape[-3] * 0.6)
    """Based on real D435i image pattern, there are two situations when pixels are too close
    Whether there is too-close pixels all the way across the image vertically.
    """

    full_block_mask = vertical_block_mask & too_close_mask
    half_block_mask = (~vertical_block_mask) & too_close_mask
    # add artifacts where vertical pixels are all too close

    for pixel_value in random.sample(
        cfg.stero_full_block_values,
        len(cfg.stero_full_block_values),
    ):
        artifacts_buffer = torch.ones_like(data)
        artifacts_buffer = _add_depth_artifacts(
            artifacts_buffer,
            cfg.stero_full_block_artifacts_prob,
            cfg.stero_full_block_height_mean_std,
            cfg.stero_full_block_width_mean_std,
            device=device,
        )
        data[full_block_mask] = ((1 - artifacts_buffer) * pixel_value)[full_block_mask]
    # add artifacts where not all the same vertical pixels are too close
    half_block_spark = (
        torch_rand_float(0.0, 1.0, (N, H * W), device=device).view(N, H, W, 1) < cfg.stero_half_block_spark_prob
    )
    data[half_block_mask] = (half_block_spark.to(torch.float32) * cfg.stero_half_block_value)[half_block_mask]

    return data


def depth_sky_artifact_noise(
    data: torch.Tensor,
    cfg: DepthSkyArtifactNoiseCfg,
    env_ids: torch.Tensor | Sequence[int],
) -> torch.Tensor:
    """Add sky artifacts to the image data, in case something like ceiling pattern or stereo failure happens."""
    device = cfg.device
    N, _, H, W = data.shape

    possible_to_sky_mask = data > cfg.sky_artifacts_far_distance

    def _recognize_top_down_seeing_sky(too_far_mask):
        N, _, H, W = too_far_mask.shape
        # whether there is too-far pixels with all pixels above it too-far
        num_too_far_above = too_far_mask.cumsum(dim=-2)
        all_too_far_above_threshold = torch.arange(H, device=device).view(1, 1, H, 1)
        all_too_far_above = num_too_far_above > all_too_far_above_threshold  # (N, 1, H, W) mask
        return all_too_far_above

    to_sky_mask = _recognize_top_down_seeing_sky(possible_to_sky_mask)
    isinf_mask = data.isinf()

    # add artifacts to the regions where they are seemingly pointing to sky
    for pixel_value in random.sample(
        cfg.sky_artifacts_values,
        len(cfg.sky_artifacts_values),
    ):
        artifacts_buffer = torch.ones_like(data)
        artifacts_buffer = _add_depth_artifacts(
            artifacts_buffer,
            cfg.sky_artifacts_prob,
            cfg.sky_artifacts_height_mean_std,
            cfg.sky_artifacts_width_mean_std,
            device=device,
        )
        data[to_sky_mask & (~isinf_mask)] *= artifacts_buffer[to_sky_mask & (~isinf_mask)]
        data[to_sky_mask & isinf_mask & (artifacts_buffer < 1)] = 0.0
        data[to_sky_mask] += ((1 - artifacts_buffer) * pixel_value)[to_sky_mask]
        pass

    return data


def depth_normalization(
    data: torch.Tensor, cfg: DepthNormalizationCfg, env_ids: torch.Tensor | Sequence[int]
) -> torch.Tensor:
    """Clip the depth values to given range and choose whether to normalize them to [0, 1] range."""

    if data.dim() == 4 and data.shape[-1] == 1:
        # Convert from (N, H, W, C) to (N, C, H, W)
        data = data.permute(0, 3, 1, 2)

    # Clip depth values to [min_depth, max_depth]
    min_depth = cfg.depth_range[0]
    max_depth = cfg.depth_range[1]
    data = data.clip(min_depth, max_depth)

    if cfg.normalize:
        # Normalize depth values to [0, 1]
        data = (data - min_depth) / (max_depth - min_depth)
        # Normalize to output range
        data = data * (cfg.output_range[1] - cfg.output_range[0]) + cfg.output_range[0]

    if len(data.shape) == 4 and data.shape[1] == 1:
        # Convert back to (N, H, W, C)
        data = data.permute(0, 2, 3, 1)

    return data


class LatencyNoiseModel(ImageNoiseModel):
    def __init__(self, cfg: LatencyNoiseCfg, num_envs, device):
        super().__init__(cfg, num_envs, device)

        # check if the cfg is valid
        if cfg.latency_distribution == "choice" and max(cfg.latency_choices) > cfg.history_length:
            raise RuntimeError(f"Latency choices {cfg.latency_choices} exceed the history length {cfg.history_length}.")
        if cfg.latency_distribution == "constant" and cfg.latency_steps > cfg.history_length:
            raise RuntimeError(f"Latency steps {cfg.latency_steps} exceed the history length {cfg.history_length}.")
        if (cfg.latency_choices == "uniform" or cfg.latency_choices == "normal") and (
            (cfg.latency_range[1] > cfg.history_length)
            or (cfg.latency_range[0] < 0)
            or cfg.latency_range[0] > cfg.latency_range[1]
        ):
            raise RuntimeError(f"Latency range {cfg.latency_range} is invalid.")

        self.delay_buffer = AsyncDelayBuffer(cfg.history_length, num_envs, device)
        self.cfg = cfg
        self.num_envs = num_envs

        # independent counters for each environment
        self.env_step_counters = torch.zeros(num_envs, dtype=torch.int, device=device)
        self.last_resample_steps = torch.zeros(num_envs, dtype=torch.int, device=device)

        # set different resamplpe intervals for each environment
        if cfg.sample_frequency == "every_n_steps":
            self.resample_intervals = self._generate_resample_intervals()

        # initialize the delay settings
        self._resample_delays(torch.arange(num_envs, device=device))

    def __call__(self, data, cfg, env_ids: torch.Tensor | Sequence[int]):
        # convert env_ids to tensor
        if isinstance(env_ids, Sequence):
            env_ids = torch.tensor(env_ids, device=self.device)

        if data.shape[0] != len(env_ids):
            raise RuntimeError(
                f"Data batch shape {data.shape[0]} does not match the number of environments {len(env_ids)}."
            )

        # update step counters
        self.env_step_counters[env_ids] += 1

        # check environments that should resample delays
        should_resample = self._should_resample(env_ids)

        if torch.any(should_resample):
            resample_env_ids = env_ids[should_resample]
            self._resample_delays(resample_env_ids)
            self.last_resample_steps[resample_env_ids] = self.env_step_counters[resample_env_ids]

        # get the delayed data
        delayed = self.delay_buffer.compute(data, batch_ids=env_ids.tolist())
        return delayed

    def _generate_resample_intervals(self, env_ids: Sequence[int] | None = None):
        """Generate resample intervals for each environment"""
        base_interval = self.cfg.sample_frequency_steps
        offset_range = self.cfg.sample_frequency_steps_offset
        if env_ids is None:
            offsets = torch.randint(-offset_range, offset_range + 1, (self.num_envs,), device=self.device)
        else:
            offsets = torch.randint(-offset_range, offset_range + 1, (len(env_ids),), device=self.device)
        intervals = base_interval + offsets
        intervals = intervals.clamp(min=1)  # ensure intervals are at least 1
        return intervals

    def _should_resample(self, env_ids: torch.Tensor):
        """Check which environments should resample delays."""
        if self.cfg.sample_frequency is not None:
            # Sample new delays based on the configured frequency
            if self.cfg.sample_frequency == "every_n_steps":
                # Resample every n steps
                current_steps = self.env_step_counters[env_ids]
                last_resample_steps = self.last_resample_steps[env_ids]
                intervals = self.resample_intervals[env_ids]
                return current_steps - last_resample_steps >= intervals

            elif self.cfg.sample_frequency == "random_with_probability":
                prob = self.cfg.sample_probability
                return torch.rand(len(env_ids), device=self.device) < prob

        # do not resample by default
        return torch.zeros(len(env_ids), dtype=torch.bool, device=self.device)

    def _resample_delays(self, env_ids: torch.Tensor):
        """Resample the delays based on the configured distribution."""

        num_envs_to_resample = len(env_ids)

        if self.cfg.latency_distribution == "uniform":
            # Uniform distribution of delays
            new_delays = torch.randint(
                self.cfg.latency_range[0],
                self.cfg.latency_range[1] + 1,
                (num_envs_to_resample,),
                dtype=torch.int,
                device=self.device,
            )
        elif self.cfg.latency_distribution == "normal":
            new_delays = (
                torch.normal(
                    mean=self.cfg.latency_mean_std[0],
                    std=self.cfg.latency_mean_std[1],
                    size=(num_envs_to_resample,),
                    device=self.device,
                )
                .round()
                .int()
                .clamp(
                    min=self.cfg.latency_range[0],
                    max=self.cfg.latency_range[1],
                )
            )
        elif self.cfg.latency_distribution == "choice":
            # Choose delays from a predefined set
            choices = torch.tensor(self.cfg.latency_choices, dtype=torch.int, device=self.device)
            if self.cfg.latency_choices_probabilities is not None:
                prob = torch.tensor(self.cfg.latency_choices_probabilities, device=self.device)
                indices = torch.multinomial(prob, num_envs_to_resample, replacement=True)
            else:
                indices = torch.randint(0, len(choices), (num_envs_to_resample,), device=self.device)
            new_delays = choices[indices]
        elif self.cfg.latency_distribution == "constant":
            new_delays = torch.full(
                (num_envs_to_resample,), self.cfg.latency_steps, dtype=torch.int, device=self.device
            )

        self.delay_buffer.set_time_lag(new_delays, env_ids.tolist())

    def reset(self, env_ids: Sequence[int] | None = None):
        """reset the noise model state for given environments."""
        if env_ids is None:
            env_ids = list(range(self.num_envs))

        env_ids_tensor = torch.tensor(env_ids, device=self.device)

        # reset the environment step counters
        self.env_step_counters[env_ids_tensor] = 0
        self.last_resample_steps[env_ids_tensor] = 0

        # reset resample intervals if "every_n_steps" is used
        if self.cfg.sample_frequency == "every_n_steps":
            new_intervals = self._generate_resample_intervals(env_ids)
            self.resample_intervals[env_ids_tensor] = new_intervals

        # reset the delay buffer
        self.delay_buffer.reset(env_ids)

        # resample delays for given environments
        self._resample_delays(env_ids_tensor)


class RandomSideMaskingNoiseModel(ImageNoiseModel):
    """Mask randomly sampled image borders, independently for each environment.

    A mask begins at random sensor updates and remains fixed for a short burst.
    This makes the augmentation appear intermittently within an episode
    without flickering during one individual occlusion.
    """

    def __init__(self, cfg: RandomSideMaskingCfg, num_envs: int, device: str | torch.device):
        super().__init__(cfg, num_envs, device)
        if not 0.0 <= cfg.start_probability <= 1.0:
            raise ValueError(f"start_probability must be in [0, 1], got {cfg.start_probability}.")
        if cfg.width_range[0] < 0 or cfg.width_range[1] < cfg.width_range[0]:
            raise ValueError(f"Invalid random side-mask width_range: {cfg.width_range}.")
        if cfg.duration_range[0] < 1 or cfg.duration_range[1] < cfg.duration_range[0]:
            raise ValueError(f"Invalid random side-mask duration_range: {cfg.duration_range}.")
        if len(cfg.mask_value_choices) == 0:
            raise ValueError("random side masking needs at least one mask value.")
        invalid_sides = set(cfg.sides) - {"top", "bottom", "left", "right"}
        if invalid_sides:
            raise ValueError(f"Unsupported random side-mask sides: {sorted(invalid_sides)}.")

        self._widths = {
            side: torch.zeros(num_envs, dtype=torch.long, device=device)
            for side in ("top", "bottom", "left", "right")
        }
        self._mask_values = torch.zeros(num_envs, dtype=torch.float32, device=device)
        self._remaining_steps = torch.zeros(num_envs, dtype=torch.long, device=device)
        self._image_grids: dict[tuple[int, int], tuple[torch.Tensor, torch.Tensor]] = {}

    def _start_bursts(self, env_ids: torch.Tensor) -> None:
        for widths in self._widths.values():
            widths[env_ids] = 0
        if len(env_ids) == 0 or len(self.cfg.sides) == 0:
            return

        selected_side_indices = torch.randint(0, len(self.cfg.sides), (len(env_ids),), device=self.device)
        value_choices = torch.as_tensor(self.cfg.mask_value_choices, device=self.device, dtype=torch.float32)
        choice_indices = torch.randint(0, len(value_choices), (len(env_ids),), device=self.device)
        self._mask_values[env_ids] = value_choices[choice_indices]
        for side, widths in self._widths.items():
            if side not in self.cfg.sides:
                continue
            side_index = self.cfg.sides.index(side)
            selected = selected_side_indices == side_index
            sampled_widths = torch.randint(
                self.cfg.width_range[0], self.cfg.width_range[1] + 1, (len(env_ids),), device=self.device
            )
            widths[env_ids] = sampled_widths * selected
        self._remaining_steps[env_ids] = torch.randint(
            self.cfg.duration_range[0], self.cfg.duration_range[1] + 1, (len(env_ids),), device=self.device
        )

    def __call__(
        self, data: torch.Tensor, cfg: RandomSideMaskingCfg, env_ids: torch.Tensor | Sequence[int]
    ) -> torch.Tensor:
        env_ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        if data.shape[0] != len(env_ids):
            raise RuntimeError(
                f"Data batch shape {data.shape[0]} does not match the number of environments {len(env_ids)}."
            )
        eligible = env_ids[self._remaining_steps[env_ids] == 0]
        starts = eligible[torch.rand(len(eligible), device=self.device) < cfg.start_probability]
        self._start_bursts(starts)
        active = self._remaining_steps[env_ids] > 0

        height, width = data.shape[1:3]
        top = self._widths["top"][env_ids].clamp(max=height).view(-1, 1, 1, 1)
        bottom = self._widths["bottom"][env_ids].clamp(max=height).view(-1, 1, 1, 1)
        left = self._widths["left"][env_ids].clamp(max=width).view(-1, 1, 1, 1)
        right = self._widths["right"][env_ids].clamp(max=width).view(-1, 1, 1, 1)
        row, column = self._get_image_grid(height, width, data.device)
        top_mask = row < top
        bottom_mask = row >= height - bottom
        left_mask = column < left
        right_mask = column >= width - right
        border_mask = top_mask | bottom_mask | left_mask | right_mask
        masked = torch.where(border_mask, self._mask_values[env_ids].view(-1, 1, 1, 1).to(data.dtype), data)
        self._remaining_steps[env_ids[active]] -= 1
        return masked

    def _get_image_grid(self, height: int, width: int, device: torch.device) -> tuple[torch.Tensor, torch.Tensor]:
        """Return cached row/column coordinates for the current camera resolution."""
        key = (height, width)
        if key not in self._image_grids:
            self._image_grids[key] = (
                torch.arange(height, device=device).view(1, height, 1, 1),
                torch.arange(width, device=device).view(1, 1, width, 1),
            )
        return self._image_grids[key]

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        else:
            env_ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        self._remaining_steps[env_ids] = 0
        for widths in self._widths.values():
            widths[env_ids] = 0


class GaussianBlurNoiseModel(ImageNoiseModel):
    """Gaussian blur with kernels cached across camera updates."""

    def __init__(self, cfg: GaussianBlurNoiseCfg, num_envs: int, device: str | torch.device):
        super().__init__(cfg, num_envs, device)
        if cfg.kernel_size <= 0 or cfg.kernel_size % 2 == 0:
            raise ValueError(f"Gaussian blur kernel_size must be a positive odd number, got {cfg.kernel_size}.")
        if cfg.sigma <= 0.0:
            raise ValueError(f"Gaussian blur sigma must be positive, got {cfg.sigma}.")
        self._kernels: dict[tuple[int, torch.dtype], torch.Tensor] = {}

    def __call__(
        self, data: torch.Tensor, cfg: GaussianBlurNoiseCfg, env_ids: torch.Tensor | Sequence[int]
    ) -> torch.Tensor:
        channels = data.shape[-1]
        kernel = self._get_kernel(channels, data.dtype)
        data_nchw = data.permute(0, 3, 1, 2)
        padding = cfg.kernel_size // 2
        padded = F.pad(data_nchw, (padding, padding, padding, padding), mode="reflect")
        blurred = F.conv2d(padded, kernel, groups=channels)
        return blurred.permute(0, 2, 3, 1)

    def _get_kernel(self, channels: int, dtype: torch.dtype) -> torch.Tensor:
        key = (channels, dtype)
        if key not in self._kernels:
            radius = self.cfg.kernel_size // 2
            coordinates = torch.arange(-radius, radius + 1, device=self.device, dtype=dtype)
            weights = torch.exp(-0.5 * (coordinates / self.cfg.sigma).square())
            kernel_2d = torch.outer(weights, weights)
            kernel_2d /= kernel_2d.sum()
            self._kernels[key] = kernel_2d.expand(channels, 1, -1, -1).contiguous()
        return self._kernels[key]


class DepthCorruptionNoiseModel(ImageNoiseModel):
    """Randomly degrade depth frames for selected environments."""

    MODE_NAMES = DEPTH_CORRUPTION_MODE_NAMES

    def __init__(self, cfg: DepthCorruptionNoiseCfg, num_envs: int, device: str | torch.device):
        super().__init__(cfg, num_envs, device)
        if cfg.duration_range[0] < 1 or cfg.duration_range[1] < cfg.duration_range[0]:
            raise ValueError(f"Invalid depth corruption duration_range: {cfg.duration_range}")
        if len(cfg.full_frame_value_choices) == 0:
            raise ValueError("depth corruption needs at least one full-frame replacement value.")
        if len(cfg.strong_artifact_value_choices) == 0:
            raise ValueError("strong artifact corruption needs at least one artifact value.")
        if cfg.strong_noise_std_range[0] < 0.0 or cfg.strong_noise_std_range[1] < cfg.strong_noise_std_range[0]:
            raise ValueError(f"Invalid strong_noise_std_range: {cfg.strong_noise_std_range}")
        if cfg.strong_artifacts_prob < 0.0:
            raise ValueError(f"Invalid strong_artifacts_prob: {cfg.strong_artifacts_prob}")
        if cfg.strong_artifacts_max_blocks < 0:
            raise ValueError(f"Invalid strong_artifacts_max_blocks: {cfg.strong_artifacts_max_blocks}")
        self._validate_int_range("stripe_count_range", cfg.stripe_count_range, minimum=1)
        self._validate_int_range("stripe_thickness_range", cfg.stripe_thickness_range, minimum=1)
        self._validate_int_range("sparkle_count_range", cfg.sparkle_count_range, minimum=1)
        self._validate_float_range("sparkle_radius_range", cfg.sparkle_radius_range, minimum=0.0, strict=True)
        self._validate_float_range(
            "sparkle_aspect_ratio_range", cfg.sparkle_aspect_ratio_range, minimum=0.0, strict=True
        )
        if not 0.0 <= cfg.stripe_angle_range_deg[0] < cfg.stripe_angle_range_deg[1] <= 180.0:
            raise ValueError(f"Invalid stripe_angle_range_deg: {cfg.stripe_angle_range_deg}")
        if not 0.0 <= cfg.stripe_value_range[0] <= cfg.stripe_value_range[1] <= 1.0:
            raise ValueError(f"Invalid stripe_value_range: {cfg.stripe_value_range}")
        if len(cfg.sparkle_value_choices) == 0:
            raise ValueError("sparkle corruption needs at least one replacement value.")
        self._validate_float_range("salt_pepper_amount_range", cfg.salt_pepper_amount_range, minimum=0.0)
        if cfg.salt_pepper_amount_range[1] > 1.0:
            raise ValueError(f"Invalid salt_pepper_amount_range: {cfg.salt_pepper_amount_range}")
        if len(cfg.salt_pepper_value_choices) == 0:
            raise ValueError("salt-and-pepper corruption needs at least one replacement value.")

        probability_by_mode = {
            "full_frame_mask": cfg.full_frame_probability,
            "random_depth_noise": cfg.strong_noise_probability,
            "artifact_patch": cfg.strong_artifact_probability,
            "random_stripe": cfg.stripe_probability,
            "sparkle": cfg.sparkle_probability,
            "salt_pepper": cfg.salt_pepper_probability,
        }
        missing_modes = [name for name in self.MODE_NAMES if name not in probability_by_mode]
        if missing_modes:
            raise ValueError(f"Missing depth corruption probabilities for: {', '.join(missing_modes)}")
        mode_probabilities = torch.tensor(
            [probability_by_mode[name] for name in self.MODE_NAMES],
            dtype=torch.float32,
            device=device,
        )
        if (mode_probabilities < 0.0).any():
            rendered = ", ".join(f"{name}={probability_by_mode[name]}" for name in self.MODE_NAMES)
            raise ValueError(f"Depth corruption mode probabilities must be non-negative: {rendered}.")
        probability_sum = mode_probabilities.sum()
        if probability_sum <= 0.0:
            raise ValueError("At least one depth corruption mode probability must be positive.")
        self.mode_probabilities = mode_probabilities / probability_sum
        self._enabled_mode_indices = tuple(
            index for index, name in enumerate(self.MODE_NAMES) if probability_by_mode[name] > 0.0
        )

        self.remaining_steps = torch.zeros(num_envs, dtype=torch.long, device=device)
        self.enabled_env_mask = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.invalid_mask = torch.zeros(num_envs, dtype=torch.bool, device=device)
        self.corruption_modes = torch.full((num_envs,), -1, dtype=torch.long, device=device)
        self.full_frame_value_choices = torch.tensor(cfg.full_frame_value_choices, device=device)

    def set_enabled_env_mask(self, mask: torch.Tensor) -> None:
        if mask.shape != self.enabled_env_mask.shape:
            raise ValueError(
                f"Depth corruption mask shape {mask.shape} does not match num_envs {self.enabled_env_mask.shape}."
            )
        self.enabled_env_mask[:] = mask.to(device=self.device, dtype=torch.bool)
        enabled = self.enabled_env_mask
        self.remaining_steps.copy_(torch.where(enabled, self.remaining_steps, 0))
        self.invalid_mask.copy_(torch.where(enabled, self.invalid_mask, False))
        self.corruption_modes.copy_(torch.where(enabled, self.corruption_modes, -1))

    def __call__(
        self,
        data: torch.Tensor,
        cfg: DepthCorruptionNoiseCfg,
        env_ids: torch.Tensor | Sequence[int],
    ) -> torch.Tensor:
        if isinstance(env_ids, Sequence):
            env_ids = torch.tensor(env_ids, device=self.device, dtype=torch.long)
        else:
            env_ids = env_ids.to(device=self.device, dtype=torch.long)

        if not cfg.enabled:
            self.invalid_mask[env_ids] = False
            self.remaining_steps[env_ids] = 0
            self.corruption_modes[env_ids] = -1
            return data

        enabled = self.enabled_env_mask[env_ids]
        remaining = self.remaining_steps[env_ids]
        starts = enabled & (remaining <= 0)
        starts &= torch.rand(len(env_ids), device=self.device) < cfg.start_probability

        # Burst state and painting stay on device. Modes with probability 0 are
        # omitted on the host, so an idle step does not sync to test the mask.
        sampled_durations = torch.randint(
            cfg.duration_range[0], cfg.duration_range[1] + 1, (len(env_ids),), device=self.device, dtype=torch.long
        )
        sampled_modes = self._sample_corruption_modes(len(env_ids))
        remaining = torch.where(starts, sampled_durations, remaining)
        modes = self.corruption_modes[env_ids]
        self.corruption_modes[env_ids] = torch.where(starts, sampled_modes, modes)

        active = enabled & (remaining > 0)
        self.invalid_mask[env_ids] = active
        self._apply_active_corruption(data, cfg, active, self.corruption_modes[env_ids])

        remaining = torch.clamp(remaining - 1, min=0)
        remaining = torch.where(enabled, remaining, 0)
        self.remaining_steps[env_ids] = remaining
        stored_modes = self.corruption_modes[env_ids]
        self.corruption_modes[env_ids] = torch.where(remaining <= 0, -1, stored_modes)
        return data

    def _sample_corruption_modes(self, num_modes: int) -> torch.Tensor:
        cumulative_probabilities = self.mode_probabilities.cumsum(dim=0)
        return torch.bucketize(torch.rand(num_modes, device=self.device), cumulative_probabilities)

    def apply_selected_modes(
        self,
        data: torch.Tensor,
        cfg: DepthCorruptionNoiseCfg,
        modes_by_env: torch.Tensor | Sequence[int],
    ) -> torch.Tensor:
        """Preview helper: paint chosen modes without updating burst state.

        Wraps :func:`apply_selected_depth_corruption_modes`. Training uses
        :meth:`__call__` / :meth:`_apply_active_corruption` instead.
        """
        return apply_selected_depth_corruption_modes(data, cfg, modes_by_env)

    def _apply_active_corruption(
        self,
        data: torch.Tensor,
        cfg: DepthCorruptionNoiseCfg,
        active: torch.Tensor,
        modes_by_env: torch.Tensor,
    ) -> None:
        apply_active_depth_corruption(
            data,
            cfg,
            active,
            modes_by_env,
            self.full_frame_value_choices,
            self._enabled_mode_indices,
        )

    @staticmethod
    def _validate_int_range(name: str, value: Sequence[int], minimum: int) -> None:
        if len(value) != 2 or value[0] < minimum or value[1] < value[0]:
            raise ValueError(f"Invalid {name}: {value}")

    @staticmethod
    def _validate_float_range(
        name: str, value: Sequence[float], minimum: float, *, strict: bool = False
    ) -> None:
        if len(value) != 2:
            raise ValueError(f"Invalid {name}: {value}")
        below_minimum = value[0] <= minimum if strict else value[0] < minimum
        if below_minimum or value[1] < value[0]:
            raise ValueError(f"Invalid {name}: {value}")

    def reset(self, env_ids: Sequence[int] | None = None):
        if env_ids is None:
            self.remaining_steps.zero_()
            self.invalid_mask.zero_()
            self.corruption_modes.fill_(-1)
            return
        self.remaining_steps[env_ids] = 0
        self.invalid_mask[env_ids] = False
        self.corruption_modes[env_ids] = -1


def _add_depth_artifacts(
    data,
    artifacts_prob,
    artifacts_height_mean_std,
    artifacts_width_mean_std,
    device,
    noise_value=0.0,
    noise_value_choices=None,
    templates: dict[tuple[int, int], tuple[torch.Tensor, torch.Tensor]] | None = None,
):
    """Simulate artifacts from stereo depth camera. In the final artifacts_mask, where there
    should be an artifacts, the mask is 1.
    """

    N, H, W, _ = data.shape

    def _clip(data, dim):
        return torch.clip(data, 0.0, (H, W)[dim])

    # random patched artifacts
    artifacts_mask = torch_rand_float(0.0, 1.0, (N, H * W), device=device).view(N, H, W) < artifacts_prob
    artifacts_mask = artifacts_mask & (data[:, :, :, 0] > 0.0)
    artifacts_coord = torch.nonzero(artifacts_mask).to(torch.float32)  # (n_, 3) n_ <= N * H * W

    if len(artifacts_coord) == 0:
        return data

    artifacts_size = (
        torch.clip(
            artifacts_height_mean_std[0]
            + torch.randn((artifacts_coord.shape[0],), device=device) * artifacts_height_mean_std[1],
            0.0,
            H,
        ),
        torch.clip(
            artifacts_width_mean_std[0]
            + torch.randn((artifacts_coord.shape[0],), device=device) * artifacts_width_mean_std[1],
            0.0,
            W,
        ),
    )  # (n_,), (n_,)

    artifacts_top = _clip(artifacts_coord[:, 1] - artifacts_size[0] / 2, 0)
    artifacts_left = _clip(artifacts_coord[:, 2] - artifacts_size[1] / 2, 1)
    artifacts_bottom = _clip(artifacts_coord[:, 1] + artifacts_size[0] / 2, 0)
    artifacts_right = _clip(artifacts_coord[:, 2] + artifacts_size[1] / 2, 1)

    # Each artifact belongs to one environment. Accumulate them directly on
    # that dimension instead of materializing an (num_artifacts, N) one-hot
    # tensor and contracting it with einsum.
    env_ids = artifacts_coord[:, 0].long()

    # batch generate all artifacts
    num_artifacts = len(artifacts_coord)
    tops_expanded = artifacts_top[:, None, None]
    lefts_expanded = artifacts_left[:, None, None]
    bottoms_expanded = artifacts_bottom[:, None, None]
    rights_expanded = artifacts_right[:, None, None]

    if templates is None:
        source_template, grid_template = _create_depth_artifact_templates(H, W, device)
    else:
        template_key = (H, W)
        if template_key not in templates:
            templates[template_key] = _create_depth_artifact_templates(H, W, device)
        source_template, grid_template = templates[template_key]

    source_patch = source_template.expand(num_artifacts, -1, -1, -1)
    grid = grid_template.expand(num_artifacts, -1, -1, -1).clone()
    grid[..., 0] = (grid[..., 0] * W + W - rights_expanded - lefts_expanded) / (rights_expanded - lefts_expanded)
    grid[..., 1] = (grid[..., 1] * H + H - bottoms_expanded - tops_expanded) / (bottoms_expanded - tops_expanded)

    # sample using the grid and form the artifacts for the entire depth image
    all_artifacts = F.grid_sample(
        source_patch, grid, mode="bilinear", padding_mode="zeros", align_corners=False
    ).squeeze(
        1
    )  # (num_artifacts, H, W)

    # Combine artifacts belonging to the same environment. ``index_add_`` has
    # the same additive overlap semantics as the previous einsum expression.
    raw_masks = torch.zeros((N, H, W), device=device, dtype=all_artifacts.dtype)
    raw_masks.index_add_(0, env_ids, all_artifacts)
    final_masks = torch.clamp(raw_masks, 0, 1)

    if noise_value_choices is None:
        artifact_values = torch.full_like(final_masks, noise_value)
    else:
        choices = torch.as_tensor(noise_value_choices, device=device, dtype=data.dtype)
        if choices.numel() == 0:
            raise ValueError("`noise_value_choices` must contain at least one value.")
        choice_ids = torch.randint(0, choices.numel(), (num_artifacts,), device=device)
        artifact_values_per_artifact = choices[choice_ids]
        artifact_values = torch.zeros_like(raw_masks)
        artifact_values.index_add_(0, env_ids, all_artifacts * artifact_values_per_artifact[:, None, None])
        artifact_values = torch.where(raw_masks > 0, artifact_values / torch.clamp(raw_masks, min=1.0e-6), raw_masks)

    data = data.squeeze(-1)  # (N, H, W)
    data = data * (1 - final_masks) + final_masks * artifact_values
    data = data.unsqueeze(-1)

    return data


def crop_and_resize(
    data: torch.Tensor,
    cfg: CropAndResizeCfg,
    env_ids: torch.Tensor | Sequence[int],
) -> torch.Tensor:
    """Crop and resize the input image tensor."""
    # Crop the image
    crop_region = cfg.crop_region
    start_up = crop_region[0]
    end_down = data.shape[1] - crop_region[1]
    start_left = crop_region[2]
    end_right = data.shape[2] - crop_region[3]
    cropped = data[:, start_up:end_down, start_left:end_right, :]
    # Resize the image
    if cfg.resize_shape is None:
        return cropped
    else:
        cropped = cropped.permute(0, 3, 1, 2)
        resized = F.interpolate(cropped, size=cfg.resize_shape, mode="bilinear", align_corners=False)
        resized = resized.permute(0, 2, 3, 1)
        return resized


def blind_spot_noise(
    data: torch.Tensor,
    cfg: BlindSpotNoiseCfg,
    env_ids: torch.Tensor | Sequence[int],
) -> torch.Tensor:
    """Remove data in the leftmost columns to mimic blind spot of stereo-matching."""
    # Crop the image
    crop_region = cfg.crop_region
    start_up = crop_region[0]
    end_down = data.shape[1] - crop_region[1]
    start_left = crop_region[2]
    end_right = data.shape[2] - crop_region[3]
    # Set the cropped region to 0
    data_modified = data.clone()
    data_modified[:, :start_up, :, :] = 0.0
    data_modified[:, end_down:, :, :] = 0.0
    data_modified[:, :, :start_left, :] = 0.0
    data_modified[:, :, end_right:, :] = 0.0
    return data_modified


def gaussian_blur_noise(
    data: torch.Tensor,
    cfg: GaussianBlurNoiseCfg,
    env_ids: torch.Tensor | Sequence[int],
) -> torch.Tensor:
    """Apply Gaussian blur to the image data."""
    # Convert from (N, H, W, C) to (N, C, H, W)
    data = data.permute(0, 3, 1, 2)
    # Create GaussianBlur transform
    blur_transform = GaussianBlur(kernel_size=cfg.kernel_size, sigma=cfg.sigma)
    # Apply Gaussian blur
    blurred = blur_transform(data)
    # Convert from (N, C, H, W) back to (N, H, W, C)
    blurred = blurred.permute(0, 2, 3, 1)
    return blurred


def random_gaussian_noise(
    data: torch.Tensor,
    cfg: RandomGaussianNoiseCfg,
    env_ids: torch.Tensor | Sequence[int],
) -> torch.Tensor:
    """Apply random Gaussian noise to the image data."""
    N, H, W, C = data.shape
    noise = torch.randn((N, H, W, C), device=cfg.device) * cfg.noise_std + cfg.noise_mean
    if random.random() < cfg.probability:
        noisy_data = data + noise
    else:
        noisy_data = data

    return noisy_data


def _recognize_top_down_too_close(too_close_mask):
    """Based on real D435i image pattern, there are two situations when pixels are too close
    Whether there is too-close pixels all the way across the image vertically.
    """
    # vertical_all_too_close = too_close_mask.all(dim= 2, keepdim= True)
    vertical_too_close = too_close_mask.sum(dim=-3, keepdim=True) > (too_close_mask.shape[-3] * 0.6)
    return vertical_too_close


def range_based_gaussian_noise(
    data: torch.Tensor,
    cfg: RangeBasedGaussianNoiseCfg,
    env_ids: torch.Tensor | Sequence[int],
) -> torch.Tensor:
    """Apply gaussian noise to the data where the original value is in the range [min_value, max_value]
    if min_value or max_value is None, the boundary is not considered.
    """
    N, H, W, C = data.shape
    noise = torch.randn((N, H, W, C), device=data.device) * cfg.noise_std

    apply_mask = torch.ones((N, H, W, C), device=data.device, dtype=bool)
    if cfg.min_value is not None:
        apply_mask = apply_mask & (data >= cfg.min_value)
    if cfg.max_value is not None:
        apply_mask = apply_mask & (data <= cfg.max_value)

    noisy_data = data + noise * apply_mask

    return noisy_data


def stereo_too_close_noise(
    data: torch.Tensor,
    cfg: StereoTooCloseNoiseCfg,
    env_ids: torch.Tensor | Sequence[int],
) -> torch.Tensor:
    """Apply noise to the data where the original value is too close to the stereo camera."""
    N, H, W, C = data.shape
    too_close_mask = data < cfg.close_threshold
    vertical_block_mask = _recognize_top_down_too_close(too_close_mask)
    full_block_mask = vertical_block_mask & too_close_mask
    half_block_mask = (~vertical_block_mask) & too_close_mask

    # add artifacts where vertical pixels are all too close (full block)
    for pixel_value in random.sample(
        cfg.full_block_values,
        len(cfg.full_block_values),
    ):
        artifacts_buffer = torch.ones_like(data)
        artifacts_buffer = _add_depth_artifacts(
            artifacts_buffer,
            cfg.full_block_artifacts_prob,
            cfg.full_block_height_mean_std,
            cfg.full_block_width_mean_std,
            device="cuda" if data.device.type == "cuda" else "cpu",
        )
        data[full_block_mask] = ((1 - artifacts_buffer) * pixel_value)[full_block_mask]

    # add artifacts where not all the same vertical pixels are too close (half block)
    half_block_spark = (
        torch_rand_float(
            0.0,
            1.0,
            (N, H * W),
            device="cuda" if data.device.type == "cuda" else "cpu",
        ).view(N, H, W, 1)
        < cfg.half_block_spark_prob
    )
    data[half_block_mask] = (half_block_spark.to(torch.float32) * cfg.half_block_value)[half_block_mask]

    return data
