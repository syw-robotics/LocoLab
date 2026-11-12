"""Observation metadata shared by manager-based tasks and analytic models."""

import math

import torch
from isaaclab.utils.noise import UniformNoiseCfg


def uniform_noise_scales(manager, group, widths, device):
    """Read per-dimension scales a for additive U(-a, a) noise, in manager term order."""
    configs = manager._group_obs_term_cfgs[group]
    dimensions = manager.group_obs_term_dim[group]
    if len(configs) != len(widths):
        raise ValueError(f"Observation widths do not match group {group!r}")
    bounds = []
    for name, cfg, dimension, width in zip(manager.active_terms[group], configs, dimensions, widths):
        history = max(1, cfg.history_length)
        if math.prod(dimension) != history * width:
            raise ValueError(f"Unexpected observation width for {group}/{name}")
        if cfg.clip is not None or cfg.scale is not None:
            raise ValueError(f"Analytic observations must implement clip/scale for {group}/{name}")
        noise = cfg.noise
        if noise is None:
            bounds.append(torch.zeros(width, device=device))
            continue
        if not isinstance(noise, UniformNoiseCfg) or noise.operation != "add":
            raise ValueError(f"Expected additive uniform noise for {group}/{name}")
        low = torch.as_tensor(noise.n_min, device=device).expand(width)
        high = torch.as_tensor(noise.n_max, device=device).expand(width)
        if not torch.equal(low, -high):
            raise ValueError(f"Expected symmetric uniform noise for {group}/{name}")
        bounds.append(high)
    return torch.cat(bounds)
