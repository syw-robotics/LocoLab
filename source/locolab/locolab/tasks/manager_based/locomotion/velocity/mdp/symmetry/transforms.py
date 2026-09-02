# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Reusable left-right symmetry providers for velocity-task terms."""

from __future__ import annotations

from math import prod

import torch

from locolab.utils.symmetry import MirrorTransform


def identity(*, term_shape, **_) -> MirrorTransform:
    """Keep every component unchanged under left-right reflection."""
    return _sign_transform(term_shape, (1.0,) * prod(term_shape))


def polar_vector(*, term_shape, **_) -> MirrorTransform:
    """Reflect a polar vector as ``[x, -y, z]``."""
    return _sign_transform(term_shape, (1.0, -1.0, 1.0))


def axial_vector(*, term_shape, **_) -> MirrorTransform:
    """Reflect an axial vector as ``[-x, y, -z]``."""
    return _sign_transform(term_shape, (-1.0, 1.0, -1.0))


def velocity_command(*, term_shape, **_) -> MirrorTransform:
    """Reflect ``[forward velocity, lateral velocity, yaw rate]``."""
    return _sign_transform(term_shape, (1.0, -1.0, -1.0))


def joint_space(*, env, term_cfg, **_) -> MirrorTransform:
    """Mirror a joint observation using its resolved names and asset mapping."""
    asset_cfg = term_cfg.params.get("asset_cfg")
    if asset_cfg is None:
        raise ValueError("Joint-space symmetry requires an 'asset_cfg' observation parameter.")
    asset = env.scene[asset_cfg.name]
    joint_names = _selected_names(asset.joint_names, asset_cfg.joint_ids)
    return _joint_name_transform(joint_names, asset.cfg.joint_symmetry_mapping, name="joint observation")


def body_scalar(*, env, term_cfg, **_) -> MirrorTransform:
    """Swap scalar body observations using resolved sensor body names."""
    sensor_cfg = term_cfg.params.get("sensor_cfg")
    if sensor_cfg is None:
        raise ValueError("Body symmetry requires a 'sensor_cfg' observation parameter.")
    sensor = env.scene.sensors[sensor_cfg.name]
    body_names = _selected_names(sensor.body_names, sensor_cfg.body_ids)
    asset = env.scene["robot"]
    return _spatial_name_transform(body_names, asset.cfg.spatial_symmetry_mapping, name="body observation")


def height_scan_y(*, env, term_cfg, **_) -> MirrorTransform:
    """Mirror flattened height-scan samples across the sensor's local y-axis."""
    sensor_cfg = term_cfg.params.get("sensor_cfg")
    if sensor_cfg is None:
        raise ValueError("Height-scan symmetry requires a 'sensor_cfg' observation parameter.")
    sensor = env.scene.sensors[sensor_cfg.name]
    ray_starts = sensor.ray_starts
    if ray_starts.ndim == 3:
        ray_starts = ray_starts[0]
    points = ray_starts[:, :2]

    # Match coordinates instead of reshaping, so GridPatternCfg ordering does not matter.
    mirrored_points = points * points.new_tensor((1.0, -1.0))
    # Direct differences avoid float32 cancellation in torch.cdist near zero.
    distance = (mirrored_points[:, None] - points[None, :]).abs().amax(dim=-1)
    min_distance, index = distance.min(dim=1)
    if min_distance.max().item() > 1.0e-5 or index.unique().numel() != points.shape[0]:
        raise ValueError("Height-scan ray pattern is not uniquely symmetric about its local y-axis.")
    return MirrorTransform.from_sequences(index.cpu().tolist(), [1.0] * points.shape[0])


def joint_action(*, term, **_) -> MirrorTransform:
    """Mirror normalized joint actions and validate their affine conversion."""
    if term is None or not hasattr(term, "_joint_names"):
        raise ValueError("Joint-action symmetry requires a resolved IsaacLab joint action term.")
    transform = _joint_name_transform(
        term._joint_names,
        term._asset.cfg.joint_symmetry_mapping,
        name="joint action",
    )
    _validate_joint_action_affine(term, transform)
    return transform


def last_joint_action(*, env, **_) -> MirrorTransform:
    """Mirror the last action with the active ``joint_pos`` action transform."""
    active_terms = env.action_manager.active_terms
    if active_terms != ["joint_pos"]:
        raise ValueError(
            f"last_joint_action currently requires exactly one 'joint_pos' action term; got {active_terms}."
        )
    return joint_action(term=env.action_manager.get_term("joint_pos"))


# -------------------- Internal helpers -------------------- #


def _sign_transform(term_shape, sign) -> MirrorTransform:
    """Create a mirror transform that applies a sign flip to each component of the term."""
    frame_dim = prod(term_shape)
    return MirrorTransform.from_sequences(range(frame_dim), sign)


def _selected_names(all_names, ids) -> tuple[str, ...]:
    if isinstance(ids, slice):
        return tuple(all_names[ids])
    return tuple(all_names[index] for index in ids)


def _joint_name_transform(names, mapping, *, name: str) -> MirrorTransform:
    if mapping is None:
        raise ValueError(f"Asset does not declare joint_symmetry_mapping for {name}.")
    name_to_index = {name: index for index, name in enumerate(names)}
    indices = []
    signs = []
    for source in names:
        if source not in mapping:
            raise KeyError(f"No joint symmetry mapping for '{source}'.")
        sign, target = mapping[source]
        if target not in name_to_index:
            raise ValueError(f"{name.title()} is not closed: '{source}' maps to absent joint '{target}'.")
        indices.append(name_to_index[target])
        signs.append(float(sign))
    return MirrorTransform.from_sequences(indices, signs)


def _spatial_name_transform(names, mapping, *, name: str) -> MirrorTransform:
    if mapping is None:
        raise ValueError(f"Asset does not declare spatial_symmetry_mapping for {name}.")
    name_to_index = {name: index for index, name in enumerate(names)}
    indices = []
    for source in names:
        if source not in mapping:
            raise KeyError(f"No spatial symmetry mapping for '{source}'.")
        target = mapping[source]
        if target not in name_to_index:
            raise ValueError(f"{name.title()} is not closed: '{source}' maps to absent body '{target}'.")
        indices.append(name_to_index[target])
    return MirrorTransform.from_sequences(indices, [1.0] * len(indices))


def _validate_joint_action_affine(term, transform: MirrorTransform) -> None:
    index = torch.tensor(transform.index, device=term.device)
    sign = torch.tensor(transform.sign, device=term.device)
    scale = _term_vector(term._scale, term.action_dim, term.device)
    offset = _term_vector(term._offset, term.action_dim, term.device)
    if not torch.allclose(scale, scale[index], atol=1.0e-6, rtol=1.0e-6):
        raise ValueError("Joint action scale is not mirror-invariant.")
    if not torch.allclose(offset, offset[index] * sign, atol=1.0e-5, rtol=1.0e-5):
        raise ValueError("Joint action offset/default pose is not mirror-invariant.")


def _term_vector(value, width: int, device) -> torch.Tensor:
    if isinstance(value, (int, float)):
        return torch.full((width,), float(value), device=device)
    tensor = torch.as_tensor(value, device=device)
    if tensor.ndim > 1:
        first = tensor[0]
        if not torch.allclose(tensor, first.expand_as(tensor)):
            raise ValueError("Per-environment action affine parameters are not supported by symmetry.")
        tensor = first
    tensor = tensor.flatten()
    if tensor.numel() != width:
        raise ValueError(f"Action affine parameter has width {tensor.numel()}, expected {width}.")
    return tensor
