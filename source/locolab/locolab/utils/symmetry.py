# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Declarations and functions for symmetry functionality，which is later used for rollout buffer symmetric data augmentation."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass

from isaaclab.managers import ObservationTermCfg
from isaaclab.utils import configclass


@dataclass(frozen=True)
class MirrorTransform:
    """Local flat-tensor transform returned by an observation/action provider."""

    index: tuple[int, ...]
    sign: tuple[float, ...]

    @classmethod
    def from_sequences(cls, index: Sequence[int], sign: Sequence[float]) -> MirrorTransform:
        return cls(tuple(index), tuple(sign))


SymmetryProvider = Callable[..., MirrorTransform]


@configclass
class SymmetricObservationTermCfg(ObservationTermCfg):
    """Observation term with an optional runtime symmetry provider."""

    symmetry_transform: SymmetryProvider | None = None


def complete_symmetry_mapping(mapping: Mapping) -> dict:
    """Return a full mapping config with only necessary manual effort.
    
    Example:
        input: { "left_hip_roll_joint": (-1.0, "right_hip_roll_joint") }
        output: { "left_hip_roll_joint": (-1.0, "right_hip_roll_joint"),
                  "right_hip_roll_joint": (-1.0, "left_hip_roll_joint") }
    This function completes the symmetry mapping by adding reverse entries for each provided mapping.
    If a reverse entry already exists and conflicts with the provided mapping, a ValueError is raised.
    """
    completed = dict(mapping)
    for source, value in mapping.items():
        if isinstance(value, tuple):
            sign, target = value
            reverse = (sign, source)
        elif isinstance(value, str):
            target = value
            reverse = source
        else:
            raise TypeError(f"Invalid symmetry mapping for '{source}': {value!r}.")
        if target in completed and completed[target] != reverse:
            raise ValueError(
                f"Conflicting symmetry mapping: '{source}' maps to '{target}', but "
                f"the reverse entry is {completed[target]!r}."
            )
        completed[target] = reverse
    return completed
