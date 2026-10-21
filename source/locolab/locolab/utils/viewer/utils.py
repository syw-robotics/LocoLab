# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Shared buffer and quaternion helpers for the browser viewer.

Isaac Lab 3.x stores quaternions as ``(x, y, z, w)``. Viser and the URDF
poser in this package use ``(w, x, y, z)``. Articulation buffers are
``ProxyArray`` objects, so callers index ``.torch`` explicitly.
"""

from __future__ import annotations

import numpy as np


def xyzw_to_wxyz(quat):
    """Reorder an Isaac Lab quaternion ``(x, y, z, w)`` to Viser's ``(w, x, y, z)``.

    The input may be a NumPy array or a torch tensor. The component order
    changes and the container type stays the same.
    """
    return quat[..., [3, 0, 1, 2]]


def select_rows(value, selector):
    """Index one articulation buffer.

    A ``ProxyArray`` is read through ``.torch`` before the index. A plain
    tensor or array is indexed directly.
    """
    tensor = value.torch if hasattr(value, "torch") else value
    return tensor[selector]


def as_numpy(value, selector) -> np.ndarray:
    """Index a buffer and return a CPU ``float64`` array."""
    rows = select_rows(value, selector)
    if hasattr(rows, "detach"):
        rows = rows.detach().cpu().numpy()
    return np.asarray(rows, dtype=np.float64)


def rotate_wxyz(wxyz: np.ndarray, vector: np.ndarray) -> np.ndarray:
    """Rotate row vectors by wxyz quaternions."""
    xyz = wxyz[..., 1:]
    turn = 2.0 * np.cross(xyz, vector)
    return vector + wxyz[..., :1] * turn + np.cross(xyz, turn)
