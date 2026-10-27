# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Cached URDF paths for Isaac spawn."""

from __future__ import annotations

import hashlib
import re
from pathlib import Path

from .merge_fixed_joints import write_merged_urdf

_DEFAULT_KEEP_FOOT_FIXED = r".*_foot_joint$"
_CACHE_DIR = ".locolab_urdf_cache"


def urdf_for_spawn(
    source_urdf: str | Path,
    *,
    cache_dir: str | Path | None = None,
    keep_fixed_pattern: str = _DEFAULT_KEEP_FOOT_FIXED,
) -> str:
    """Return a URDF path for ``UrdfFileCfg(..., merge_fixed_joints=False)``.

    Rotor/sensor fixed chains are merged in Python; joints matching *keep_fixed_pattern*
    (default: quadruped ``*_foot_joint``) stay fixed. Output is cached beside the source URDF.
    """
    source = Path(source_urdf).resolve()
    keep = re.compile(keep_fixed_pattern)
    cache_root = Path(cache_dir).resolve() if cache_dir is not None else source.parent / _CACHE_DIR
    tag = hashlib.sha256(keep_fixed_pattern.encode()).hexdigest()[:8]
    cached = cache_root / f"{source.stem}_spawn_{tag}.urdf"
    stamp = cache_root / f"{source.stem}_spawn_{tag}.stamp"

    cache_root.mkdir(parents=True, exist_ok=True)
    mtime = str(source.stat().st_mtime_ns)
    if cached.is_file() and stamp.is_file() and stamp.read_text(encoding="utf-8") == mtime:
        return str(cached)

    write_merged_urdf(source, cached, keep_fixed=keep)
    stamp.write_text(mtime, encoding="utf-8")
    return str(cached)
