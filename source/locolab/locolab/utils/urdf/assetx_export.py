# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Cached URDF/MJCF → USD through assetx. See README.md beside this file."""

from __future__ import annotations

import hashlib
import importlib.util
import logging
import re
import subprocess
import sys
from pathlib import Path

logger = logging.getLogger(__name__)

_CACHE_DIR = ".locolab_usd_cache"
_CONVERT = Path(__file__).with_name("convert.py")
_USD = {".usd", ".usda", ".usdc"}
_MJCF = {".xml", ".mjcf"}


def assetx_export_usd(
    source_path: str | Path,
    *,
    merge_fixed_joints: bool = True,
    replace_cylinders_with_capsules: bool = True,
    fix_base: bool = False,
    keep_fixed_pattern: str | None = None,
    honor_dont_collapse: bool = True,
) -> Path:
    """Return a cached USD path, converting with assetx when the cache is stale."""
    source = Path(source_path).resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    if importlib.util.find_spec("assetx") is None:
        raise ModuleNotFoundError(
            "assetx is missing. It is a locolab dependency from "
            "git+https://github.com/syw-robotics/assetx.git (see README)."
        )

    cli = [] if source.suffix.lower() in _MJCF else _urdf_cli(
        merge_fixed_joints=merge_fixed_joints,
        replace_cylinders_with_capsules=replace_cylinders_with_capsules,
        fix_base=fix_base,
        keep_fixed_pattern=keep_fixed_pattern,
        honor_dont_collapse=honor_dont_collapse,
    )
    key = _cache_key(source, cli)
    cache = source.parent / _CACHE_DIR / key
    stamp = cache / "stamp"
    hit = _find_usd(cache)
    if hit is not None and stamp.is_file() and stamp.read_text(encoding="utf-8") == key:
        return hit

    cache.mkdir(parents=True, exist_ok=True)
    cmd = [sys.executable, str(_CONVERT), str(source), "-o", str(cache), *cli]
    logger.info("assetx: %s -> %s", source, cache)
    subprocess.run(cmd, check=True)

    usd = _find_usd(cache)
    if usd is None:
        raise RuntimeError(f"assetx wrote no USD under {cache / 'usd'}")
    stamp.write_text(key, encoding="utf-8")
    return usd


def _urdf_cli(
    *,
    merge_fixed_joints: bool,
    replace_cylinders_with_capsules: bool,
    fix_base: bool,
    keep_fixed_pattern: str | None,
    honor_dont_collapse: bool,
) -> list[str]:
    cli: list[str] = []
    if not merge_fixed_joints:
        cli.append("--no-merge-fixed")
    if keep_fixed_pattern:
        cli += ["--keep-fixed", keep_fixed_pattern]
    if not honor_dont_collapse:
        cli.append("--no-honor-dont-collapse")
    if not replace_cylinders_with_capsules:
        cli.append("--no-replace-cylinders")
    if fix_base:
        cli.append("--fix-base")
    return cli


def _find_usd(cache: Path) -> Path | None:
    usd_dir = cache / "usd"
    if not usd_dir.is_dir():
        return None
    files = [p for p in usd_dir.iterdir() if p.suffix in _USD]
    return max(files, key=lambda p: p.stat().st_mtime) if files else None


def _cache_key(source: Path, cli: list[str]) -> str:
    h = hashlib.sha256()
    h.update("\0".join(cli).encode())
    data = source.read_bytes()
    h.update(data)
    for mesh in _meshes(source, data.decode("utf-8", errors="replace")):
        h.update(str(mesh).encode())
        if mesh.is_file():
            st = mesh.stat()
            h.update(f"{st.st_mtime_ns}:{st.st_size}".encode())
    st = _CONVERT.stat()
    h.update(f"convert.py:{st.st_mtime_ns}:{st.st_size}".encode())
    try:
        import assetx

        root = Path(assetx.__file__).resolve().parent
        for path in sorted(root.rglob("*.py")):
            st = path.stat()
            h.update(f"{path.relative_to(root)}:{st.st_mtime_ns}:{st.st_size}".encode())
    except ImportError:
        pass
    return h.hexdigest()[:16]


def _meshes(source: Path, text: str) -> list[Path]:
    if source.suffix.lower() in _MJCF:
        match = re.search(r'meshdir="([^"]*)"', text)
        base = source.parent / (match.group(1) if match else "")
        return [base / name for name in re.findall(r'\bfile="([^"]+)"', text)]

    out: list[Path] = []
    for name in re.findall(r'filename="([^"]+)"', text):
        path = _urdf_mesh(source, name)
        if path is not None:
            out.append(path)
    return out


def _urdf_mesh(urdf: Path, filename: str) -> Path | None:
    if filename.startswith("package://"):
        _, _, rel = filename.removeprefix("package://").partition("/")
        for parent in (urdf.parent, *urdf.parent.parents):
            candidate = parent / rel
            if candidate.is_file():
                return candidate
        return None
    path = Path(filename)
    if not path.is_absolute():
        path = urdf.parent / path
    return path.resolve() if path.is_file() else None
