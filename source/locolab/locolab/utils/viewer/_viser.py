# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Import Viser against the environment's websockets, not Isaac Sim's prebundle.

Kit's fast importer remembers ``websockets.http11`` from ``pip_prebundle``. That
copy has no ``SERVER``, while Viser loads ``websockets.asyncio`` from the
environment. Drop the cached prebundle modules before importing Viser, then put
the prebundle path back so the rest of Kit is unchanged.
"""

from __future__ import annotations

import importlib
import sys


def _is_websockets(name: str) -> bool:
    return name == "websockets" or name.startswith("websockets.")


def _from_prebundle(path: object) -> bool:
    return "pip_prebundle" in str(path).replace("\\", "/")


def _drop_fast_importer_cache() -> None:
    """Forget prebundle websockets entries stashed by ``omni.ext`` fast importer."""
    try:
        from omni.ext._impl import fast_importer
    except ImportError:
        return
    cache = getattr(fast_importer, "_cache", None)
    modules = getattr(cache, "modules", None)
    if isinstance(modules, dict):
        for name, path in list(modules.items()):
            if _is_websockets(name) and _from_prebundle(path):
                del modules[name]
    namespaces = getattr(cache, "namespaces", None)
    if isinstance(namespaces, dict):
        for name, paths in list(namespaces.items()):
            if not _is_websockets(name):
                continue
            kept = {path for path in paths if not _from_prebundle(path)}
            if kept:
                namespaces[name] = kept
            else:
                del namespaces[name]


def _drop_prebundle_modules() -> None:
    for name in list(sys.modules):
        if not _is_websockets(name):
            continue
        module = sys.modules[name]
        origin = getattr(module, "__file__", "") or ""
        search = getattr(module, "__path__", ())
        if _from_prebundle(origin) or any(_from_prebundle(path) for path in search):
            del sys.modules[name]


_prebundle = [path for path in sys.path if _from_prebundle(path)]
for _path in _prebundle:
    sys.path.remove(_path)
_drop_fast_importer_cache()
_drop_prebundle_modules()
importlib.invalidate_caches()
try:
    import viser
    from viser.extras import ViserUrdf
finally:
    sys.path[:0] = _prebundle

__all__ = ["ViserUrdf", "viser"]
