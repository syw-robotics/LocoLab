# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Package containing task implementations for various robotic environments."""

from __future__ import annotations

import sys


def _cli_selects_newton(argv: list[str]) -> bool:
    """Return whether this process asked for a Newton physics preset.

    Importing the Isaac Sim URDF helper registers ``physxSchemaFallback``. OvPhysX then
    fails when it registers the real ``physxSchema`` plugin for the same types. Newton URDF
    conversion still needs those codeless schemas before the first USD lookup.
    """
    newton_presets = {"newton", "newton_mjwarp", "newton_kamino", "kamino"}
    selected_physics: list[str] = []
    selected_presets: list[str] = []
    for arg in argv:
        key, separator, value = arg.partition("=")
        if not separator:
            continue
        names = [item.strip() for item in value.split(",") if item.strip()]
        key = key.removeprefix("--")
        if key == "physics":
            selected_physics.extend(names)
        elif key == "presets":
            selected_presets.extend(names)
    if any(name in newton_presets for name in selected_physics):
        return True
    return not selected_physics and any(name in newton_presets for name in selected_presets)


if _cli_selects_newton(sys.argv[1:]):
    from isaacsim.asset.importer.urdf import URDFImporter as _URDFImporter

    del _URDFImporter

from isaaclab_tasks.utils import import_packages

##
# Register Gym environments.
##


# The blacklist is used to prevent importing configs from sub-packages
_BLACKLIST_PKGS = ["utils"]
# Import all configs in this package
import_packages(__name__, _BLACKLIST_PKGS)


def _import_task_plugins() -> None:
    """Load Gym tasks from installed packages that declare ``locolab.tasks`` entry points."""
    from importlib.metadata import entry_points

    for ep in entry_points(group="locolab.tasks"):
        module_name = ep.value.split(":", 1)[0]
        # Skip this package (already imported above) and anything already loaded.
        if module_name == __name__ or module_name in sys.modules:
            continue
        ep.load()


_import_task_plugins()
