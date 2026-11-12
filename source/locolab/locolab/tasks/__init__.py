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
