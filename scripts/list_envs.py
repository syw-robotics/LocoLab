# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""
Script to print all the available environments in Loco Lab.

The script iterates over all registered environments and stores the details in a table.
It prints the name of the environment, the entry point and the config file.

All the environments are registered in `locolab`.
"""

import importlib
import pathlib
import pkgutil
import sys

SKIP_MODULE_PATTERNS = [
    "mdp_cfg",
]


def _walk_packages(
    path: str | None = None,
    prefix: str = "",
    onerror=None,
):
    """Yields ModuleInfo for all modules recursively on path, or, if path is None, all accessible modules.

    Note:
        This function is a modified version of the original ``pkgutil.walk_packages`` function. Please refer to the original
        ``pkgutil.walk_packages`` function for more details.
    """

    def seen(p, m={}):
        if p in m:
            return True
        m[p] = True  # noqa: R503

    for info in pkgutil.iter_modules(path, prefix):

        if any(pattern in info.name for pattern in SKIP_MODULE_PATTERNS):
            continue

        # yield the module info
        yield info

        if info.ispkg:
            try:
                __import__(info.name)
            except Exception:
                if onerror is not None:
                    onerror(info.name)
                else:
                    raise
            else:
                path = getattr(sys.modules[info.name], "__path__", None) or []

                # don't traverse path items we've seen before
                path = [p for p in path if not seen(p)]

                yield from _walk_packages(path, info.name + ".", onerror)


def import_packages():
    sys.path.insert(0, f"{pathlib.Path(__file__).parent.parent}/source/locolab/locolab/tasks/")
    for package in ["manager_based.locomotion.velocity.config"]:
        package = importlib.import_module(package)
        for _ in _walk_packages(package.__path__, package.__name__ + "."):
            pass
    sys.path.pop(0)


import_packages()

"""Rest everything follows."""

import gymnasium as gym
from prettytable import PrettyTable


def main():
    """Print all registered environments"""
    # print all the available environments
    table = PrettyTable(["S. No.", "Task Name", "Entry Point", "Config"])
    table.title = "Available Environments in Loco Lab"
    # set alignment of table columns
    table.align["Task Name"] = "l"
    table.align["Entry Point"] = "l"
    table.align["Config"] = "l"

    # count of environments
    index = 0
    # acquire all Isaac environments names
    for task_spec in gym.registry.values():
        if "Velocity" in task_spec.id and "Isaac" not in task_spec.id:
            # add details to table
            table.add_row([index + 1, task_spec.id, task_spec.entry_point, task_spec.kwargs["env_cfg_entry_point"]])
            # increment count
            index += 1

    print(table)


if __name__ == "__main__":
    try:
        # run the main function
        main()
    except Exception as e:
        raise e
