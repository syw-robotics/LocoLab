# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""
Script to print all the available environments in Loco Lab.

The script imports ``locolab.tasks`` (which also loads installed ``locolab.tasks``
plugins) and prints registered Gym environments that are not Isaac Lab built-ins.
"""

import gymnasium as gym
from prettytable import PrettyTable

import locolab.tasks  # noqa: F401


def main():
    """Print all registered environments"""
    table = PrettyTable(["S. No.", "Task Name", "Entry Point", "Config"])
    table.title = "Available Environments in Loco Lab"
    table.align["Task Name"] = "l"
    table.align["Entry Point"] = "l"
    table.align["Config"] = "l"

    index = 0
    for task_spec in gym.registry.values():
        if task_spec.id.startswith("Isaac-"):
            continue
        env_cfg = (task_spec.kwargs or {}).get("env_cfg_entry_point")
        if not env_cfg:
            continue
        table.add_row([index + 1, task_spec.id, task_spec.entry_point, env_cfg])
        index += 1

    print(table)


if __name__ == "__main__":
    main()
