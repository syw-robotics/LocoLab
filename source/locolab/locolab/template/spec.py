# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ProjectSpec:
    """Inputs used to render a LocoLab external project."""

    parent_dir: Path
    project_name: str
    package_name: str
    robot_name: str
    robot_class: str
    task_id: str
    rl_libraries: tuple[str, ...]

    @property
    def dest_dir(self) -> Path:
        return self.parent_dir / self.project_name

    @property
    def experiment_name(self) -> str:
        return f"{self.robot_name}_flat"

    @property
    def agent_cfg_entry_points(self) -> str:
        lines: list[str] = []
        if "z_rl" in self.rl_libraries:
            lines.append(
                f'        "z_rl_cfg_entry_point": f"{{agents.__name__}}.z_rl_ppo_cfg:{self.robot_class}FlatPPORunnerCfg",'
            )
        if "rsl_rl" in self.rl_libraries:
            lines.append(
                f'        "rsl_rl_cfg_entry_point": f"{{agents.__name__}}.rsl_rl_ppo_cfg:{self.robot_class}FlatPPORunnerCfg",'
            )
        return "\n".join(lines)

    @property
    def train_play_commands(self) -> str:
        lines: list[str] = []
        for library in self.rl_libraries:
            lines.append(f"python scripts/{library}/train.py --task={self.task_id}")
            lines.append(f"python scripts/{library}/play.py --task={self.task_id}")
        return "\n".join(lines)

    def replacements(self) -> dict[str, str]:
        return {
            "__PROJECT_NAME__": self.project_name,
            "__PACKAGE_NAME__": self.package_name,
            "__ROBOT_NAME__": self.robot_name,
            "__ROBOT_CLASS__": self.robot_class,
            "__TASK_ID__": self.task_id,
            "__EXPERIMENT_NAME__": self.experiment_name,
            "__AGENT_CFG_ENTRY_POINTS__": self.agent_cfg_entry_points,
            "__TRAIN_PLAY_COMMANDS__": self.train_play_commands,
        }
