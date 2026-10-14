# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from __future__ import annotations

import subprocess
from pathlib import Path

from locolab.template.spec import ProjectSpec

TEMPLATE_ROOT = Path(__file__).resolve().parent / "external_project"

_RL_AGENT_FILES = {
    "z_rl": "z_rl_ppo_cfg.py",
    "rsl_rl": "rsl_rl_ppo_cfg.py",
}
_SKIP_DIR_NAMES = {"__pycache__", ".git"}
_SKIP_SUFFIXES = {".pyc", ".pyo"}


def generate(spec: ProjectSpec, *, git_init: bool = True) -> Path:
    """Render the external project template into ``spec.dest_dir``."""
    dest = spec.dest_dir
    dest.mkdir(parents=True, exist_ok=False)

    replacements = spec.replacements()
    for src in sorted(TEMPLATE_ROOT.rglob("*")):
        if not src.is_file() or _skip_template_file(src):
            continue
        if _skip_rl_library_file(src, spec.rl_libraries):
            continue
        rel = _replace_tokens(str(src.relative_to(TEMPLATE_ROOT)), replacements)
        if rel.endswith(".tmpl"):
            rel = rel[: -len(".tmpl")]
        out_path = dest / rel
        out_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            text = src.read_text(encoding="utf-8")
        except UnicodeDecodeError as error:
            raise ValueError(f"Template file is not UTF-8 text: {src}") from error
        out_path.write_text(_replace_tokens(text, replacements), encoding="utf-8")

    if git_init:
        _git_init(dest)
    return dest


def _skip_template_file(src: Path) -> bool:
    """Ignore bytecode and VCS metadata that can appear beside the templates."""
    if src.suffix in _SKIP_SUFFIXES:
        return True
    return any(part in _SKIP_DIR_NAMES for part in src.relative_to(TEMPLATE_ROOT).parts)


def _skip_rl_library_file(src: Path, rl_libraries: tuple[str, ...]) -> bool:
    name = src.name.removesuffix(".tmpl")
    for library, filename in _RL_AGENT_FILES.items():
        if name == filename and library not in rl_libraries:
            return True
    rel_parts = src.relative_to(TEMPLATE_ROOT).parts
    if len(rel_parts) >= 2 and rel_parts[0] == "scripts" and rel_parts[1] in _RL_AGENT_FILES:
        return rel_parts[1] not in rl_libraries
    return False


def _replace_tokens(text: str, replacements: dict[str, str]) -> str:
    for token, value in replacements.items():
        text = text.replace(token, value)
    return text


def _git_init(dest: Path) -> None:
    try:
        subprocess.run(
            ["git", "init"],
            cwd=dest,
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except (OSError, subprocess.CalledProcessError):
        # git is optional; the project is still usable without a repo.
        pass
