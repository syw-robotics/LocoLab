# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from __future__ import annotations

import argparse
import keyword
import os
import sys
from pathlib import Path

try:
    import readline  # noqa: F401  # enable arrow keys / line editing for input()
except ImportError:
    pass

from locolab.template.render import generate
from locolab.template.spec import ProjectSpec

_RL_CHOICES = ("z_rl", "rsl_rl", "both")
_BUILTIN_TASK_IDS = {
    "Velocity-Flat-Go2",
    "Velocity-Rough-Go2",
    "Velocity-Flat-G1",
    "Velocity-Flat-B2",
}


def _use_color(stream) -> bool:
    if os.environ.get("NO_COLOR"):
        return False
    return hasattr(stream, "isatty") and stream.isatty()


def _paint(text: str, *codes: str, stream=sys.stdout, readline_prompt: bool = False) -> str:
    if not codes or not _use_color(stream):
        return text
    start = "".join(codes)
    reset = "\033[0m"
    if readline_prompt:
        # Readline counts prompt bytes unless non-printing sequences are wrapped.
        start = f"\001{start}\002"
        reset = f"\001{reset}\002"
    return f"{start}{text}{reset}"


def _bold(text: str, **kwargs) -> str:
    return _paint(text, "\033[1m", **kwargs)


def _dim(text: str, **kwargs) -> str:
    return _paint(text, "\033[2m", **kwargs)


def _cyan(text: str, **kwargs) -> str:
    return _paint(text, "\033[36m", **kwargs)


def _green(text: str, **kwargs) -> str:
    return _paint(text, "\033[32m", **kwargs)


def _yellow(text: str, **kwargs) -> str:
    return _paint(text, "\033[33m", **kwargs)


def _red(text: str, stream=sys.stderr) -> str:
    return _paint(text, "\033[31m", stream=stream)


def _magenta(text: str, **kwargs) -> str:
    return _paint(text, "\033[35m", **kwargs)


def main(argv: list[str] | None = None) -> int:
    """Run the interactive external-project generator."""
    parser = argparse.ArgumentParser(description="Generate a LocoLab external project.")
    parser.parse_args(argv)

    _print_banner()
    try:
        spec = _prompt_spec()
        dest = generate(spec)
    except (EOFError, KeyboardInterrupt):
        print(f"\n{_yellow('Cancelled.')}")
        return 130
    except (OSError, ValueError) as error:
        print(_red(f"Error: {error}"), file=sys.stderr)
        return 1

    _print_next_steps(spec, dest)
    return 0


def _print_banner() -> None:
    bar = _cyan("━" * 56)
    print()
    print(bar)
    print(_bold(_cyan("  LocoLab")) + _dim("  ·  external project generator"))
    print(bar)
    print()
    print(_dim("  Isaac Lab  →  LocoLab  →  your project"))
    print(_dim("  Values in [brackets] are defaults. Ctrl-C to cancel."))
    print()


def _prompt_spec() -> ProjectSpec:
    parent_dir = _prompt_path(
        "Parent directory",
        default=Path.cwd(),
        hint="Must be outside the LocoLab repository. The project is created as <parent>/<project>.",
    )
    project_name = _prompt_project_name()
    package_name = _prompt_package_name(project_name)
    robot_name = _prompt_cfg_name()
    rl_libraries = _prompt_rl_libraries()

    dest = parent_dir / project_name
    _ensure_external_path(dest)
    if dest.exists():
        raise FileExistsError(f"Destination already exists: {dest}")

    robot_class = _to_pascal(robot_name)
    spec = ProjectSpec(
        parent_dir=parent_dir,
        project_name=project_name,
        package_name=package_name,
        robot_name=robot_name,
        robot_class=robot_class,
        task_id=f"Velocity-Flat-{robot_class}",
        rl_libraries=rl_libraries,
    )
    _print_summary(spec)
    if not _prompt_yes_no("Create this project?", default=True):
        raise KeyboardInterrupt
    return spec


def _prompt_project_name() -> str:
    print(_dim("  Folder / repository name. Not the Python package or Gym task id."))
    print(_dim("  Example: Go2ParkourLab → <parent>/Go2ParkourLab/"))
    value = _prompt("Project name", example="Go2ParkourLab")
    _validate_project_name(value)
    print()
    return value


def _prompt_package_name(project_name: str) -> str:
    suggested = _suggest_package_name(project_name)
    print(_dim("  Importable Python package under source/<package>/<package>/."))
    print(_dim("  Must be a valid identifier, e.g. go2_parkour."))
    value = _prompt_identifier("Package name", default=suggested, example="go2_parkour")
    print()
    return value


def _prompt_cfg_name() -> str:
    print(_dim("  Config package under tasks/.../config/<name>/."))
    print(_dim("  The Gym task id is derived from this:"))
    print(f"    {_cyan('my_go2')}  →  {_green('Velocity-Flat-MyGo2')}")
    print(f"    {_cyan('go2_parkour')}  →  {_green('Velocity-Flat-Go2Parkour')}")
    print(_dim("  Avoid go2 / g1 / b2; those collide with LocoLab built-in tasks."))
    value = _prompt_identifier("Config / task name", example="my_go2")
    task_id = f"Velocity-Flat-{_to_pascal(value)}"
    print(f"  {_dim('→')} task id  {_green(task_id)}")
    print(f"  {_dim('→')} env cfg  {_cyan(f'{_to_pascal(value)}FlatEnvCfg')}")
    if task_id in _BUILTIN_TASK_IDS:
        print(
            _yellow(
                f"  Warning: {task_id} is already registered by LocoLab. "
                "Pick a different config name unless you intend to replace it."
            )
        )
        if not _prompt_yes_no("Use this name anyway?", default=False):
            print(_dim("  Try again with a unique identifier, e.g. my_go2."))
            print()
            return _prompt_cfg_name()
    print()
    return value


def _prompt_path(message: str, default: Path, hint: str | None = None) -> Path:
    if hint:
        print(_dim(f"  {hint}"))
    raw = _prompt(message, default=str(default))
    print()
    return Path(raw).expanduser().resolve()


def _prompt_identifier(
    message: str,
    hint: str | None = None,
    example: str | None = None,
    default: str | None = None,
) -> str:
    if hint:
        print(_dim(f"  {hint}"))
    value = _prompt(message, default=default, example=example)
    if not value.isidentifier() or keyword.iskeyword(value):
        raise ValueError(f"{message} must be a valid Python identifier, got {value!r}.")
    return value


def _validate_project_name(value: str) -> None:
    if not value or value in {".", ".."} or "/" in value or "\\" in value or any(char.isspace() for char in value):
        raise ValueError(f"Project name must be a single directory name, got {value!r}.")


def _suggest_package_name(project_name: str) -> str | None:
    candidate = project_name.replace("-", "_")
    if candidate.isidentifier() and not keyword.iskeyword(candidate):
        return candidate
    return None


def _prompt_rl_libraries() -> tuple[str, ...]:
    print(_dim("  Agent config files to generate. Default trains with Z-RL."))
    raw = _prompt("RL library", default="z_rl", choices=_RL_CHOICES)
    if raw not in _RL_CHOICES:
        raise ValueError(f"RL library must be one of {', '.join(_RL_CHOICES)}, got {raw!r}.")
    print()
    if raw == "both":
        return ("z_rl", "rsl_rl")
    return (raw,)


def _prompt(message: str, default: str | None = None, example: str | None = None, choices: tuple[str, ...] | None = None) -> str:
    rl = {"readline_prompt": True}
    label = _bold(_cyan(message, **rl), **rl)
    extras: list[str] = []
    if choices:
        extras.append(_dim("/".join(choices), **rl))
    if default is not None:
        extras.append(_yellow(f"default {default}", **rl))
    elif example is not None:
        extras.append(_dim(f"e.g. {example}", **rl))
    suffix = f" {_dim('(', **rl)}{' · '.join(extras)}{_dim(')', **rl)}" if extras else ""
    value = input(f"  {label}{suffix}: ").strip()
    if value:
        return value
    if default is not None:
        return default
    raise ValueError(f"{message} is required.")


def _prompt_yes_no(message: str, default: bool) -> bool:
    rl = {"readline_prompt": True}
    hint = "Y/n" if default else "y/N"
    value = input(f"  {_bold(message, **rl)} {_dim(f'[{hint}]', **rl)}: ").strip().lower()
    if not value:
        return default
    return value in {"y", "yes"}


def _to_pascal(name: str) -> str:
    return "".join(part.capitalize() for part in name.split("_"))


def _locolab_repo_root() -> Path | None:
    for parent in Path(__file__).resolve().parents:
        if (parent / "source" / "locolab").is_dir() and (parent / "scripts" / "z_rl").is_dir():
            return parent
    return None


def _ensure_external_path(dest: Path) -> None:
    repo = _locolab_repo_root()
    if repo is None:
        return
    if dest == repo or repo in dest.parents:
        raise ValueError("External projects must live outside the LocoLab repository.")


def _print_summary(spec: ProjectSpec) -> None:
    print(_bold("Summary"))
    rows = [
        ("Path", str(spec.dest_dir)),
        ("Package", spec.package_name),
        ("Config dir", f"tasks/locomotion/velocity/config/{spec.robot_name}/"),
        ("Task id", spec.task_id),
        ("RL", ", ".join(spec.rl_libraries)),
    ]
    width = max(len(label) for label, _ in rows)
    for label, value in rows:
        print(f"  {_dim(label.ljust(width))}  {_magenta(value)}")
    print()


def _print_next_steps(spec: ProjectSpec, dest: Path) -> None:
    scripts = dest / "scripts"

    print()
    print(_green(_bold("Project generated")))
    print(f"  {dest}")
    print()
    print(_bold("Next steps"))
    commands = [
        f"python -m pip install -e {dest / 'source' / spec.package_name}",
        f"python {scripts / 'list_envs.py'}",
    ]
    for library in spec.rl_libraries:
        commands.append(f"python {scripts / library / 'train.py'} --task={spec.task_id}")
        commands.append(f"python {scripts / library / 'play.py'} --task={spec.task_id}")
    for index, command in enumerate(commands, start=1):
        print(f"  {_dim(f'{index}.')} {_cyan(command)}")
    print()
    print(_yellow("Note:") + _dim(" env_cfg.py is a placeholder. Fill in scene and MDP terms before training."))
    print()


if __name__ == "__main__":
    raise SystemExit(main())
