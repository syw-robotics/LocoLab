"""Render the bundle's README template with its robot name."""

from pathlib import Path

from . import TEMPLATE_DIR


def write_readme(directory: Path, robot: str, *, tracking: bool) -> None:
    template = TEMPLATE_DIR / ("tracking" if tracking else "velocity") / "README.md"
    text = template.read_text(encoding="utf-8").replace("{{robot}}", robot)
    (directory / "README.md").write_text(text, encoding="utf-8")
