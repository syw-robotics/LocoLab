"""Copy the tracking controller's editable YAML templates into the bundle."""

from pathlib import Path
import shutil

from . import TEMPLATE_DIR


def write_tracking_state_machine(directory: str | Path, *, general: bool = False) -> None:
    for name in ("multi_ckpt.yaml", "state_machine.yaml"):
        template = f"gmt_{name}" if general else name
        shutil.copyfile(TEMPLATE_DIR / "tracking" / template, Path(directory) / name)
