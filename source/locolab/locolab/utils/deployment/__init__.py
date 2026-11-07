"""Add unitree-deploy artifacts after the runner exports its normalized ONNX actor."""

from pathlib import Path


def export_deployment(env, path: str | Path, clip_actions: float | None = None) -> None:
    if getattr(env.cfg.commands, "motion", None) is not None:
        from .tracking import export_tracking_deployment

        export_tracking_deployment(env, path, clip_actions)
    elif getattr(env.cfg.commands, "base_velocity", None) is not None:
        from .velocity import export_velocity_deployment

        export_velocity_deployment(env, path, clip_actions)
    else:
        print("[ONNX] No unitree-deploy exporter for this task; exported the actor only.")
