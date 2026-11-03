# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Resolve and compile the MJCF saved alongside the articulation's USD."""

from pathlib import Path

import mujoco


def robot_mjcf_path(cfg) -> Path:
    """Prefer the converted model, including fixed-link and collision edits."""
    spawn = getattr(cfg, "spawn", None)
    usd_path = getattr(spawn, "usd_path", None)
    if usd_path:
        # assetx writes <bundle>/model.xml and <bundle>/usd/<robot>.usdc.
        # Use that XML so fixed-link and collision edits match the spawned USD.
        usd = Path(usd_path).expanduser()
        xml = usd.parent.parent / "model.xml"
        if usd.parent.name == "usd" and xml.is_file():
            return xml.resolve()
    # Non-assetx USDs can provide their matching MJCF explicitly.
    source = getattr(cfg, "mjcf_path", None) or getattr(spawn, "asset_path", None)
    if source:
        path = Path(source).expanduser()
        if path.suffix.lower() in {".xml", ".mjcf"}:
            if not path.is_file():
                raise FileNotFoundError(path)
            return path.resolve()
    raise ValueError(
        "Robot visualization needs the assetx model.xml beside spawn.usd_path's usd/ directory, "
        "or an MJCF path in robot.cfg.mjcf_path. Convert URDF assets with assetx first."
    )


def load_mjcf_model(path: str | Path) -> mujoco.MjModel:
    """Generate hull data for visual-only meshes in this display model too."""
    spec = mujoco.MjSpec.from_file(str(path))
    original_collision_types = []
    for geom in spec.geoms:
        if geom.type == mujoco.mjtGeom.mjGEOM_MESH:
            original_collision_types.append((geom, geom.contype))
            # MuJoCo otherwise omits hulls for meshes used only by visual geoms.
            geom.contype = 1
    try:
        model = spec.compile()
    except ValueError:
        # Planar visual meshes cannot form 3D hulls. Retry unchanged; an invalid
        # source still raises its original error rather than being hidden here.
        return mujoco.MjModel.from_xml_path(str(path))
    # Collision flags are used below to distinguish visual and collision layers.
    for geom, contype in original_collision_types:
        model.geom_contype[geom.id] = contype
    return model
