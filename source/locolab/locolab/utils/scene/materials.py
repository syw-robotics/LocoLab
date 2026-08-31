# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

import isaaclab.sim as sim_utils
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR, NVIDIA_NUCLEUS_DIR


def flat_terrain_visual_material_cfg() -> sim_utils.PreviewSurfaceCfg:
    """Create the default visual material for flat terrain."""
    return sim_utils.PreviewSurfaceCfg(
        diffuse_color=(1.0, 1.0, 1.0),
        metallic=0.0,
        roughness=0.8,
    )

def flat_rough_terrain_visual_material_cfg() -> sim_utils.PreviewSurfaceCfg:
    """Create the default visual material for flat rough terrain."""
    return sim_utils.MdlFileCfg(
        mdl_path=f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
        project_uvw=True,
        texture_scale=(0.25, 0.25),
    )

def rough_terrain_visual_material_cfg() -> sim_utils.MdlFileCfg:
    """Create the default visual material for rough terrain."""
    return sim_utils.MdlFileCfg(
        mdl_path=
            #  f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl"
            #  f"{ISAACLAB_NUCLEUS_DIR}/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            #  f"{NVIDIA_NUCLEUS_DIR}/Materials/Base/Wood/Oak_Planks.mdl",
            f"{NVIDIA_NUCLEUS_DIR}/Materials/Base/Wood/Oak.mdl",
            #  f"{NVIDIA_NUCLEUS_DIR}/Materials/Base/Wood/Birch_Planks.mdl",
            #  f"{NVIDIA_NUCLEUS_DIR}/Materials/Base/Wood/Birch.mdl",
        project_uvw=True,
        #  texture_scale=(0.25, 0.25),
        texture_scale=(1.0, 1.0),
    )
