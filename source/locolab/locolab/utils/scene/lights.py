# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

def white_sky_light_cfg() -> AssetBaseCfg:
    """Create the default sky light for terrain scenes."""
    return AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

def blue_sky_light_cfg() -> AssetBaseCfg:
    """Create the default sky light for terrain scenes."""
    return AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr", intensity=750.0),
    )
