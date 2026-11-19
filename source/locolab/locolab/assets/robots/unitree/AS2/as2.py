# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Configuration for the Unitree AS2 quadruped."""

import os

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from locolab.assets.robots.common import LocoLabArticulationCfg, assetx_export_usd_cfg
from locolab.utils.symmetry import complete_symmetry_mapping


_AS2_DIR = os.path.dirname(os.path.abspath(__file__))
_AS2_SOURCE_URDF = os.path.join(_AS2_DIR, "urdf", "as2.urdf")


# fmt: off
AS2_JOINT_SDK_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]
# fmt: on

AS2_JOINT_SYMMETRY_MAPPING = complete_symmetry_mapping(
    {
        "FR_hip_joint": (-1.0, "FL_hip_joint"),
        "RR_hip_joint": (-1.0, "RL_hip_joint"),
        "FR_thigh_joint": (1.0, "FL_thigh_joint"),
        "RR_thigh_joint": (1.0, "RL_thigh_joint"),
        "FR_calf_joint": (1.0, "FL_calf_joint"),
        "RR_calf_joint": (1.0, "RL_calf_joint"),
    }
)

AS2_SPATIAL_SYMMETRY_MAPPING = complete_symmetry_mapping(
    {
        "FR_hip": "FL_hip",
        "RR_hip": "RL_hip",
        "FR_thigh": "FL_thigh",
        "RR_thigh": "RL_thigh",
        "FR_calf": "FL_calf",
        "RR_calf": "RL_calf",
        "FR_foot": "FL_foot",
        "RR_foot": "RL_foot",
        "base_link": "base_link",
    }
)

UNITREE_AS2_CFG = LocoLabArticulationCfg(
    deployment_robot="as2",
    spawn=assetx_export_usd_cfg(
        asset_path=_AS2_SOURCE_URDF,
        foot_expression="/.*_foot/collisions.*",
        merge_fixed_joints=True,
        keep_fixed_pattern=r".*_foot_joint$",
        honor_dont_collapse=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.45),
        joint_pos={
            ".*_hip_joint": 0.0,
            "F[LR]_thigh_joint": 0.8,
            "R[LR]_thigh_joint": 1.0,
            ".*_calf_joint": -1.5,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "AS2Implicit_Hip_Thigh": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_joint", ".*_thigh_joint"],
            joint_effort_limit=60.0,
            joint_velocity_limit=24.0,
            stiffness=40.0,
            damping=1.0,
        ),
        "AS2Implicit_Calf": ImplicitActuatorCfg(
            joint_names_expr=[".*_calf_joint"],
            joint_effort_limit=90.0,
            joint_velocity_limit=16.0,
            stiffness=40.0,
            damping=1.0,
        ),
    },
    joint_symmetry_mapping=AS2_JOINT_SYMMETRY_MAPPING,
    spatial_symmetry_mapping=AS2_SPATIAL_SYMMETRY_MAPPING,
    joint_sdk_names=AS2_JOINT_SDK_NAMES,
)
