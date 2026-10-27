# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Configuration for the Unitree B2 quadruped.

Reference: https://github.com/unitreerobotics/unitree_rl_lab
"""

import os

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

from locolab.assets.robots.common import LocoLabArticulationCfg, quadruped_urdf_cfg
from locolab.utils.symmetry import complete_symmetry_mapping

_B2_DIR = os.path.dirname(os.path.abspath(__file__))
_B2_SOURCE_URDF = os.path.join(_B2_DIR, "urdf", "b2_description", "urdf", "b2_description.urdf")

# fmt: off
B2_JOINT_SDK_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]
# fmt: on

B2_JOINT_SYMMETRY_MAPPING = complete_symmetry_mapping(
    {
        "FR_hip_joint": (-1.0, "FL_hip_joint"),
        "RR_hip_joint": (-1.0, "RL_hip_joint"),
        "FR_thigh_joint": (1.0, "FL_thigh_joint"),
        "RR_thigh_joint": (1.0, "RL_thigh_joint"),
        "FR_calf_joint": (1.0, "FL_calf_joint"),
        "RR_calf_joint": (1.0, "RL_calf_joint"),
    }
)

B2_SPATIAL_SYMMETRY_MAPPING = complete_symmetry_mapping(
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

UNITREE_B2_CFG = LocoLabArticulationCfg(
    spawn=quadruped_urdf_cfg(
        _B2_SOURCE_URDF,
        "/.*_foot/sphere.*",
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.58),
        joint_pos={
            ".*R_hip_joint": -0.1,
            ".*L_hip_joint": 0.1,
            "F[L,R]_thigh_joint": 0.8,
            "R[L,R]_thigh_joint": 1.0,
            ".*_calf_joint": -1.5,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "B2Implicit_Hip_Thigh": ImplicitActuatorCfg(
            joint_names_expr=[".*_hip_joint", ".*_thigh_joint"],
            joint_effort_limit=200.0,
            joint_velocity_limit=23.0,
            stiffness=100.0,
            damping=2.0,
            friction=0.1,
            armature=0.1,
        ),
        "B2Implicit_Calf": ImplicitActuatorCfg(
            joint_names_expr=[".*_calf_joint"],
            joint_effort_limit=320.0,
            joint_velocity_limit=14.0,
            stiffness=100.0,
            damping=2.0,
            friction=0.1,
            armature=0.1,
        ),
    },
    joint_symmetry_mapping=B2_JOINT_SYMMETRY_MAPPING,
    spatial_symmetry_mapping=B2_SPATIAL_SYMMETRY_MAPPING,
    joint_sdk_names=B2_JOINT_SDK_NAMES,
)
"""Configuration for the Unitree B2 quadrupedal robot."""
