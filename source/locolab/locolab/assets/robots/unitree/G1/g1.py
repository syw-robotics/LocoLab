# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Configuration for the Unitree G1 29-DOF humanoid.

Reference: https://github.com/unitreerobotics/unitree_rl_lab
"""

import os

from isaaclab.assets.articulation import ArticulationCfg

from locolab.assets.actuators.g1_beyondmimic import beyondmimic_g1_29dof_actuators
from locolab.assets.robots.common import (
    LocoLabArticulationCfg,
    assetx_export_usd_cfg,
)
from locolab.utils.symmetry import complete_symmetry_mapping

_G1_DIR = os.path.dirname(os.path.abspath(__file__))

_G1_XML_DIR = os.path.join(_G1_DIR, "xml")
_G1_PELVIS_BASE_XML = os.path.join(_G1_XML_DIR, "g1_29dof_pelvis_base.xml")
_G1_TORSO_BASE_XML = os.path.join(_G1_XML_DIR, "g1_29dof_torso_base.xml")


G1_JOINT_SYMMETRY_MAPPING = complete_symmetry_mapping(
    {
        "left_hip_pitch_joint": (1.0, "right_hip_pitch_joint"),
        "left_hip_roll_joint": (-1.0, "right_hip_roll_joint"),
        "left_hip_yaw_joint": (-1.0, "right_hip_yaw_joint"),
        "left_knee_joint": (1.0, "right_knee_joint"),
        "left_ankle_pitch_joint": (1.0, "right_ankle_pitch_joint"),
        "left_ankle_roll_joint": (-1.0, "right_ankle_roll_joint"),
        "waist_yaw_joint": (-1.0, "waist_yaw_joint"),
        "waist_roll_joint": (-1.0, "waist_roll_joint"),
        "waist_pitch_joint": (1.0, "waist_pitch_joint"),
        "left_shoulder_pitch_joint": (1.0, "right_shoulder_pitch_joint"),
        "left_shoulder_roll_joint": (-1.0, "right_shoulder_roll_joint"),
        "left_shoulder_yaw_joint": (-1.0, "right_shoulder_yaw_joint"),
        "left_elbow_joint": (1.0, "right_elbow_joint"),
        "left_wrist_roll_joint": (-1.0, "right_wrist_roll_joint"),
        "left_wrist_pitch_joint": (1.0, "right_wrist_pitch_joint"),
        "left_wrist_yaw_joint": (-1.0, "right_wrist_yaw_joint"),
    }
)

G1_SPATIAL_SYMMETRY_MAPPING = complete_symmetry_mapping(
    {
        "pelvis": "pelvis",
        "left_hip_pitch_link": "right_hip_pitch_link",
        "left_hip_roll_link": "right_hip_roll_link",
        "left_hip_yaw_link": "right_hip_yaw_link",
        "left_knee_link": "right_knee_link",
        "left_ankle_pitch_link": "right_ankle_pitch_link",
        "left_ankle_roll_link": "right_ankle_roll_link",
        "waist_yaw_link": "waist_yaw_link",
        "waist_roll_link": "waist_roll_link",
        "torso_link": "torso_link",
        "left_shoulder_pitch_link": "right_shoulder_pitch_link",
        "left_shoulder_roll_link": "right_shoulder_roll_link",
        "left_shoulder_yaw_link": "right_shoulder_yaw_link",
        "left_elbow_link": "right_elbow_link",
        "left_wrist_roll_link": "right_wrist_roll_link",
        "left_wrist_pitch_link": "right_wrist_pitch_link",
        "left_wrist_yaw_link": "right_wrist_yaw_link",
    }
)

_G1_SDK_JOINT_NAMES = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

UNITREE_G1_29DOF_PELVIS_BASE_CFG = LocoLabArticulationCfg(
    spawn=assetx_export_usd_cfg(
        asset_path=_G1_PELVIS_BASE_XML,
        foot_expression="/.*ankle_roll_link/collisions.*",
        merge_fixed_joints=True,
    ),
    source_asset=_G1_PELVIS_BASE_XML,
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.88),
        joint_pos={
            ".*_hip_pitch_joint": -0.1,
            ".*_knee_joint": 0.3,
            ".*_ankle_pitch_joint": -0.2,
            ".*_elbow_joint": 0.9,
            ".*_shoulder_pitch_joint": 0.2,
            "left_shoulder_roll_joint": 0.25,
            "right_shoulder_roll_joint": -0.25,
        },
        joint_vel={".*": 0.0},
    ),
    actuators=beyondmimic_g1_29dof_actuators,
    joint_symmetry_mapping=G1_JOINT_SYMMETRY_MAPPING,
    spatial_symmetry_mapping=G1_SPATIAL_SYMMETRY_MAPPING,
    joint_sdk_names=_G1_SDK_JOINT_NAMES,
)
"""Configuration for the pelvis-base Unitree G1 29DOF humanoid robot."""

UNITREE_G1_29DOF_TORSO_BASE_CFG = UNITREE_G1_29DOF_PELVIS_BASE_CFG.replace(
    spawn=assetx_export_usd_cfg(
        asset_path=_G1_TORSO_BASE_XML,
        foot_expression="/.*ankle_roll_link/collisions.*",
        merge_fixed_joints=True,
    ),
    source_asset=_G1_TORSO_BASE_XML,
)
"""Configuration for the torso-base Unitree G1 29DOF humanoid robot."""
