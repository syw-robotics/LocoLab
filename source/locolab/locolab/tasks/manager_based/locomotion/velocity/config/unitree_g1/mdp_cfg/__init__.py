# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

FOOT_LINK_NAMES = ".*ankle_roll.*"
CONTACT_SENSOR_LINK_NAMES = ".*_(wrist_yaw|hip_roll|knee|ankle_roll)_link"
UNDESIRED_CONTACT_LINK_NAMES = ["(?!.*ankle.*).*"]
ARM_JOINT_NAMES = [".*_shoulder_.*", ".*_elbow_.*", ".*_wrist_.*"]
ARM_JOINT_NAMES_WO_SHOULDER_PITCH = [
    ".*_shoulder_roll_joint",
    ".*_shoulder_yaw_joint",
    ".*_elbow_.*",
    ".*_wrist_.*",
]
ARM_SHOULDER_PITCH_JOINT_NAME = ".*_shoulder_pitch_joint"
HIP_ROLL_JOINT_NAME = ".*_hip_roll_.*"
HIP_YAW_JOINT_NAME = ".*_hip_yaw_.*"
WAIST_JOINT_NAMES = ["waist_.*"]
TORSO_LINK_NAME = "torso_link"
PELVIS_LINK_NAME = "pelvis"
COORDINATION_JOINT_NAMES = [
    "left_hip_pitch_joint",
    "right_shoulder_pitch_joint",
    "right_hip_pitch_joint",
    "left_shoulder_pitch_joint",
]

PRESERVE_ORDER = True

# fmt: off
JOINT_NAMES = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
# fmt: on

from .actions_cfg import ActionsCfg
from .commands_cfg import CommandsCfg
from .curriculumns_cfg import FlatCurriculumsCfg
from .events_cfg import EventCfg, RoughEventCfg
from .observations_cfg import HeightmapPropObsCfg, PrivObsCfg, PropObsCfg
from .rewards_cfg import FlatRewardsCfg, RoughRewardsCfg
from .terminations_cfg import FlatTerminationsCfg, RoughTerminationsCfg

__all__ = [
    "ActionsCfg",
    "CommandsCfg",
    "EventCfg",
    "RoughEventCfg",
    "FlatCurriculumsCfg",
    "RoughTerminationsCfg",
    "PropObsCfg",
    "PrivObsCfg",
    "HeightmapPropObsCfg",
    "FlatRewardsCfg",
    "RoughRewardsCfg",
    "FlatTerminationsCfg",
    "RoughTerminationsCfg",
]
