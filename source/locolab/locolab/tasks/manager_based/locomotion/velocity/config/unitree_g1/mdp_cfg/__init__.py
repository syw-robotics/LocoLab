# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

BASE_LINK_NAME = "base"
FOOT_LINK_NAMES = ".*_foot"
HIP_JOINT_NAMES = [".*_hip_joint"]
UNDESIRED_CONTACT_LINK_NAMES = ["Head_.*", ".*_hip", ".*_thigh", ".*_calf"]
OTHER_BODY_LINK_NAMES = [".*_hip", ".*_thigh", ".*_calf"]
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
from .curriculumns_cfg import RoughCurriculumsCfg
from .events_cfg import EventCfg
from .observations_cfg import PrivObsCfg, PropObsCfg
from .rewards_cfg import FlatRewardsCfg, RoughRewardsCfg
from .terminations_cfg import FlatTerminationsCfg, RoughTerminationsCfg

__all__ = [
    "ActionsCfg",
    "CommandsCfg",
    "EventCfg",
    "RoughCurriculumsCfg",
    "PropObsCfg",
    "PrivObsCfg",
    "FlatRewardsCfg",
    "RoughRewardsCfg",
    "FlatTerminationsCfg",
    "RoughTerminationsCfg",
]
