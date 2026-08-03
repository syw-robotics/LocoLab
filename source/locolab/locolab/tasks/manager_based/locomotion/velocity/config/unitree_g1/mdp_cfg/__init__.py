# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

FOOT_LINK_NAMES = ".*ankle_roll.*"
UNDESIRED_CONTACT_LINK_NAMES = ["(?!.*ankle.*).*"]
ARM_JOINT_NAMES = [".*_shoulder_.*", ".*_elbow_.*", ".*_wrist_.*"]
HIP_ROLL_JOINT_NAME = ".*_hip_roll_.*"
HIP_YAW_JOINT_NAME = ".*_hip_yaw_.*"
WAIST_JOINT_NAMES = ["waist_.*"]
TORSO_LINK_NAME = "torso_link"
ROOT_LINK_NAME = "pelvis"

FOOT_BODY_NAMES = ["left_ankle_roll_link", "right_ankle_roll_link"]
HAND_BODY_NAMES = ["left_wrist_yaw_link", "right_wrist_yaw_link"]
HIP_BODY_NAMES = ["left_hip_yaw_link", "right_hip_yaw_link"]
KNEE_BODY_NAMES = ["left_knee_link", "right_knee_link"]
ANKLE_BODY_NAMES = ["left_ankle_roll_link", "right_ankle_roll_link"]
CORE_BODY_NAMES = ["pelvis", "waist_yaw_link", "waist_roll_link", "torso_link"]

ELBOW_WRIST_JOINT_NAMES = [
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
WRIST_JOINT_NAMES = [
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
ELBOW_JOINT_NAMES = ["left_elbow_joint", "right_elbow_joint"]

AMP_ANCHOR_NAME = "torso_link"

# fmt: off
AMP_BODY_NAMES = [
    "pelvis",
    "left_hip_pitch_link", "left_hip_roll_link", "left_hip_yaw_link", "left_knee_link",
    "left_ankle_pitch_link", "left_ankle_roll_link",
    "right_hip_pitch_link", "right_hip_roll_link", "right_hip_yaw_link", "right_knee_link",
    "right_ankle_pitch_link", "right_ankle_roll_link",
    "waist_yaw_link", "waist_roll_link", "torso_link",
]
# fmt: on


# fmt: off
AMP_MOTION_BODY_NAMES = [
    "pelvis",
    "left_hip_pitch_link", "left_hip_roll_link", "left_hip_yaw_link", "left_knee_link",
    "left_ankle_pitch_link", "left_ankle_roll_link",
    "right_hip_pitch_link", "right_hip_roll_link", "right_hip_yaw_link", "right_knee_link",
    "right_ankle_pitch_link", "right_ankle_roll_link",
    "waist_yaw_link", "waist_roll_link", "torso_link",
    "left_shoulder_pitch_link", "left_shoulder_roll_link", "left_shoulder_yaw_link",
    "left_elbow_link", "left_wrist_roll_link", "left_wrist_pitch_link", "left_wrist_yaw_link",
    "right_shoulder_pitch_link", "right_shoulder_roll_link", "right_shoulder_yaw_link",
    "right_elbow_link", "right_wrist_roll_link", "right_wrist_pitch_link", "right_wrist_yaw_link",
]
# fmt: on


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
from .events_cfg import EventCfg
from .observations_cfg import AmpPolicyObsCfg, AmpReferenceObsCfg, PrivObsCfg, PropObsCfg
from .rewards_cfg import FlatAmpRewardsCfg, FlatRewardsCfg
from .terminations_cfg import FlatTerminationsCfg

__all__ = [
    "ActionsCfg",
    "CommandsCfg",
    "EventCfg",
    "FlatCurriculumsCfg",
    "PropObsCfg",
    "PrivObsCfg",
    "AmpPolicyObsCfg",
    "AmpReferenceObsCfg",
    "FlatAmpRewardsCfg",
    "FlatRewardsCfg",
    "FlatTerminationsCfg",
    "AMP_BODY_NAMES",
    "AMP_MOTION_BODY_NAMES",
    "AMP_ANCHOR_NAME",
    "ROOT_LINK_NAME",
    "FOOT_BODY_NAMES",
    "HAND_BODY_NAMES",
    "HIP_BODY_NAMES",
    "KNEE_BODY_NAMES",
    "ANKLE_BODY_NAMES",
    "CORE_BODY_NAMES",
    "ELBOW_WRIST_JOINT_NAMES",
    "WRIST_JOINT_NAMES",
    "ELBOW_JOINT_NAMES",
]
