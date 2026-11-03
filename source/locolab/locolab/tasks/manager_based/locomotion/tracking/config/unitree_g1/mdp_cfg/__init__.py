# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""G1 motion-tracking MDP configuration."""

LAFAN_DATASET_ROOT = "/mnt/datasets/HumanoidMotionDataset/any4hdmi-converted/any4hdmi-g1-lafan"
DEFAULT_MOTION_FILE = f"{LAFAN_DATASET_ROOT}/motions/dance1_subject3.npz"

FOOT_LINK_NAMES = ".*ankle_roll.*"
TORSO_LINK_NAME = "torso_link"

# Same hinge order as the converted Lafan qpos, after the 7 root coordinates.
JOINT_NAMES = [
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

BODY_NAMES = (
    "pelvis",
    "left_hip_roll_link",
    "left_knee_link",
    "left_ankle_roll_link",
    "right_hip_roll_link",
    "right_knee_link",
    "right_ankle_roll_link",
    "torso_link",
    "left_shoulder_roll_link",
    "left_elbow_link",
    "left_wrist_yaw_link",
    "right_shoulder_roll_link",
    "right_elbow_link",
    "right_wrist_yaw_link",
)

ANCHOR_BODY_NAME = "torso_link"
OTHER_BODY_LINK_NAMES = [
    ".*_hip_.*_link",
    ".*_knee_link",
    ".*_shoulder_.*_link",
    ".*_elbow_link",
    ".*_wrist_.*_link",
]
VELOCITY_RANGE: dict[str, tuple[float, float]] = {
    "x": (-0.5, 0.5),
    "y": (-0.5, 0.5),
    "z": (-0.2, 0.2),
    "roll": (-0.52, 0.52),
    "pitch": (-0.52, 0.52),
    "yaw": (-0.78, 0.78),
}
END_EFFECTOR_BODY_NAMES = (
    "left_ankle_roll_link",
    "right_ankle_roll_link",
    "left_wrist_yaw_link",
    "right_wrist_yaw_link",
)
from .actions_cfg import ActionsCfg
from .commands_cfg import CommandsCfg
from .events_cfg import EventCfg
from .observations_cfg import PolicyCfg, PrivilegedCfg
from .rewards_cfg import RewardsCfg
from .terminations_cfg import TerminationsCfg

__all__ = [
    "ActionsCfg",
    "CommandsCfg",
    "EventCfg",
    "PolicyCfg",
    "PrivilegedCfg",
    "RewardsCfg",
    "TerminationsCfg",
]
