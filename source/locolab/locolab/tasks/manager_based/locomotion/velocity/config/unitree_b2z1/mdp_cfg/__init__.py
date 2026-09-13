# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

BASE_LINK_NAME = "base_link"
FOOT_LINK_NAMES = ".*_foot"
HIP_JOINT_NAMES = [".*_hip_joint"]
ARM_JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]
ARM_EE_LINK_NAME = "ee_gripper_link"
GRIPPER_LINK_NAMES = ["ee_gripper_link"]
UNDESIRED_CONTACT_LINK_NAMES = ["(?!.*_foot).*"]
OTHER_BODY_LINK_NAMES = [".*_hip", ".*_thigh", ".*_calf", "link0[0-6]"]
CONTACT_SENSOR_LINK_NAMES = "(base_link|.*_(foot|calf))"

PRESERVE_ORDER = True

# fmt: off
LEG_JOINT_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]
JOINT_NAMES = LEG_JOINT_NAMES + ARM_JOINT_NAMES
# fmt: on

# npz key "arm_joint_q_and_ee_pose", shape (N, n_arm + 7): [arm_q..., x, y, z, qw, qx, qy, qz] in base_link
DEFAULT_ARM_EE_POSE_DATASET = "/home/syw/.gitrepos/pyroki_to/data/b2_z1_arm_joint_q_and_ee_pose.npz"
# npz key "ee_pose", shape (N, T, 7): [x, y, z, qw, qx, qy, qz] in base_link
DEFAULT_ARM_EE_TRAJ_DATASET = "/home/syw/.gitrepos/pyroki_to/data/b2_z1_arm_ws_ee_traj.npz"

# Nominal flat-terrain base height in the environment world frame [m].
NOMINAL_BASE_HEIGHT_Z = 0.46

from .actions_cfg import ActionsCfg
from .commands_cfg import (
    VelocityEEPositionCmdCfg,
    VelocityEETrajPositionCmdCfg,
)
from .curriculumns_cfg import RoughCurriculumsCfg
from .events_cfg import EventCfg
from .observations_cfg import (
    PropObsArmEePosCfg,
    PropObsArmEePoseCfg,
    PrivObsArmEePosCfg,
    PrivObsArmEePoseCfg,
)
from .rewards_cfg import FlatRewardsArmEePosCfg
from .terminations_cfg import FlatTerminationsCfg, RoughTerminationsCfg

__all__ = [
    "ActionsCfg",
    "VelocityEEPositionCmdCfg",
    "VelocityEETrajPositionCmdCfg",
    "EventCfg",
    "RoughCurriculumsCfg",
    "PropObsArmEePosCfg",
    "PropObsArmEePoseCfg",
    "PrivObsArmEePosCfg",
    "PrivObsArmEePoseCfg",
    "FlatRewardsArmEePosCfg",
    # "FlatRewardsArmEePoseCfg",
    "FlatTerminationsCfg",
    "RoughTerminationsCfg",
]
