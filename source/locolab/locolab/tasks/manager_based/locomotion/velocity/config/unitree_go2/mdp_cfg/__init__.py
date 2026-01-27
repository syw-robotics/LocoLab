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

PRESERVE_ORDER = True

# fmt: off
JOINT_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
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
