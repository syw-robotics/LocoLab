# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Implicit actuator groups for the Unitree G1 29-DOF model."""

from isaaclab.actuators import ImplicitActuatorCfg

# Following the principles of BeyondMimic, and the kp/kd computation logic.
# NOTE: These logic are still being tested, so we put them here for substitution in users Cfg class
ARMATURE_5020 = 0.003609725
ARMATURE_7520_14 = 0.010177520
ARMATURE_7520_22 = 0.025101925
ARMATURE_4010 = 0.00425

NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0

STIFFNESS_5020 = ARMATURE_5020 * NATURAL_FREQ**2
STIFFNESS_7520_14 = ARMATURE_7520_14 * NATURAL_FREQ**2
STIFFNESS_7520_22 = ARMATURE_7520_22 * NATURAL_FREQ**2
STIFFNESS_4010 = ARMATURE_4010 * NATURAL_FREQ**2

DAMPING_5020 = 2.0 * DAMPING_RATIO * ARMATURE_5020 * NATURAL_FREQ
DAMPING_7520_14 = 2.0 * DAMPING_RATIO * ARMATURE_7520_14 * NATURAL_FREQ
DAMPING_7520_22 = 2.0 * DAMPING_RATIO * ARMATURE_7520_22 * NATURAL_FREQ
DAMPING_4010 = 2.0 * DAMPING_RATIO * ARMATURE_4010 * NATURAL_FREQ

# Motion Tracking PD gains
mimic_g1_29dof_actuators = {
    "legs": ImplicitActuatorCfg(
        joint_names_expr=[
            ".*_hip_yaw_joint",
            ".*_hip_roll_joint",
            ".*_hip_pitch_joint",
            ".*_knee_joint",
        ],
        joint_effort_limit={
            ".*_hip_yaw_joint": 88.0,
            ".*_hip_roll_joint": 139.0,
            ".*_hip_pitch_joint": 88.0,
            ".*_knee_joint": 139.0,
        },
        joint_velocity_limit={
            ".*_hip_yaw_joint": 32.0,
            ".*_hip_roll_joint": 20.0,
            ".*_hip_pitch_joint": 32.0,
            ".*_knee_joint": 20.0,
        },
        stiffness={
            ".*_hip_pitch_joint": STIFFNESS_7520_14,
            ".*_hip_roll_joint": STIFFNESS_7520_22,
            ".*_hip_yaw_joint": STIFFNESS_7520_14,
            ".*_knee_joint": STIFFNESS_7520_22,
        },
        damping={
            ".*_hip_pitch_joint": DAMPING_7520_14,
            ".*_hip_roll_joint": DAMPING_7520_22,
            ".*_hip_yaw_joint": DAMPING_7520_14,
            ".*_knee_joint": DAMPING_7520_22,
        },
        armature={
            ".*_hip_pitch_joint": ARMATURE_7520_14,
            ".*_hip_roll_joint": ARMATURE_7520_22,
            ".*_hip_yaw_joint": ARMATURE_7520_14,
            ".*_knee_joint": ARMATURE_7520_22,
        },
    ),
    "feet": ImplicitActuatorCfg(
        joint_effort_limit=50.0,
        joint_velocity_limit=37.0,
        joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
        stiffness=2.0 * STIFFNESS_5020,
        damping=2.0 * DAMPING_5020,
        armature=2.0 * ARMATURE_5020,
    ),
    "waist": ImplicitActuatorCfg(
        joint_effort_limit=50,
        joint_velocity_limit=37.0,
        joint_names_expr=["waist_roll_joint", "waist_pitch_joint"],
        stiffness=2.0 * STIFFNESS_5020,
        damping=2.0 * DAMPING_5020,
        armature=2.0 * ARMATURE_5020,
    ),
    "waist_yaw": ImplicitActuatorCfg(
        joint_effort_limit=88,
        joint_velocity_limit=32.0,
        joint_names_expr=["waist_yaw_joint"],
        stiffness=STIFFNESS_7520_14,
        damping=DAMPING_7520_14,
        armature=ARMATURE_7520_14,
    ),
    "arms": ImplicitActuatorCfg(
        joint_names_expr=[
            ".*_shoulder_pitch_joint",
            ".*_shoulder_roll_joint",
            ".*_shoulder_yaw_joint",
            ".*_elbow_joint",
            ".*_wrist_roll_joint",
            ".*_wrist_pitch_joint",
            ".*_wrist_yaw_joint",
        ],
        joint_effort_limit={
            ".*_shoulder_pitch_joint": 25.0,
            ".*_shoulder_roll_joint": 25.0,
            ".*_shoulder_yaw_joint": 25.0,
            ".*_elbow_joint": 25.0,
            ".*_wrist_roll_joint": 25.0,
            ".*_wrist_pitch_joint": 5.0,
            ".*_wrist_yaw_joint": 5.0,
        },
        joint_velocity_limit={
            ".*_shoulder_pitch_joint": 37.0,
            ".*_shoulder_roll_joint": 37.0,
            ".*_shoulder_yaw_joint": 37.0,
            ".*_elbow_joint": 37.0,
            ".*_wrist_roll_joint": 37.0,
            ".*_wrist_pitch_joint": 22.0,
            ".*_wrist_yaw_joint": 22.0,
        },
        stiffness={
            ".*_shoulder_pitch_joint": STIFFNESS_5020,
            ".*_shoulder_roll_joint": STIFFNESS_5020,
            ".*_shoulder_yaw_joint": STIFFNESS_5020,
            ".*_elbow_joint": STIFFNESS_5020,
            ".*_wrist_roll_joint": STIFFNESS_5020,
            ".*_wrist_pitch_joint": STIFFNESS_4010,
            ".*_wrist_yaw_joint": STIFFNESS_4010,
        },
        damping={
            ".*_shoulder_pitch_joint": DAMPING_5020,
            ".*_shoulder_roll_joint": DAMPING_5020,
            ".*_shoulder_yaw_joint": DAMPING_5020,
            ".*_elbow_joint": DAMPING_5020,
            ".*_wrist_roll_joint": DAMPING_5020,
            ".*_wrist_pitch_joint": DAMPING_4010,
            ".*_wrist_yaw_joint": DAMPING_4010,
        },
        armature={
            ".*_shoulder_pitch_joint": ARMATURE_5020,
            ".*_shoulder_roll_joint": ARMATURE_5020,
            ".*_shoulder_yaw_joint": ARMATURE_5020,
            ".*_elbow_joint": ARMATURE_5020,
            ".*_wrist_roll_joint": ARMATURE_5020,
            ".*_wrist_pitch_joint": ARMATURE_4010,
            ".*_wrist_yaw_joint": ARMATURE_4010,
        },
    ),
}


mimic_g1_action_scale = {}
for a in mimic_g1_29dof_actuators.values():
    e = a.joint_effort_limit
    s = a.stiffness
    names = a.joint_names_expr
    if not isinstance(e, dict):
        e = {n: e for n in names}
    if not isinstance(s, dict):
        s = {n: s for n in names}
    for n in names:
        if n in e and n in s and s[n]:
            mimic_g1_action_scale[n] = 0.25 * e[n] / s[n]


# Locomotion PD gains
locomotion_g1_29dof_actuators = {
    "N7520-14.3": ImplicitActuatorCfg(
        joint_names_expr=[".*_hip_pitch_.*", ".*_hip_yaw_.*", "waist_yaw_joint"],
        joint_effort_limit=88,
        joint_velocity_limit=32.0,
        stiffness={
            ".*_hip_.*": 100.0,
            "waist_yaw_joint": 200.0,
        },
        damping={
            ".*_hip_.*": 2.0,
            "waist_yaw_joint": 5.0,
        },
        armature=0.01,
    ),
    "N7520-22.5": ImplicitActuatorCfg(
        joint_names_expr=[".*_hip_roll_.*", ".*_knee_.*"],
        joint_effort_limit=139,
        joint_velocity_limit=20.0,
        stiffness={
            ".*_hip_roll_.*": 100.0,
            ".*_knee_.*": 150.0,
        },
        damping={
            ".*_hip_roll_.*": 2.0,
            ".*_knee_.*": 4.0,
        },
        armature=0.01,
    ),
    "N5020-16": ImplicitActuatorCfg(
        joint_names_expr=[
            ".*_shoulder_.*",
            ".*_elbow_.*",
            ".*_wrist_roll.*",
            ".*_ankle_.*",
            "waist_roll_joint",
            "waist_pitch_joint",
        ],
        joint_effort_limit=25,
        joint_velocity_limit=37,
        stiffness=40.0,
        damping={
            ".*_shoulder_.*": 1.0,
            ".*_elbow_.*": 1.0,
            ".*_wrist_roll.*": 1.0,
            ".*_ankle_.*": 2.0,
            "waist_.*_joint": 5.0,
        },
        armature=0.01,
    ),
    "W4010-25": ImplicitActuatorCfg(
        joint_names_expr=[".*_wrist_pitch.*", ".*_wrist_yaw.*"],
        joint_effort_limit=5,
        joint_velocity_limit=22,
        stiffness=40.0,
        damping=1.0,
        armature=0.01,
    ),
}


# Matches unitree_rl_lab g1 velocity ``JointPositionActionCfg(scale=0.25)``.
locomotion_g1_action_scale = 0.25

