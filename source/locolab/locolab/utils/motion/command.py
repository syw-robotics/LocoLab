"""Selectable reference observations, independent of the tracking anchor."""

import torch

from isaaclab.utils.math import quat_apply_inverse


COMMAND_DIMS = {
    "joint_pos": None,
    "joint_vel": None,
    "base_height": 1,
    "base_lin_vel": 3,
    "base_ang_vel": 3,
    "projected_gravity": 3,
}


def command_dimension(terms, num_joints, frame_offsets=(0,)):
    """Validate the configured layout once, before constructing observations."""
    if not terms or len(set(terms)) != len(terms):
        raise ValueError("command_terms must be non-empty and contain no duplicates.")
    unknown = set(terms) - COMMAND_DIMS.keys()
    if unknown:
        raise ValueError(f"Unknown command_terms: {sorted(unknown)}. Choose from {tuple(COMMAND_DIMS)}.")
    if not frame_offsets or any(type(offset) is not int for offset in frame_offsets):
        raise ValueError("command_frame_offsets must be a non-empty sequence of integer control-step offsets.")
    return len(frame_offsets) * sum(COMMAND_DIMS[name] or num_joints for name in terms)


def command_sources(motion, terms):
    """Share joint trajectories and precompute only selected root fields once."""
    return tuple(
        getattr(motion, name) if name in ("joint_pos", "joint_vel")
        else reference_command(motion, slice(None), (name,))
        for name in terms
    )


def reference_command(motion, frames, terms):
    """Concatenate selected reference fields in configuration order.

    Base means the motion root (pelvis on G1). Height is in the clip's world
    frame; absolute velocities and unit gravity are expressed in the reference
    base frame. Root quaternions use xyzw. No temporal lookahead is applied.
    """
    values = []
    for name in terms:
        if name in ("joint_pos", "joint_vel"):
            value = getattr(motion, name)[frames]
        elif name == "base_height":
            value = motion.root_pos[frames][..., 2:3]
        elif name in ("base_lin_vel", "base_ang_vel"):
            velocity = motion.root_lin_vel if name == "base_lin_vel" else motion.root_ang_vel
            value = quat_apply_inverse(motion.root_quat[frames], velocity[frames])
        else:  # projected_gravity; names were validated at initialization.
            quat = motion.root_quat[frames]
            value = quat_apply_inverse(quat, quat.new_tensor((0.0, 0.0, -1.0)))
        values.append(value)
    return torch.cat(values, dim=-1)
