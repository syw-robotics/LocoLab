# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""
Reference: https://github.com/project-instinct/instinct_rl.git
"""

from __future__ import annotations

from dataclasses import MISSING, dataclass

import torch


@dataclass
class VolumePointsData:
    """Data container for the volume points sensor."""

    pos_w: torch.Tensor = MISSING
    """The position of the volume points sensor in the world frame.

    Shape: (N, B, 3), where N is the number of envs, B is the number of bodies in each env.
    """

    quat_w: torch.Tensor = MISSING
    """The quaternion of the volume points sensor in the world frame.

    Shape: (N, B, 4), where N is the number of envs, B is the number of bodies in each env.
    The quaternion is in the format (x, y, z, w).
    """

    vel_w: torch.Tensor = MISSING
    """The velocity of the volume points sensor in the world frame.

    Shape: (N, B, 3), where N is the number of envs, B is the number of bodies in each env.
    The velocity is in the format (vx, vy, vz).
    """

    ang_vel_w: torch.Tensor = MISSING
    """The angular velocity of the volume points sensor in the world frame.

    Shape: (N, B, 3), where N is the number of envs, B is the number of bodies in each env.
    """

    point_num_each_body: int = MISSING
    """The number of volume points in each body.
    This is used to calculate the shape of the volume points data.
    """

    points_pos_w: torch.Tensor = MISSING
    """The position of the volume points in the world frame.

    Shape is (N, B, point_num_each_body, 3),
    where N is the number of sensors and B is the number of bodies in each sensor.
    """

    points_vel_w: torch.Tensor = MISSING
    """The velocity of the volume points in the world frame.

    Shape is (N, B, point_num_each_body, 3),
    where N is the number of sensors and B is the number of bodies in each sensor.
    """

    penetration_offset: torch.Tensor = MISSING
    """The penetration offset of the volume points sensor.
    This is the offset from the surface of the body to the volume points.

    If the point moves along the penetration direction, the penetration depth increases.
    If the point has no penetration, the penetration depth is zero and the direction is undefined.

    Shape is (N, B, point_num_each_body, 3), where N is the number of envs, B is the number of bodies in each env.
    """

    @staticmethod
    def make_zero(
        num_envs: int,
        num_bodies: int,
        point_num_each_body: int,
        device="cpu",
        dtype=torch.float32,
        # Note: device and dtype are optional parameters for flexibility in tensor creation
    ) -> VolumePointsData:
        """Creates a zero-initialized VolumePointsData object."""
        return VolumePointsData(
            pos_w=torch.zeros((num_envs, num_bodies, 3), device=device, dtype=dtype),
            quat_w=torch.zeros((num_envs, num_bodies, 4), device=device, dtype=dtype),
            vel_w=torch.zeros((num_envs, num_bodies, 3), device=device, dtype=dtype),
            ang_vel_w=torch.zeros((num_envs, num_bodies, 3), device=device, dtype=dtype),
            point_num_each_body=point_num_each_body,
            points_pos_w=torch.zeros((num_envs, num_bodies, point_num_each_body, 3), device=device, dtype=dtype),
            points_vel_w=torch.zeros((num_envs, num_bodies, point_num_each_body, 3), device=device, dtype=dtype),
            penetration_offset=torch.zeros((num_envs, num_bodies, point_num_each_body, 3), device=device, dtype=dtype),
        )
