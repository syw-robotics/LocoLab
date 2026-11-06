# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Load qpos motion clips and resample them to the control rate.

Converted Lafan clips store root position, a wxyz root quaternion, and hinge
joints. Body poses are filled later from the simulated robot.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import torch

from .math import finite_difference_ang_vel, finite_difference_vel, resample_motion

_ROOT_QPOS_NAMES = (
    "root_tx",
    "root_ty",
    "root_tz",
    "root_qw",
    "root_qx",
    "root_qy",
    "root_qz",
)


class MotionLoader:
    """One clip or a directory of clips packed at ``output_dt`` seconds on ``device``.

    Root trajectories have shape ``(T, 3)`` / ``(T, 4)``; joint trajectories
    have shape ``(T, J)`` in the supplied ``joint_names`` order. Root poses and
    velocities use the clip's world frame; quaternions are exposed as xyzw.
    """

    def __init__(
        self,
        motion_file: str,
        joint_names: tuple[str, ...] | list[str],
        output_dt: float,
        device: str | torch.device,
    ) -> None:
        path = Path(motion_file).expanduser().resolve()
        if path.is_dir():
            files = sorted(path.rglob("*.npz"))
            if not files:
                raise FileNotFoundError(f"No .npz motion clips found in {path}")
            clips = [MotionLoader(str(p), joint_names, output_dt, device) for p in files]
            self.motion_file = str(path)
            self.input_dt, self.output_dt = clips[0].input_dt, float(output_dt)
            self.device, self.joint_names = torch.device(device), tuple(joint_names)
            for name in ("root_pos", "root_quat", "joint_pos", "root_lin_vel", "joint_vel", "root_ang_vel"):
                setattr(self, name, torch.cat([getattr(clip, name) for clip in clips]))
            self.time_step_total = len(self.joint_pos)
            self._set_clips([str(p) for p in files], [clip.time_step_total for clip in clips])
            self.body_pos_w = self.body_quat_w = self.body_lin_vel_w = self.body_ang_vel_w = None
            return
        if not path.is_file():
            raise FileNotFoundError(f"Motion file not found: {path}")
        if output_dt <= 0.0:
            raise ValueError(f"output_dt must be positive, got {output_dt}.")

        with np.load(path, allow_pickle=False) as payload:
            if "qpos" not in payload:
                raise KeyError(f"Motion file {path} does not contain a qpos array.")
            qpos = np.asarray(payload["qpos"], dtype=np.float32)
        if qpos.ndim != 2 or qpos.shape[1] < 8:
            raise ValueError(f"Expected qpos with shape (T, >=8), got {qpos.shape}.")

        # Columns must already match the robot: validate their order rather than reorder them.
        manifest = _load_manifest(path)
        input_dt, qpos_names = _motion_timing(manifest, qpos.shape[1])
        expected_names = list(_ROOT_QPOS_NAMES) + list(joint_names)
        if qpos_names is not None and list(qpos_names) != expected_names:
            raise ValueError(
                f"Motion qpos names in {path} do not match the robot joint order.\n"
                f"Expected: {expected_names}\n"
                f"Got: {list(qpos_names)}"
            )
        if qpos.shape[1] != len(expected_names):
            raise ValueError(
                f"Expected qpos width {len(expected_names)} (7 root + {len(joint_names)} joints), got {qpos.shape[1]}."
            )
        if qpos.shape[0] < 3:
            raise ValueError(f"Motion {path} has {qpos.shape[0]} frames; need at least 3.")

        self.motion_file = str(path)
        self.input_dt = input_dt
        self.output_dt = float(output_dt)
        self.device = torch.device(device)
        self.joint_names = tuple(joint_names)

        # Resample on CPU before transferring the resulting trajectories to the target device.
        qpos_t = torch.from_numpy(qpos)
        root_pos = qpos_t[:, 0:3]
        # any4hdmi stores the root quaternion as wxyz. Isaac Lab uses xyzw.
        root_quat_xyzw = qpos_t[:, [4, 5, 6, 3]]
        joint_pos = qpos_t[:, 7:]
        root_pos, root_quat_xyzw, joint_pos = resample_motion(
            root_pos, root_quat_xyzw, joint_pos, input_dt, self.output_dt
        )

        self.root_pos = root_pos.to(device=self.device)
        self.root_quat = root_quat_xyzw.to(device=self.device)
        self.joint_pos = joint_pos.to(device=self.device)
        # Differentiate at the output rate so velocities match the resampled positions.
        self.root_lin_vel = finite_difference_vel(self.root_pos, self.output_dt)
        self.joint_vel = finite_difference_vel(self.joint_pos, self.output_dt)
        self.root_ang_vel = finite_difference_ang_vel(self.root_quat, self.output_dt)
        self.time_step_total = int(self.joint_pos.shape[0])
        self._set_clips([str(path)], [self.time_step_total])

        # qpos clips omit body trajectories; the simulator supplies them after loading.
        self.body_pos_w: torch.Tensor | None = None
        self.body_quat_w: torch.Tensor | None = None
        self.body_lin_vel_w: torch.Tensor | None = None
        self.body_ang_vel_w: torch.Tensor | None = None

    def _set_clips(self, files: list[str], lengths: list[int]) -> None:
        self.motion_files = files
        self.clip_lengths = lengths
        lengths_t = torch.tensor(lengths, device=self.device)
        self.clip_ends = lengths_t.cumsum(0)
        self.clip_starts = self.clip_ends - lengths_t
        self.frame_end = self.clip_ends.repeat_interleave(lengths_t)
        self.frame_clip = torch.arange(len(files), device=self.device).repeat_interleave(lengths_t)

    def sampling_bins(self, frames_per_bin: int, kernel_size: int):
        """Partition every clip independently; smoothing neighbors stay in the same clip."""
        starts, ends, neighbors = [], [], []
        offset = 0
        for length in self.clip_lengths:
            local = list(range(0, length, frames_per_bin))
            first, last = len(starts), len(starts) + len(local) - 1
            starts.extend(offset + frame for frame in local)
            ends.extend(offset + min(frame + frames_per_bin, length) for frame in local)
            neighbors.extend([min(index + k, last) for k in range(kernel_size)] for index in range(first, last + 1))
            offset += length
        starts = torch.tensor(starts, device=self.device)
        ends = torch.tensor(ends, device=self.device)
        frame_bin = torch.arange(len(starts), device=self.device).repeat_interleave(ends - starts)
        return starts, ends, frame_bin, torch.tensor(neighbors, device=self.device)

    def set_body_poses(self, body_pos_w: torch.Tensor, body_quat_w: torch.Tensor) -> None:
        """Differentiate each clip separately, including both endpoints."""
        positions = body_pos_w.split(self.clip_lengths)
        rotations = body_quat_w.split(self.clip_lengths)
        self.set_body_state(
            body_pos_w,
            body_quat_w,
            torch.cat([finite_difference_vel(p, self.output_dt) for p in positions]),
            torch.cat([finite_difference_ang_vel(q, self.output_dt) for q in rotations]),
        )

    def evaluation_starts(self, horizon: int, segments_per_clip: int):
        """Evenly spaced deterministic segments, including clips shorter than the horizon."""
        starts, offset = [], 0
        for length in self.clip_lengths:
            starts.append(
                torch.linspace(0, max(0, length - horizon), segments_per_clip, device=self.device).long() + offset
            )
            offset += length
        clip = torch.arange(len(self.motion_files), device=self.device).repeat_interleave(segments_per_clip)
        return clip, torch.cat(starts)

    def set_body_state(
        self,
        body_pos_w: torch.Tensor,
        body_quat_w: torch.Tensor,
        body_lin_vel_w: torch.Tensor,
        body_ang_vel_w: torch.Tensor,
    ) -> None:
        """Attach world-frame body trajectories computed for the resampled clip.

        Positions and velocities have shape ``(T, B, 3)``; xyzw orientations
        have shape ``(T, B, 4)``, with the same body order across all tensors.
        """
        if body_pos_w.shape[0] != self.time_step_total:
            raise ValueError(
                f"Body trajectory length {body_pos_w.shape[0]} does not match {self.time_step_total} motion frames."
            )
        self.body_pos_w = body_pos_w
        self.body_quat_w = body_quat_w
        self.body_lin_vel_w = body_lin_vel_w
        self.body_ang_vel_w = body_ang_vel_w


def _load_manifest(motion_file: Path) -> dict | None:
    """Find the nearest ancestor manifest shared by the dataset's nested clips."""
    current = motion_file.parent
    for candidate in (current, *current.parents):
        manifest_path = candidate / "manifest.json"
        if manifest_path.is_file():
            return json.loads(manifest_path.read_text(encoding="utf-8"))
        if candidate == candidate.parent:
            break
    return None


def _motion_timing(manifest: dict | None, qpos_dim: int) -> tuple[float, list[str] | None]:
    """Read frame spacing in seconds and optional column names from the manifest."""
    if manifest is None:
        raise FileNotFoundError(
            "Could not find manifest.json above the motion file. "
            "Point motion_file at a clip inside the converted Lafan dataset."
        )
    timestep = float(manifest["timestep"])
    if timestep <= 0.0:
        raise ValueError(f"Manifest timestep must be positive, got {timestep}.")
    names = manifest.get("qpos_names")
    if names is not None and len(names) != qpos_dim:
        raise ValueError(f"Manifest lists {len(names)} qpos names, but the motion has width {qpos_dim}.")
    return timestep, None if names is None else list(names)
