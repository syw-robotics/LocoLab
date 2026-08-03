from __future__ import annotations

from pathlib import Path

import numpy as np
import torch

from isaaclab.utils import math as math_utils


class AmpMotionReference:
    """Loads AMP reference motions and returns reference body-state observations.

    This class is the LocoLab-side provider for ``obs["amp_reference"]``.
    It does not train anything. Its job is only to sample expert motion frames and
    convert them into the same 195-dim body-state format as the simulated robot.
    """

    def __init__(
        self,
        motion_dir: str,
        amp_body_names: list[str],
        amp_anchor_name: str,
        motion_body_names: list[str],
        device: str,
    ) -> None:
        self.device = device
        self.motion_dir = Path(motion_dir)

        # AMP only compares a selected subset, so cache the indices once.
        self.amp_body_ids = torch.tensor(
            [motion_body_names.index(name) for name in amp_body_names],
            dtype=torch.long,
            device=device,
        )
        self.amp_anchor_id = motion_body_names.index(amp_anchor_name)

        # Each item contains one .npz motion already moved to the training device.
        self.motions = self._load_motions(self.motion_dir)

        # Per-environment sampling state. For env i:
        #   motion_ids[i]   -> which expert motion it follows
        #   start_frames[i] -> where this episode starts inside that motion
        self.motion_ids = None
        self.start_frames = None

    def _load_motions(self, motion_dir: Path) -> list[dict]:
        """Load all AMP .npz files under ``motion_dir``."""
        motion_paths = sorted(motion_dir.glob("*.npz"))
        if len(motion_paths) == 0:
            raise FileNotFoundError(f"No AMP motion files found in: {motion_dir}")

        motions = []
        for motion_path in motion_paths:
            data = np.load(motion_path)

            # Expected motion arrays:
            #   body_pos_w      [T, num_motion_bodies, 3]
            #   body_quat_w     [T, num_motion_bodies, 4]
            #   body_lin_vel_w  [T, num_motion_bodies, 3]
            #   body_ang_vel_w  [T, num_motion_bodies, 3]
            # The suffix ``_w`` means world frame.
            motion = {
                "path": str(motion_path),
                "fps": float(np.asarray(data["fps"]).reshape(-1)[0]),
                "body_pos_w": torch.as_tensor(data["body_pos_w"], dtype=torch.float32, device=self.device),
                "body_quat_w": torch.as_tensor(data["body_quat_w"], dtype=torch.float32, device=self.device),
                "body_lin_vel_w": torch.as_tensor(data["body_lin_vel_w"], dtype=torch.float32, device=self.device),
                "body_ang_vel_w": torch.as_tensor(data["body_ang_vel_w"], dtype=torch.float32, device=self.device),
            }
            motion["num_frames"] = motion["body_pos_w"].shape[0]
            motions.append(motion)

        return motions

    def get_state(self, env) -> torch.Tensor:
        """Return reference AMP state with shape ``[num_envs, num_amp_bodies * 15]``."""
        num_envs = env.num_envs
        self._ensure_buffers(num_envs)

        # When an IsaacLab env starts a new episode, its episode_length_buf is 0.
        # Resample a new expert clip/start frame for those envs.
        reset_env_ids = torch.nonzero(env.episode_length_buf == 0, as_tuple=False).squeeze(-1)
        if reset_env_ids.numel() > 0:
            self._resample(reset_env_ids)

        num_bodies = self.amp_body_ids.numel()
        output = torch.zeros(num_envs, num_bodies * 15, device=self.device)

        # Different envs may be following different motion files. Group them by
        # motion_id so we can index each .npz tensor in batches.
        for motion_id, motion in enumerate(self.motions):
            env_ids = torch.nonzero(self.motion_ids == motion_id, as_tuple=False).squeeze(-1)
            if env_ids.numel() == 0:
                continue

            # Convert simulator elapsed time to reference-motion frame offset.
            # frame = random_start + elapsed_seconds * reference_fps
            frame_offset = torch.round(
                env.episode_length_buf[env_ids].float() * env.step_dt * motion["fps"]
            ).long()

            # WalkOnly clips are loops, so modulo lets the reference wrap around.
            frame_ids = (self.start_frames[env_ids] + frame_offset) % motion["num_frames"]

            output[env_ids] = self._build_amp_state(motion, frame_ids)

        return output

    def _ensure_buffers(self, num_envs: int) -> None:
        """Allocate per-env sampling buffers when env count is first known."""
        if self.motion_ids is not None and self.motion_ids.shape[0] == num_envs:
            return

        self.motion_ids = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        self.start_frames = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        self._resample(torch.arange(num_envs, device=self.device))

    def _resample(self, env_ids: torch.Tensor) -> None:
        """Choose a random motion file and random start frame for selected envs."""
        sampled_motion_ids = torch.randint(
            low=0,
            high=len(self.motions),
            size=(env_ids.numel(),),
            device=self.device,
        )
        self.motion_ids[env_ids] = sampled_motion_ids

        for motion_id, motion in enumerate(self.motions):
            selected = env_ids[sampled_motion_ids == motion_id]
            if selected.numel() == 0:
                continue

            max_start = max(motion["num_frames"] - 1, 1)
            self.start_frames[selected] = torch.randint(
                low=0,
                high=max_start,
                size=(selected.numel(),),
                device=self.device,
            )

    def _build_amp_state(self, motion: dict, frame_ids: torch.Tensor) -> torch.Tensor:
        """Convert reference motion frames into AMP body-state features.

        For every selected body, AMP uses 15 numbers:
            local position       3
            local orientation    6  first two columns of rotation matrix
            local linear vel     3
            local angular vel    3

        With 13 selected bodies, this returns 13 * 15 = 195 numbers per env.
        """
        # Pick the requested frames, then pick the AMP body subset.
        body_pos_w = motion["body_pos_w"][frame_ids][:, self.amp_body_ids, :]
        body_quat_w = motion["body_quat_w"][frame_ids][:, self.amp_body_ids, :]
        body_lin_vel_w = motion["body_lin_vel_w"][frame_ids][:, self.amp_body_ids, :]
        body_ang_vel_w = motion["body_ang_vel_w"][frame_ids][:, self.amp_body_ids, :]

        # The anchor is the local coordinate frame AMP compares bodies in.
        # For G1 we currently use torso_link.
        anchor_pos_w = motion["body_pos_w"][frame_ids][:, self.amp_anchor_id, :]
        anchor_quat_w = motion["body_quat_w"][frame_ids][:, self.amp_anchor_id, :]

        num_envs, num_bodies = body_pos_w.shape[:2]

        # Expand anchor from [N, 3/4] to [N, num_bodies, 3/4] so each body can
        # be transformed relative to the same anchor frame.
        anchor_pos_w = anchor_pos_w.unsqueeze(1).expand(-1, num_bodies, -1)
        anchor_quat_w = anchor_quat_w.unsqueeze(1).expand(-1, num_bodies, -1)

        # Convert body pose from world frame to anchor-local frame.
        body_pos_b, body_quat_b = math_utils.subtract_frame_transforms(
            anchor_pos_w.reshape(-1, 3),
            anchor_quat_w.reshape(-1, 4),
            body_pos_w.reshape(-1, 3),
            body_quat_w.reshape(-1, 4),
        )

        body_pos_b = body_pos_b.reshape(num_envs, num_bodies, 3)
        body_quat_b = body_quat_b.reshape(num_envs, num_bodies, 4)

        # AMP_mjlab-style orientation feature: use the first two rotation-matrix
        # columns instead of raw quaternion. This gives 6 continuous numbers.
        body_ori_b = math_utils.matrix_from_quat(body_quat_b)[..., :, :2].reshape(num_envs, num_bodies, 6)

        # Velocities are expressed in each body local frame, matching policy AMP state.
        body_lin_vel_b = math_utils.quat_apply_inverse(body_quat_w, body_lin_vel_w)
        body_ang_vel_b = math_utils.quat_apply_inverse(body_quat_w, body_ang_vel_w)

        return torch.cat(
            [
                body_pos_b.reshape(num_envs, -1),
                body_ori_b.reshape(num_envs, -1),
                body_lin_vel_b.reshape(num_envs, -1),
                body_ang_vel_b.reshape(num_envs, -1),
            ],
            dim=-1,
        )
