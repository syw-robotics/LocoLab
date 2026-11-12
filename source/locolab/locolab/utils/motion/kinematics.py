"""Torch FK built once from the same MJCF as the simulated robot."""

from pathlib import Path

import mujoco
import numpy as np
import torch
from torch import nn
from z_rl.models.dynamics import quat_apply, quat_mul, rotvec_to_quat

G1_MJCF = Path(__file__).parents[2] / "assets/robots/unitree/G1/xml/g1_29dof_pelvis_base.xml"


class Kinematics(nn.Module):
    def __init__(self, xml_path, joint_names, body_names):
        super().__init__()
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        if model.body(1).name != "pelvis":
            raise ValueError("Tracking WM requires the pelvis-base G1 asset")
        self.parents = tuple(int(i) - 1 for i in model.body_parentid[1:])
        self.joints = []
        axes, anchors = [], []
        for body in range(1, model.nbody):
            jid = model.body_jntadr[body]
            if body == 1 or model.body_jntnum[body] == 0:
                self.joints.append(-1)
                axes.append([0.0, 0.0, 0.0])
                anchors.append([0.0, 0.0, 0.0])
            else:
                if model.body_jntnum[body] != 1 or model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
                    raise ValueError("Expected one hinge per G1 moving link")
                self.joints.append(joint_names.index(model.joint(jid).name))
                axes.append(model.jnt_axis[jid].copy())
                anchors.append(model.jnt_pos[jid].copy())
        self.register_buffer("offset", torch.tensor(model.body_pos[1:].copy()))
        self.register_buffer("rotation", torch.tensor(model.body_quat[1:, [1, 2, 3, 0]].copy()))
        self.register_buffer("axes", torch.tensor(np.asarray(axes)))
        self.register_buffer("anchors", torch.tensor(np.asarray(anchors)))
        self.register_buffer("selected", torch.tensor([model.body(name).id - 1 for name in body_names]))
        self.float()

    @torch.no_grad()
    def motion_poses(self, motion, batch_size=8192):
        """Precompute reference poses in bounded batches, independent of sim env count."""
        positions, rotations = [], []
        names = ("root_pos", "root_quat", "root_lin_vel", "root_ang_vel", "joint_pos", "joint_vel")
        for start in range(0, len(motion.joint_pos), batch_size):
            state = torch.cat([getattr(motion, name)[start : start + batch_size] for name in names], -1)
            p, q, _, _ = self(state)
            positions.append(p)
            rotations.append(q)
        return torch.cat(positions), torch.cat(rotations)

    def forward(self, state):
        position, rotation = [state[..., :3]], [state[..., 3:7]]
        linear, angular = [state[..., 7:10]], [state[..., 10:13]]
        joints = (state.shape[-1] - 13) // 2
        for body in range(1, len(self.parents)):
            parent = self.parents[body]
            base_p = position[parent] + quat_apply(rotation[parent], self.offset[body])
            base_q = quat_mul(rotation[parent], self.rotation[body].expand_as(rotation[parent]))
            jid = self.joints[body]
            if jid >= 0:
                q = quat_mul(base_q, rotvec_to_quat(state[..., 13 + jid, None] * self.axes[body]))
                anchor = base_p + quat_apply(base_q, self.anchors[body])
                p = anchor - quat_apply(q, self.anchors[body])
                w = angular[parent] + quat_apply(base_q, self.axes[body]) * state[..., 13 + joints + jid, None]
                v = (
                    linear[parent]
                    + torch.linalg.cross(angular[parent], anchor - position[parent])
                    + torch.linalg.cross(w, p - anchor)
                )
            else:
                p, q, w = base_p, base_q, angular[parent]
                v = linear[parent] + torch.linalg.cross(w, p - position[parent])
            position.append(p)
            rotation.append(q)
            linear.append(v)
            angular.append(w)
        return tuple(
            torch.stack(items, -2).index_select(-2, self.selected) for items in (position, rotation, linear, angular)
        )
