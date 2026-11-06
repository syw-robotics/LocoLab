# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Transient world-frame velocity-push arrows shared by native and browser viewers."""

import torch

import isaaclab.utils.math as math_utils

from locolab.utils.debug_viz.push.push_debug_vis_cfg import PushDebugVisCfg


class PushDebugVis:
    """Fade the last linear velocity increment using the shared viewer configuration."""

    def __init__(self, env, asset_name, cfg: PushDebugVisCfg):
        self.env = env
        self.robot = env.scene[asset_name]
        self.cfg = cfg
        self.debug_vis = True
        self.params = None
        self.duration_steps = max(1, round(cfg.duration_s / env.step_dt))
        self.velocity_w = torch.zeros(env.num_envs, 3, device=self.robot.device)
        self.expires = torch.full((env.num_envs,), -1, device=self.robot.device, dtype=torch.long)
        self.episode_start = torch.zeros_like(self.expires)
        self.markers = None
        self._debug_vis_handle = None
        registry = getattr(getattr(env, "sim", None), "vis_marker_registry", None)
        if registry is not None:
            self._debug_vis_handle = registry.add_debug_vis_callback(self)

    def set_debug_vis(self, enabled):
        self.debug_vis = enabled
        if self.params is not None:
            self.params["debug_vis"] = enabled
        registry = getattr(getattr(self.env, "sim", None), "vis_marker_registry", None)
        if registry is not None:
            if enabled and self._debug_vis_handle is None:
                self._debug_vis_handle = registry.add_debug_vis_callback(self)
            elif not enabled:
                registry.clear_debug_vis_callback(self)
        if not enabled:
            self.expires.fill_(-1)
            if self.markers is not None:
                self.markers.set_visibility(False)

    def record(self, env_ids, velocity_w):
        self.velocity_w[env_ids] = velocity_w[:, :3]
        self.expires[env_ids] = self.env.common_step_counter + self.duration_steps
        self.episode_start[env_ids] = self.env.common_step_counter - self.env.episode_length_buf[env_ids]

    def active(self, selector):
        step = self.env.common_step_counter
        return (
            (self.expires[selector] > step)
            & (self.episode_start[selector] == step - self.env.episode_length_buf[selector])
            & (self.velocity_w[selector].norm(dim=-1) > 1.0e-4)
        )

    def opacity(self, selector):
        remaining = (self.expires[selector] - self.env.common_step_counter) / self.duration_steps
        return remaining.clamp(0.0, 1.0) * self.active(selector)

    def _debug_vis_callback(self, event):
        if not self.debug_vis or not self.robot.is_initialized:
            return
        opacity = self.opacity(slice(None))
        active = opacity > 0.0
        if not active.any():
            if self.markers is not None:
                self.markers.set_visibility(False)
            return
        if self.markers is None:
            from locolab.utils.debug_viz.push.push_debug_markers import PushMarkers

            self.markers = PushMarkers(self.cfg.visualizer_cfg.copy())
        # Dense env-major batches let Newton's native viewer filter selected environments.
        velocity = self.velocity_w
        yaw = torch.atan2(velocity[:, 1], velocity[:, 0])
        pitch = -torch.atan2(velocity[:, 2], velocity[:, :2].norm(dim=-1))
        orientation = math_utils.quat_from_euler_xyz(torch.zeros_like(yaw), pitch, yaw)
        scale = torch.ones_like(velocity)
        scale[:, 0] = velocity.norm(dim=-1)
        scale *= active[:, None]
        position = self.robot.data.root_link_pos_w.torch
        self.markers.set_visibility(True)
        self.markers.visualize(position, orientation, scale, opacities=opacity)


def get_push_debug_vis(env, asset_name, cfg: PushDebugVisCfg | None = None):
    """Allocate state only for environments that request push visualization."""
    if not hasattr(env, "_push_debug_vis"):
        env._push_debug_vis = {}
    if asset_name not in env._push_debug_vis:
        env._push_debug_vis[asset_name] = PushDebugVis(
            env, asset_name, cfg if cfg is not None else PushDebugVisCfg()
        )
    return env._push_debug_vis[asset_name]
