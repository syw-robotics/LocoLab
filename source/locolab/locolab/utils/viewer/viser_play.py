# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Browser viewer for headless play.

The simulation keeps running in Isaac Sim. The robot drawn here is
``source_asset``: the MJCF when that is what the task spawns, otherwise the
URDF. Nothing here starts unless a play script constructs :class:`ViserPlayViewer`.
"""

from __future__ import annotations

import io
import logging
import re
import threading
import time
from contextlib import suppress

import numpy as np

from ._viser import viser
from .browser import VIEWER_KEYS_JS, run_javascript, widen_broadcast_window
from .debug_vis import DebugScene, debug_sources, draw_debug
from .terrain import HEIGHT_COLOR_SCALE_M, SOLID_COLOR, paint_terrain
from .urdf import load_robot, urdf_joint_ids
from .utils import select_rows, xyzw_to_wxyz


class _ProbeHandshakeFilter(logging.Filter):
    """Drop the traceback from a client that connects and leaves immediately.

    Viser adds this filter only after the first real HTTP request. A port probe,
    or a browser that opens ``https://127.0.0.1`` against an HTTP server, happens
    first and otherwise logs ``opening handshake failed``.
    """

    def filter(self, record: logging.LogRecord) -> bool:
        return record.getMessage() not in (
            "opening handshake failed",
            "connection rejected (200 OK)",
        )


def _quiet_probe_handshakes() -> None:
    logger = logging.getLogger("websockets.server")
    if not any(isinstance(item, _ProbeHandshakeFilter) for item in logger.filters):
        logger.addFilter(_ProbeHandshakeFilter())


# 50Hz by default.
UPDATE_INTERVAL_S = 1.0 / 50.0
# Viser's built-in WASDQE handler moves the camera at this many meters per second.
DEFAULT_MOVE_SPEED_M_S = 10.0
# Close third-person offset applied when tracking starts or the tracked env changes.
TRACK_OFFSET = np.array([1.8, -1.8, 1.1])
# Playback rates for the Simulation speed buttons. 1x matches wall-clock step time.
SPEEDS = (1 / 8, 1 / 4, 1 / 2, 1.0, 2.0, 4.0, 8.0)
# Viser's own default vertical field of view, in degrees.
DEFAULT_FOV_DEG = 75.0
# Green: this link's collider is enabled. Gray: the source asset has it, this env turned it off.
ENABLED_COLLISION_RGBA = np.array([40, 200, 90, 120], dtype=np.uint8)
DISABLED_COLLISION_RGBA = np.array([170, 176, 184, 70], dtype=np.uint8)
# A client echo of a pose we sent stays within this; a left-drag does not.
_CAMERA_MATCH_M = 5e-3


def _glb_bytes(mesh) -> bytes:
    """Encode one link mesh before the server starts, so the URL means the scene is ready."""
    with io.BytesIO() as buffer:
        mesh.export(buffer, file_type="glb")
        return buffer.getvalue()


def _collision_mesh(mesh, rgba: np.ndarray):
    """Copy a collision mesh with a flat translucent color."""
    import trimesh

    painted = mesh.copy()
    painted.visual = trimesh.visual.TextureVisuals(
        material=trimesh.visual.material.PBRMaterial(
            baseColorFactor=rgba,
            alphaMode="BLEND",
            doubleSided=True,
        )
    )
    return painted


def _enabled_collision_links(spawn, link_names: list[str]) -> set[str]:
    """Links whose collider survives ``spawn.collision_props``.

    Patterns are the same suffixes Isaac appends to the robot prim and matches
    against the whole collider path. A link with no ``collision_enabled`` override stays on.
    """
    enabled = set(link_names)
    props = getattr(spawn, "collision_props", None)
    if not isinstance(props, dict):
        return enabled
    for pattern, fragments in props.items():
        if not isinstance(fragments, (list, tuple)):
            fragments = (fragments,)
        flag = None
        for fragment in fragments:
            value = getattr(fragment, "collision_enabled", None)
            if value is not None:
                flag = bool(value)
        if flag is None:
            continue
        try:
            regex = re.compile("/Robot" + pattern)
        except re.error:
            continue
        for name in link_names:
            probes = (f"/Robot/{name}", f"/Robot/{name}/collisions")
            if any(regex.fullmatch(probe) for probe in probes):
                if flag:
                    enabled.add(name)
                else:
                    enabled.discard(name)
    return enabled


def _seen_camera(position: np.ndarray, look_at: np.ndarray, history: list[tuple[np.ndarray, np.ndarray]]) -> bool:
    return any(
        np.allclose(position, sent_pos, atol=_CAMERA_MATCH_M) and np.allclose(look_at, sent_look, atol=_CAMERA_MATCH_M)
        for sent_pos, sent_look in history
    )


def _robot_arrays(position, orientation, joints):
    """Copy root pose and joint positions to CPU in one transfer."""
    if not hasattr(position, "detach"):
        return (
            np.asarray(position, dtype=np.float64),
            np.asarray(orientation, dtype=np.float64),
            np.asarray(joints, dtype=np.float64),
        )
    import torch

    width_position = position.shape[-1]
    width_orientation = orientation.shape[-1]
    packed = torch.cat((position, orientation, joints), dim=-1).detach().cpu().numpy()
    packed = packed.astype(np.float64, copy=False)
    return (
        packed[..., :width_position],
        packed[..., width_position : width_position + width_orientation],
        packed[..., width_position + width_orientation :],
    )


class ViserPlayViewer:
    """Mirror the URDF robots into a Viser server.

    ``paces`` is true so the play loop leaves realtime sleeping to the Speed control.
    Link meshes are uploaded once and instanced per environment. The Scene panel
    can show all of them, or only the environment selected by the slider.

    Args:
        env: Wrapped or unwrapped Isaac Lab environment. The robot entity is
            ``scene["robot"]`` and must have been spawned from a URDF.
        host: Interface to bind. ``0.0.0.0`` prints ``http://localhost``, which the browser can open.
        port: TCP port. Forward it with ``ssh -L <port>:localhost:<port>``.
        robot_name: Scene entity name of the articulation.
    """

    paces = True

    def __init__(
        self,
        env,
        host: str = "0.0.0.0",
        port: int = 8080,
        robot_name: str = "robot",
    ) -> None:
        self._env = env
        unwrapped = env.unwrapped
        scene = unwrapped.scene
        try:
            self._robot = scene[robot_name]
        except KeyError as exc:
            available = ", ".join(scene.keys()) if hasattr(scene, "keys") else "unknown"
            raise KeyError(f"Scene has no '{robot_name}' entity. Available entities: {available}") from exc

        asset_path = getattr(self._robot.cfg, "source_asset", None) or getattr(
            self._robot.cfg.spawn, "asset_path", None
        )
        if not asset_path:
            raise ValueError(
                "Viser play needs robot.cfg.source_asset "
                "(the spawned MJCF or URDF, or the legacy spawn.asset_path)."
            )
        self._bodies, actuated_names = load_robot(asset_path)
        self._body_glb = [_glb_bytes(mesh) for mesh in self._bodies.meshes]
        enabled_links = _enabled_collision_links(self._robot.cfg.spawn, self._bodies.link_names)
        self._collision_glb = []
        for mesh, link in zip(self._bodies.collision_meshes, self._bodies.collision_mesh_links):
            name = self._bodies.link_names[int(link)]
            rgba = ENABLED_COLLISION_RGBA if name in enabled_links else DISABLED_COLLISION_RGBA
            self._collision_glb.append(_glb_bytes(_collision_mesh(mesh, rgba)))
        self._show_visual = bool(self._body_glb)
        self._show_collision = not self._show_visual and bool(self._collision_glb)
        self._joint_ids = urdf_joint_ids(list(self._robot.joint_names), actuated_names)

        self._num_envs = int(unwrapped.num_envs)
        self._env_idx = 0
        self._show_all = self._num_envs > 1
        self._visible_signature: tuple | None = None

        self._track = False
        # Per client: (position offset, look offset, recent poses we already accepted).
        self._track_view: dict[int, tuple] = {}
        self._camera_pose: np.ndarray | None = None
        self._camera_offset = TRACK_OFFSET.copy()
        self._framed_clients: set[int] = set()
        self._move_speed = DEFAULT_MOVE_SPEED_M_S
        self._fov_deg = DEFAULT_FOV_DEG

        self._paused = threading.Event()
        # The play loop steps only while this is clear. Hold it until a browser connects.
        self._paused.set()
        self._release_on_client = True
        self._reset_requested = threading.Event()
        self._last_update = 0.0
        self._step_dt = float(getattr(unwrapped, "step_dt", 0.02) or 0.02)
        self._speed_index = SPEEDS.index(1.0)
        self._speed = SPEEDS[self._speed_index]
        self._pace_time: float | None = None

        self._terrain_mesh = None
        self._terrain_handle = None
        self._terrain_signature: tuple | None = None
        self._terrain_color = True
        self._terrain_scale = HEIGHT_COLOR_SCALE_M
        self._terrain_rgb = SOLID_COLOR
        self._terrain_lock = threading.Lock()
        self._terrain_timer: threading.Timer | None = None

        self._debug_terms = debug_sources(self._env)
        self._debug_enabled = {
            label: bool(getattr(getattr(obj, "cfg", None), "debug_vis", False)) for label, obj in self._debug_terms
        }
        self._status_html = ""
        self._link_pos = np.zeros((self._num_envs, len(self._bodies.link_names), 3), dtype=np.float32)
        self._link_wxyz = np.zeros((self._num_envs, len(self._bodies.link_names), 4), dtype=np.float32)
        self._link_wxyz[..., 0] = 1.0
        self._body_scales = np.ones(self._num_envs, dtype=np.float32)
        self._visual_handles: list = []
        self._collision_handles: list = []
        self._prepare_ground(scene)

        _quiet_probe_handshakes()
        self._server = viser.ViserServer(host=host, port=port, label="LocoLab")
        widen_broadcast_window(self._server)
        self._debug_scene = DebugScene(self._server)
        try:
            self._add_robots()
            self._add_ground()
            self._build_gui()
            self._server.on_client_connect(self._on_client_connect)
        except Exception:
            self._server.stop()
            raise

    @property
    def paused(self) -> bool:
        return self._paused.is_set()

    def apply_reset(self, env, reset_policy) -> bool:
        """Reset the sim and the policy when the GUI asked for it.

        ``reset_policy`` receives a bool mask with one entry per environment.
        Returns whether a reset ran, so the caller can refresh observations.
        """
        if not self._reset_requested.is_set():
            return False
        self._reset_requested.clear()
        import torch

        env.reset()
        reset_policy(torch.ones(env.unwrapped.num_envs, dtype=torch.bool, device=env.unwrapped.device))
        return True

    def after_step(self) -> bool:
        """Push the latest pose to the browser. Return True while playback is paused.

        While running, sleep so the loop matches ``step_dt / speed``. The play
        script should skip its own ``--realtime`` sleep when ``paces`` is true.
        """
        self.update()
        if self.paused:
            self._pace_time = None
            time.sleep(0.05)
            return True

        now = time.perf_counter()
        previous = self._pace_time
        speed = self._speed
        if previous is None or speed <= 0.0:
            self._pace_time = now
            return False
        remain = self._step_dt / speed - (now - previous)
        if remain > 0.0:
            time.sleep(remain)
        self._pace_time = time.perf_counter()
        return False

    def update(self) -> None:
        """Copy root poses and joint positions into the visible URDF robots."""
        if self._release_on_client and self._server.get_clients():
            self._release_if_waiting()
        now = time.monotonic()
        if now - self._last_update < UPDATE_INTERVAL_S:
            return
        self._last_update = now

        env_idx = min(max(self._env_idx, 0), self._num_envs - 1)
        show_all = self._show_all and self._num_envs > 1
        signature = ("all",) if show_all else ("one", env_idx)
        show_scales = signature != self._visible_signature
        if show_scales:
            self._visible_signature = signature

        selector = slice(None) if show_all else slice(env_idx, env_idx + 1)
        env_ids = range(self._num_envs) if show_all else (env_idx,)
        data = self._robot.data
        position, orientation, joints = _robot_arrays(
            select_rows(data.root_pos_w, selector),
            xyzw_to_wxyz(select_rows(data.root_quat_w, selector)),
            select_rows(data.joint_pos, selector),
        )
        with self._server.atomic():
            self._sync_robots(env_ids, position, orientation, joints, show_scales)
            draw_debug(self._debug_scene, self._debug_terms, self._debug_enabled, selector)

        tracked = position[env_idx] if show_all else position[0]
        if self._camera_pose is None:
            self._frame_camera(tracked)
        self._offer_initial_camera()
        self._follow_camera(tracked)

        shown = f"all {self._num_envs}" if show_all else f"{env_idx} / {self._num_envs - 1}"
        tracking = f"env {env_idx}" if self._track else "off"
        speed_text = f"{self._speed:.3g}x" if self._speed >= 1.0 else f"1/{round(1.0 / self._speed)}x"
        if self._release_on_client:
            state = "Waiting for browser"
        elif self.paused:
            state = "Paused"
        else:
            state = "Running"
        status = (
            "<div style='font-size:0.85em;line-height:1.3;padding:0 1em 0.5em 1em;'>"
            f"<strong>Status:</strong> {state}<br/>"
            f"<strong>Speed:</strong> {speed_text}<br/>"
            f"<strong>Environment:</strong> {shown}<br/>"
            f"<strong>Track:</strong> {tracking}"
            "</div>"
        )
        if status != self._status_html:
            self._status_html = status
            self._status.content = status

    def close(self) -> None:
        with self._terrain_lock:
            timer = self._terrain_timer
            self._terrain_timer = None
            self._terrain_mesh = None
        if timer is not None:
            timer.cancel()
        if getattr(self, "_debug_scene", None) is not None:
            self._debug_scene.clear()
        self._server.stop()

    def _add_robots(self) -> None:
        """Upload each link mesh once, with one instance per environment."""
        self._visual_handles = self._add_mesh_layer("/robots/visual", self._body_glb, self._show_visual)
        self._collision_handles = self._add_mesh_layer(
            "/robots/collision", self._collision_glb, self._show_collision
        )
        self._body_glb = []
        self._collision_glb = []

    def _add_mesh_layer(self, prefix: str, payloads: list[bytes], visible: bool) -> list:
        count = self._num_envs
        identity = np.zeros((count, 4), dtype=np.float32)
        identity[:, 0] = 1.0
        origin = np.zeros((count, 3), dtype=np.float32)
        handles = []
        for index, payload in enumerate(payloads):
            handles.append(
                self._server.scene.add_batched_glb(
                    f"{prefix}/{index}",
                    payload,
                    identity,
                    origin,
                    batched_scales=self._body_scales,
                    lod="off",
                    visible=visible,
                )
            )
        return handles

    def _sync_robots(self, env_ids, position, orientation, joints, show_scales: bool) -> None:
        """Write instance poses. Hidden environments keep scale 0, so their mesh stays put."""
        link_pos, link_wxyz = self._bodies.poses(position, orientation, joints[:, self._joint_ids])
        ids = list(env_ids)
        self._link_pos[ids] = link_pos
        self._link_wxyz[ids] = link_wxyz
        if show_scales:
            self._body_scales[:] = 0.0
            self._body_scales[ids] = 1.0
        layers = (
            (self._visual_handles, self._bodies.mesh_links),
            (self._collision_handles, self._bodies.collision_mesh_links),
        )
        for handles, links in layers:
            for handle, link in zip(handles, links):
                # Copy: the websocket thread serializes this array after the next pose overwrites the buffer.
                handle.batched_positions = self._link_pos[:, link].copy()
                handle.batched_wxyzs = self._link_wxyz[:, link].copy()
                if show_scales:
                    handle.batched_scales = self._body_scales.copy()

    def _prepare_ground(self, scene) -> None:
        """Color the terrain before the server prints its URL."""
        self._terrain_ready = None
        terrain = getattr(scene, "terrain", None)
        terrain_mesh = getattr(terrain, "terrain_mesh", None)
        if terrain_mesh is None:
            return
        self._terrain_mesh = terrain_mesh
        self._terrain_scale = HEIGHT_COLOR_SCALE_M
        self._terrain_color = True
        self._terrain_ready = paint_terrain(
            self._terrain_mesh,
            by_height=True,
            scale_m=self._terrain_scale,
            rgb=self._terrain_rgb,
        )
        self._terrain_signature = ("color", round(self._terrain_scale, 4))

    def _add_ground(self) -> None:
        if self._terrain_ready is None:
            self._server.scene.add_grid(
                "/ground",
                infinite_grid=True,
                fade_distance=50.0,
                shadow_opacity=0.2,
                plane_opacity=0.4,
            )
            return
        # mjlab fixed geometry receives a light shadow and does not cast one.
        # The mesh is replaced only when the Terrain panel changes, never from update().
        self._terrain_handle = self._server.scene.add_mesh_trimesh(
            "/terrain",
            self._terrain_ready,
            cast_shadow=False,
            receive_shadow=0.2,
        )
        self._terrain_ready = None

    def _build_gui(self) -> None:
        self._status = self._server.gui.add_html("")
        tabs = self._server.gui.add_tab_group()
        with tabs.add_tab("Controls", icon=viser.Icon.SETTINGS):
            with self._server.gui.add_folder("Simulation"):
                pause_label = "Play" if self.paused else "Pause"
                pause_icon = viser.Icon.PLAYER_PLAY if self.paused else viser.Icon.PLAYER_PAUSE
                self._pause_button = self._server.gui.add_button(pause_label, icon=pause_icon)

                @self._pause_button.on_click
                def _(_) -> None:
                    if self._paused.is_set():
                        self._release_on_client = False
                        self._paused.clear()
                        self._pause_button.label = "Pause"
                        self._pause_button.icon = viser.Icon.PLAYER_PAUSE
                    else:
                        self._paused.set()
                        self._pause_button.label = "Play"
                        self._pause_button.icon = viser.Icon.PLAYER_PLAY

                reset_button = self._server.gui.add_button("Reset", icon=viser.Icon.RESTORE)

                @reset_button.on_click
                def _(_) -> None:
                    self._reset_requested.set()

                self._server.gui.add_markdown("**Space** pause/resume · **R** reset · **F** track env")
                speed_buttons = self._server.gui.add_button_group("Speed", options=("Slower", "1x", "Faster"))

                @speed_buttons.on_click
                def _(event) -> None:
                    if event.target.value == "Slower":
                        self._set_speed_index(self._speed_index - 1)
                    elif event.target.value == "Faster":
                        self._set_speed_index(self._speed_index + 1)
                    else:
                        self._set_speed_index(SPEEDS.index(1.0))

            with self._server.gui.add_folder("Camera"):
                track = self._server.gui.add_checkbox("Track env", initial_value=False)

                @track.on_update
                def _(event) -> None:
                    self._track = bool(event.target.value)

                speed = self._server.gui.add_slider(
                    "Move speed",
                    min=1.0,
                    max=20.0,
                    step=0.1,
                    initial_value=DEFAULT_MOVE_SPEED_M_S,
                )

                @speed.on_update
                def _(event) -> None:
                    self._move_speed = float(event.target.value)
                    self._apply_move_speed()

                fov = self._server.gui.add_slider(
                    "FOV (°)",
                    min=20.0,
                    max=150.0,
                    step=1.0,
                    initial_value=DEFAULT_FOV_DEG,
                    hint="Vertical field of view.",
                )

                @fov.on_update
                def _(event) -> None:
                    self._apply_fov(float(event.target.value))

            self._build_scene_gui()

        with tabs.add_tab("Visualization", icon=viser.Icon.EYE):
            self._build_robot_vis_gui()
            if self._terrain_mesh is not None:
                self._build_terrain_gui()
        self._apply_fov(self._fov_deg)
        run_javascript(self._server, VIEWER_KEYS_JS.replace("__MOVE_SPEED__", f"{self._move_speed:g}"))
        self._apply_move_speed()

    def _build_scene_gui(self) -> None:
        if self._num_envs <= 1 and not self._debug_terms:
            return
        with self._server.gui.add_folder("Scene"):
            if self._num_envs > 1:
                show_all = self._server.gui.add_checkbox("Show all envs", initial_value=True)

                @show_all.on_update
                def _(event) -> None:
                    self._show_all = bool(event.target.value)

                slider = self._server.gui.add_slider(
                    "Env index",
                    min=0,
                    max=self._num_envs - 1,
                    step=1,
                    initial_value=0,
                )

                @slider.on_update
                def _(event) -> None:
                    self._env_idx = int(event.target.value)

            if not self._debug_terms:
                return
            with self._server.gui.add_folder("debug_viz"):
                for label, _obj in self._debug_terms:
                    checkbox = self._server.gui.add_checkbox(label, initial_value=self._debug_enabled[label])

                    @checkbox.on_update
                    def _(event, key=label) -> None:
                        self._debug_enabled[key] = bool(event.target.value)

    def _build_robot_vis_gui(self) -> None:
        if not self._visual_handles and not self._collision_handles:
            return
        with self._server.gui.add_folder("Robot"):
            if self._visual_handles:
                visual = self._server.gui.add_checkbox("Visual", initial_value=self._show_visual)

                @visual.on_update
                def _(event) -> None:
                    self._show_visual = bool(event.target.value)
                    self._set_layer_visible(self._visual_handles, self._show_visual)

            if self._collision_handles:
                collision = self._server.gui.add_checkbox(
                    "Collision",
                    initial_value=self._show_collision,
                    hint="Green: enabled in this env. Gray: in the URDF/MJCF, but disabled.",
                )

                @collision.on_update
                def _(event) -> None:
                    self._show_collision = bool(event.target.value)
                    self._set_layer_visible(self._collision_handles, self._show_collision)

    def _set_layer_visible(self, handles, visible: bool) -> None:
        for handle in handles:
            handle.visible = visible

    def _build_terrain_gui(self) -> None:
        with self._server.gui.add_folder("Terrain"):
            height_color = self._server.gui.add_checkbox("Height color", initial_value=True)
            self._terrain_scale_slider = self._server.gui.add_slider(
                "Color scale",
                min=0.05,
                max=5.0,
                step=0.05,
                initial_value=HEIGHT_COLOR_SCALE_M,
                hint="Meters from the ground to full red or full blue.",
            )
            self._terrain_color_picker = self._server.gui.add_rgb(
                "Color",
                SOLID_COLOR,
                disabled=True,
                hint="Solid color when height coloring is off.",
            )

            @height_color.on_update
            def _(event) -> None:
                enabled = bool(event.target.value)
                self._terrain_color = enabled
                self._terrain_scale_slider.disabled = not enabled
                self._terrain_color_picker.disabled = enabled
                self._schedule_terrain_appearance(0.0)

            @self._terrain_scale_slider.on_update
            def _(event) -> None:
                self._terrain_scale = float(event.target.value)
                if self._terrain_color:
                    self._schedule_terrain_appearance(0.2)

            @self._terrain_color_picker.on_update
            def _(event) -> None:
                red, green, blue = event.target.value
                self._terrain_rgb = (int(red), int(green), int(blue))
                if not self._terrain_color:
                    self._schedule_terrain_appearance(0.2)

    def _apply_move_speed(self) -> None:
        """Push the slider value. The page script otherwise keeps Viser's own 2 m/s."""
        run_javascript(self._server, f"window.__locolabMoveSpeed = {float(self._move_speed)};")

    def _release_if_waiting(self) -> None:
        """Start playback on the first browser. Later clients leave the pause button alone."""
        if not self._release_on_client:
            return
        self._release_on_client = False
        self._paused.clear()
        button = getattr(self, "_pause_button", None)
        if button is None:
            return
        button.label = "Pause"
        button.icon = viser.Icon.PLAYER_PAUSE

    def _set_speed_index(self, index: int) -> None:
        self._speed_index = min(max(index, 0), len(SPEEDS) - 1)
        self._speed = SPEEDS[self._speed_index]
        self._pace_time = None

    def _apply_fov(self, degrees: float) -> None:
        self._fov_deg = degrees
        radians = float(np.deg2rad(degrees))
        self._server.initial_camera.fov = radians
        for client in self._server.get_clients().values():
            try:
                client.camera.fov = radians
            except AssertionError:
                continue

    def _schedule_terrain_appearance(self, delay_s: float) -> None:
        """Coalesce color changes so a slider drag uploads the mesh once."""
        with self._terrain_lock:
            if self._terrain_timer is not None:
                self._terrain_timer.cancel()
            timer = threading.Timer(delay_s, self._apply_terrain_appearance)
            timer.daemon = True
            self._terrain_timer = timer
            timer.start()

    def _terrain_appearance(self) -> tuple:
        if self._terrain_color:
            return ("color", round(self._terrain_scale, 4))
        return ("solid", self._terrain_rgb)

    def _apply_terrain_appearance(self) -> None:
        """Match the terrain panel. Geometry is uploaded once per change, not every frame.

        Height coloring and a solid color share one mesh upload. The only
        difference is whether each vertex has its own height color.
        """
        with self._terrain_lock:
            mesh = self._terrain_mesh
            if mesh is None:
                return
            signature = self._terrain_appearance()
            if signature == self._terrain_signature:
                return
            by_height = signature[0] == "color"
            scale = self._terrain_scale
            rgb = self._terrain_rgb
        displayed = paint_terrain(mesh, by_height=by_height, scale_m=scale, rgb=rgb)
        with self._terrain_lock:
            if self._terrain_mesh is None or self._terrain_appearance() != signature:
                return
            if self._terrain_handle is not None:
                self._terrain_handle.remove()
            self._terrain_handle = self._server.scene.add_mesh_trimesh(
                "/terrain",
                displayed,
                cast_shadow=False,
                receive_shadow=0.2,
            )
            self._terrain_signature = signature

    def _on_client_connect(self, client: viser.ClientHandle) -> None:
        with suppress(AssertionError):
            client.camera.fov = float(np.deg2rad(self._fov_deg))
        self._offer_initial_camera()
        self._release_if_waiting()

    def _frame_camera(self, target: np.ndarray) -> None:
        """Record a close view of one robot. ``initial_camera`` only affects clients that connect later."""
        target = np.asarray(target, dtype=np.float64).reshape(3)
        self._camera_pose = target.copy()
        self._camera_offset = TRACK_OFFSET.copy()
        self._server.initial_camera.position = target + self._camera_offset
        self._server.initial_camera.look_at = target

    def _offer_initial_camera(self) -> None:
        """Move each browser onto the opening view once. Later orbit and track are left alone."""
        if self._camera_pose is None:
            return
        position = self._camera_pose + self._camera_offset
        for client in self._server.get_clients().values():
            if client.client_id in self._framed_clients:
                continue
            if self._place_camera(client, position, self._camera_pose):
                self._framed_clients.add(client.client_id)

    def _place_camera(self, client, position: np.ndarray, look_at: np.ndarray) -> bool:
        try:
            client.camera.position = position
            client.camera.look_at = look_at
        except AssertionError:
            return False
        return True

    def _follow_camera(self, target: np.ndarray) -> None:
        """Keep the robot in frame, and keep a left-drag as the new orbit.

        Poses we already sent are ignored, so a late camera echo is not a drag.
        A pose we did not send is the mouse; it becomes the offset and is not overwritten.
        """
        if not self._track:
            self._track_view.clear()
            return
        target = np.asarray(target, dtype=np.float64).reshape(3)
        for client in self._server.get_clients().values():
            self._follow_client(client, target)

    def _follow_client(self, client, target: np.ndarray) -> None:
        client_id = client.client_id
        view = self._track_view.get(client_id)
        if view is None:
            self._write_track_view(client, target, TRACK_OFFSET, np.zeros(3), [])
            return
        pos_off, look_off, history = view
        try:
            reported_pos = np.asarray(client.camera.position, dtype=np.float64)
            reported_look = np.asarray(client.camera.look_at, dtype=np.float64)
        except AssertionError:
            self._write_track_view(client, target, pos_off, look_off, history)
            return
        if not _seen_camera(reported_pos, reported_look, history):
            self._track_view[client_id] = (
                reported_pos - target,
                reported_look - target,
                [(reported_pos.copy(), reported_look.copy())],
            )
            return
        self._write_track_view(client, target, pos_off, look_off, history)

    def _write_track_view(self, client, target, pos_off, look_off, history: list) -> None:
        position = target + pos_off
        look_at = target + look_off
        if not self._place_camera(client, position, look_at):
            return
        history.append((position.copy(), look_at.copy()))
        del history[:-6]
        self._track_view[client.client_id] = (np.array(pos_off, copy=True), np.array(look_off, copy=True), history)
