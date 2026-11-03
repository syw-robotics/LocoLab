# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Browser viewer for headless play.

The simulation keeps running in Isaac Lab. mjviser draws the MJCF saved
alongside the spawned USD, or an explicitly supplied MJCF source. Nothing here
starts unless a play script constructs :class:`ViserPlayViewer`.
"""

from __future__ import annotations

import logging
import threading
import time

from ._viser import viser
from .debug_vis import DebugScene, debug_sources, draw_debug
from .mjviser_robot import MjviserRobot
from .terrain import HEIGHT_COLOR_SCALE_M, SOLID_COLOR, paint_terrain


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
# Playback rates for the Simulation speed buttons. 1x matches wall-clock step time.
SPEEDS = (1 / 8, 1 / 4, 1 / 2, 1.0, 2.0, 4.0, 8.0)


class ViserPlayViewer:
    """Mirror simulated robots through mjviser into a Viser server.

    ``paces`` is true so the play loop leaves realtime sleeping to the Speed control.
    Link meshes are uploaded once and instanced per environment. The Scene panel
    can show all of them, or only the environment selected by the slider.

    Args:
        env: Wrapped or unwrapped Isaac Lab environment. The robot entity is
            ``scene["robot"]`` and must have an assetx bundle or MJCF source.
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

        self._num_envs = int(unwrapped.num_envs)
        self._robot_view = MjviserRobot(self._robot, self._num_envs)
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
        self._prepare_ground(scene)

        _quiet_probe_handshakes()
        self._server = viser.ViserServer(host=host, port=port, label="LocoLab")
        self._debug_scene = DebugScene(self._server)
        try:
            self._robot_view.create_scene(self._server)
            self._add_ground()
            self._build_gui()
            self._server.on_client_connect(lambda _client: self._release_if_waiting())
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
        """Copy simulated body poses into the visible robots."""
        if self._release_on_client and self._server.get_clients():
            self._release_if_waiting()
        now = time.monotonic()
        if now - self._last_update < UPDATE_INTERVAL_S:
            return
        self._last_update = now

        scene = self._robot_view.scene
        env_idx = scene.env_idx
        show_all = not scene.show_only_selected
        selector = slice(None) if show_all else slice(env_idx, env_idx + 1)
        with self._server.atomic():
            self._robot_view.update(selector, env_idx)
            self._debug_scene.set_ghost_hulls(self._robot_view.visual_hulls)
            self._debug_scene.distance_lod = self._robot_view.distance_lod
            draw_debug(self._debug_scene, self._debug_terms, self._debug_enabled, selector)

        shown = f"all {self._num_envs}" if show_all else f"{env_idx} / {self._num_envs - 1}"
        tracking = f"env {env_idx}" if scene.camera_tracking_enabled else "off"
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
                "/fixed_bodies/ground",
                infinite_grid=True,
                fade_distance=50.0,
                shadow_opacity=0.2,
                plane_opacity=0.4,
            )
            return
        # mjlab fixed geometry receives a light shadow and does not cast one.
        # The mesh is replaced only when the Terrain panel changes, never from update().
        self._terrain_handle = self._server.scene.add_mesh_trimesh(
            "/fixed_bodies/terrain",
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

                speed_buttons = self._server.gui.add_button_group("Speed", options=("Slower", "1x", "Faster"))

                @speed_buttons.on_click
                def _(event) -> None:
                    if event.target.value == "Slower":
                        self._set_speed_index(self._speed_index - 1)
                    elif event.target.value == "Faster":
                        self._set_speed_index(self._speed_index + 1)
                    else:
                        self._set_speed_index(SPEEDS.index(1.0))

            self._robot_view.scene.create_scene_gui()
            self._build_debug_gui()

        with tabs.add_tab("Visualization", icon=viser.Icon.EYE):
            self._build_robot_vis_gui()
            if self._terrain_mesh is not None:
                self._build_terrain_gui()

    def _build_debug_gui(self) -> None:
        if not self._debug_terms:
            return
        with self._server.gui.add_folder("debug_viz"):
            for label, _obj in self._debug_terms:
                checkbox = self._server.gui.add_checkbox(label, initial_value=self._debug_enabled[label])

                @checkbox.on_update
                def _(event, key=label) -> None:
                    self._debug_enabled[key] = bool(event.target.value)

    def _build_robot_vis_gui(self) -> None:
        with self._server.gui.add_folder("Robot"):
            visual = self._server.gui.add_checkbox("Visual", initial_value=self._robot_view.has_visual)
            collision = self._server.gui.add_checkbox(
                "Collision", initial_value=not self._robot_view.has_visual and self._robot_view.has_collision
            )
            hull = self._server.gui.add_checkbox(
                "Convex hull", initial_value=False,
                hint="Use the same convex-hull visual meshes for the robot and browser ghost.",
            )
            distance_lod = self._server.gui.add_checkbox(
                "Distance LOD", initial_value=True,
                hint="Reduce distant Visual and ghost meshes. Disable to keep full detail at every distance.",
            )

            @distance_lod.on_update
            def _(_event) -> None:
                self._robot_view.set_distance_lod(distance_lod.value)

            def update_layers(_event) -> None:
                self._robot_view.set_visibility(visual.value, collision.value, hull.value)

            for checkbox in (visual, collision, hull):
                checkbox.on_update(update_layers)

            self._server.gui.add_markdown(
                "Collision: green enabled · gray disabled. Convex hull replaces Visual meshes."
            )

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
                "/fixed_bodies/terrain",
                displayed,
                cast_shadow=False,
                receive_shadow=0.2,
            )
            self._terrain_signature = signature
