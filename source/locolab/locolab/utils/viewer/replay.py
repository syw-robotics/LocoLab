# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Session-local recording of poses already published by mjviser."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from datetime import datetime
from html import escape
from operator import attrgetter
from pathlib import Path
from queue import SimpleQueue
from tempfile import TemporaryDirectory

import numpy as np

from ._viser import viser
from .debug_vis import DebugFrame
from .replay_export import VIDEO_QUALITY

MAX_FRAMES = 12_000
MAX_RECORDING_BYTES = 256 * 1024**2


@dataclass(frozen=True)
class ReplayFrame:
    tick: int
    positions: np.ndarray
    rotations: np.ndarray
    debug: DebugFrame | None


class PoseReplay:
    """Use the play loop as the sole writer of recording and replay state."""

    def __init__(self, gui, step_dt: float, pose_values: int, num_envs: int, recordings_dir: Path | None) -> None:
        self.recording = False
        self.active = False
        self._step_dt = step_dt
        self._pose_bytes = pose_values * np.dtype(np.float32).itemsize
        self._bytes = 0
        self._limit_reached = False
        self.frames: list[ReplayFrame] = []
        self._recordings_dir = recordings_dir
        self.export_request = None
        self._cursor = 0
        self._shown: tuple[int, int] | None = None
        self._play_start: float | None = None
        self._commands: SimpleQueue = SimpleQueue()

        self._status = gui.add_html("")
        self._layer_options = {
            key: gui.add_checkbox(label, initial_value=False)
            for key, label in (("commands", "Record command debug"), ("debug", "Record other debug"),
                               ("terrain", "Include terrain"))
        }
        self.layers: frozenset[str] = frozenset()
        self._record = gui.add_button(
            "Start recording", icon=viser.Icon.PLAYER_RECORD,
            hint="Record robot poses and the selected layers. Starting a clip replaces the previous clip.",
        )
        self._seek = gui.add_slider("Frame", min=0, max=1, step=1, initial_value=0)
        self._play = gui.add_button("Play recording", icon=viser.Icon.PLAYER_PLAY)
        self._live = gui.add_button("Return to live")
        for button, action in ((self._record, "record"), (self._play, "play"), (self._live, "live")):
            button.on_click(lambda _event, action=action: self._commands.put(action))

        @self._seek.on_update
        def _(event) -> None:
            if event.client is not None:
                self._commands.put(event.target.value)

        if recordings_dir is not None:
            with gui.add_folder("Export", expand_by_default=False):
                quality = gui.add_dropdown("MP4 quality", tuple(VIDEO_QUALITY),
                                           initial_value="Medium · 720p")
                self._video = gui.add_button("Export MP4")
                env_id = gui.add_number("Trajectory env ID", initial_value=0, min=0, max=num_envs - 1, step=1)
                self._trajectory = gui.add_button("Export Blender trajectory (.usdc)")
                self._export_status = gui.add_html("")

                @self._video.on_click
                def _(event) -> None:
                    self._commands.put(("mp4", event.client, quality.value))

                @self._trajectory.on_click
                def _(_event) -> None:
                    self._commands.put(("usdc", None, min(max(0, int(env_id.value)), num_envs - 1)))

        self._sync_gui()

    def update_controls(self, now: float) -> None:
        if self._commands.empty():
            return
        while not self._commands.empty():
            action = self._commands.get_nowait()
            if action == "record" and not self.active and self._pose_bytes <= MAX_RECORDING_BYTES:
                if self.recording:
                    self._stop_recording()
                else:
                    self.recording = True
                    self.layers = frozenset(key for key, option in self._layer_options.items() if option.value)
                    self.frames.clear()
                    self._bytes = 0
                    self._limit_reached = False
                    self._cursor = 0
            elif isinstance(action, tuple):
                if self.frames and not self.recording and self.export_request is None:
                    self.export_request = action
                    if action[0] == "mp4":
                        self.active = True
                        self._play_start = None
                    break
            elif action == "live":
                self.active = False
                self._play_start = None
                self._shown = None
            elif self.frames and not self.recording:
                self.active = True
                if action == "play":
                    if self._play_start is not None:
                        self._play_start = None
                    else:
                        if self._cursor == len(self.frames) - 1:
                            self._cursor = 0
                        self._play_start = now - self._time(self._cursor)
                elif isinstance(action, int):
                    self._cursor = min(max(0, action), len(self.frames) - 1)
                    self._play_start = None
        self._sync_gui()

    def needs_frame(self, tick: int) -> bool:
        return not self.frames or tick > self.frames[-1].tick

    @property
    def has_debug(self) -> bool:
        return "commands" in self.layers or "debug" in self.layers

    def append(self, tick: int, poses: tuple[np.ndarray, np.ndarray], debug: DebugFrame | None) -> None:
        new_debug = not self.frames or self.frames[-1].debug is not debug
        size = self._pose_bytes + (debug.nbytes if new_debug and debug is not None else 0)
        self._limit_reached = self._bytes + size > MAX_RECORDING_BYTES
        if not self._limit_reached:
            self.frames.append(ReplayFrame(tick, poses[0].astype(np.float32), poses[1].astype(np.float32), debug))
            self._bytes += size
            self._limit_reached = len(self.frames) == MAX_FRAMES or self._bytes + self._pose_bytes > MAX_RECORDING_BYTES
        if self._limit_reached:
            self._stop_recording()
            self._sync_gui()
        else:
            self._update_status()

    def _stop_recording(self) -> None:
        self.recording = False
        self._cursor = max(0, len(self.frames) - 1)

    def frame(self, now: float, env_idx: int, *, refresh: bool = False) -> ReplayFrame | None:
        if self._play_start is not None:
            tick = self.frames[0].tick + (now - self._play_start) / self._step_dt
            self._cursor = max(self._cursor, bisect_right(self.frames, tick, key=attrgetter("tick")) - 1)
            if self._cursor == len(self.frames) - 1:
                self._play_start = None
                self._sync_gui()
        shown = (self._cursor, env_idx)
        if shown == self._shown and not refresh:
            return None
        self._shown = shown
        self._seek.value = self._cursor
        self._update_status()
        return self.frames[self._cursor]

    def _time(self, index: int) -> float:
        return (self.frames[index].tick - self.frames[0].tick) * self._step_dt

    @property
    def duration(self) -> float:
        return self._time(-1) if self.frames else 0.0

    def export(self, server, model, show_frame) -> None:
        from .replay_export import export_mp4, export_trajectory

        kind, client, option = self.export_request
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        name = f"replay-{stamp}.mp4" if kind == "mp4" else f"trajectory-env{option}-{stamp}.usdc"
        destination = self._recordings_dir / name

        def progress(done, total):
            self._export_status.content = f"<p>Exporting {done}/{total} frames</p>"

        try:
            progress(0, len(self.frames))
            destination.parent.mkdir(parents=True, exist_ok=True)
            with TemporaryDirectory(dir=destination.parent) as tmp:
                path = Path(tmp) / name
                if kind == "mp4":
                    export_mp4(path, self.frames, self._step_dt, option, server, client, show_frame, progress)
                else:
                    export_trajectory(path, self.frames, self._step_dt, model, option)
                path.replace(destination)
            self._export_status.content = f"<p>Saved {escape(str(destination))}</p>"
        except Exception as exc:
            self._export_status.content = f"<p>Export failed · {escape(str(exc))}</p>"
        finally:
            self.export_request = None
            self._shown = None
            self._sync_gui()

    def _sync_gui(self) -> None:
        last = len(self.frames) - 1
        playing = self._play_start is not None
        exporting = self.export_request is not None
        for option in self._layer_options.values():
            option.disabled = self.recording or self.active or exporting
        self._record.disabled = exporting or self.active or self._pose_bytes > MAX_RECORDING_BYTES
        self._record.label = "Stop recording" if self.recording else "Start recording"
        self._record.icon = viser.Icon.PLAYER_STOP if self.recording else viser.Icon.PLAYER_RECORD
        self._seek.disabled = exporting or self.recording or last < 1
        self._seek.max = max(1, last)
        self._seek.value = self._cursor
        self._play.disabled = exporting or self.recording or last < 0
        self._play.label = "Pause recording playback" if playing else "Play recording"
        self._play.icon = viser.Icon.PLAYER_PAUSE if playing else viser.Icon.PLAYER_PLAY
        self._live.disabled = exporting or not self.active
        if self._recordings_dir is not None:
            self._video.disabled = self._trajectory.disabled = self.recording or last < 0 or exporting
        self._update_status()

    def _update_status(self) -> None:
        duration = self.duration
        if self.recording:
            status = f"Recording · {duration:.1f} s"
        elif self.frames:
            status = f"{self._time(self._cursor):.2f} / {duration:.2f} s · {len(self.frames)} frames"
            if self._limit_reached:
                status += " · Recording limit reached"
        elif self._pose_bytes > MAX_RECORDING_BYTES or self._limit_reached:
            status = f"A recorded frame exceeds the {MAX_RECORDING_BYTES // 1024**2} MiB limit"
        else:
            status = "No recording"
        self._status.content = f"<p>{status}</p>"
