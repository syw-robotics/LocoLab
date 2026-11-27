# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Session-local recording of poses already published by mjviser."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from operator import attrgetter
from queue import SimpleQueue

import numpy as np

from ._viser import viser

MAX_FRAMES = 12_000
MAX_RECORDING_BYTES = 256 * 1024**2


@dataclass(frozen=True)
class ReplayFrame:
    tick: int
    positions: np.ndarray
    rotations: np.ndarray


class PoseReplay:
    """Use the play loop as the sole writer of recording and replay state."""

    def __init__(self, gui, step_dt: float, pose_values: int) -> None:
        self.recording = False
        self.active = False
        self._step_dt = step_dt
        pose_bytes = pose_values * np.dtype(np.float32).itemsize
        self._max_frames = min(MAX_FRAMES, MAX_RECORDING_BYTES // pose_bytes)
        self._limit_reached = False
        self.frames: list[ReplayFrame] = []
        self._cursor = 0
        self._shown: tuple[int, int] | None = None
        self._play_start: float | None = None
        self._commands: SimpleQueue = SimpleQueue()

        self._status = gui.add_html("")
        self._record = gui.add_button(
            "Start recording", icon=viser.Icon.PLAYER_RECORD,
            hint="Record robot link poses. Starting a clip replaces the previous clip.",
        )
        self._seek = gui.add_slider("Frame", min=0, max=1, step=1, initial_value=0)
        self._play = gui.add_button("Play recording", icon=viser.Icon.PLAYER_PLAY)
        self.video_start = gui.add_button("Record a video", icon=viser.Icon.VIDEO)
        self.video_stop = gui.add_button("Stop video", icon=viser.Icon.PLAYER_STOP, color="red")
        self._live = gui.add_button("Return to live")
        for button, action in ((self._record, "record"), (self._play, "play"), (self._live, "live")):
            button.on_click(lambda _event, action=action: self._commands.put(action))

        @self._seek.on_update
        def _(event) -> None:
            if event.client is not None:
                self._commands.put(event.target.value)

        self._sync_gui()

    def update_controls(self, now: float) -> None:
        if self._commands.empty():
            return
        while not self._commands.empty():
            action = self._commands.get_nowait()
            if action == "record" and not self.active and self._max_frames > 0:
                if self.recording:
                    self._stop_recording()
                else:
                    self.recording = True
                    self.frames.clear()
                    self._limit_reached = False
                    self._cursor = 0
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

    def append(self, tick: int, poses: tuple[np.ndarray, np.ndarray]) -> None:
        positions, rotations = poses
        self.frames.append(ReplayFrame(tick, positions.astype(np.float32), rotations.astype(np.float32)))
        self._limit_reached = len(self.frames) == self._max_frames
        if self._limit_reached:
            self._stop_recording()
            self._sync_gui()
        else:
            self._update_status()

    def _stop_recording(self) -> None:
        self.recording = False
        self._cursor = max(0, len(self.frames) - 1)

    def frame(self, now: float, env_idx: int) -> ReplayFrame | None:
        if self._play_start is not None:
            tick = self.frames[0].tick + (now - self._play_start) / self._step_dt
            self._cursor = max(self._cursor, bisect_right(self.frames, tick, key=attrgetter("tick")) - 1)
            if self._cursor == len(self.frames) - 1:
                self._play_start = None
                self._sync_gui()
        shown = (self._cursor, env_idx)
        if shown == self._shown:
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

    def _sync_gui(self) -> None:
        last = len(self.frames) - 1
        playing = self._play_start is not None
        self._record.disabled = self.active or self._max_frames == 0
        self._record.label = "Stop recording" if self.recording else "Start recording"
        self._record.icon = viser.Icon.PLAYER_STOP if self.recording else viser.Icon.PLAYER_RECORD
        self._seek.disabled = self.recording or last < 1
        self._seek.max = max(1, last)
        self._seek.value = self._cursor
        self._play.disabled = self.recording or last < 0
        self._play.label = "Pause recording playback" if playing else "Play recording"
        self._play.icon = viser.Icon.PLAYER_PAUSE if playing else viser.Icon.PLAYER_PLAY
        self._live.disabled = not self.active
        self._update_status()

    def _update_status(self) -> None:
        duration = self.duration
        if self.recording:
            status = f"Recording · {duration:.1f} s"
        elif self.frames:
            status = f"{self._time(self._cursor):.2f} / {duration:.2f} s · {len(self.frames)} frames"
            if self._limit_reached:
                status += " · Recording limit reached"
        elif self._max_frames == 0:
            status = f"A recorded frame exceeds the {MAX_RECORDING_BYTES // 1024**2} MiB limit"
        else:
            status = "No recording"
        self._status.content = (
            "<div style='margin:0 10px 6px;font-size:12px;line-height:1.4;"
            "color:var(--mantine-color-dimmed);font-variant-numeric:tabular-nums'>"
            f"{status}</div>"
        )
