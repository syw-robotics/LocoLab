# Viewer

Add `--viser` to a play command to open the [mjviser](https://github.com/mujocolab/mjviser)
browser viewer at `http://localhost:8080`. Use `--viser-host` and `--viser-port` to
change the address.

Pass `--viser --viser-low-bandwidth` for low bandwith. The viewer starts with
only the selected environment visible. Robot poses update at up to 20 Hz, and
debug drawings and the depth preview update at up to 5 Hz. Sensor point clouds
start off and show at most 1,000 points per sensor when enabled. Playback speed
still controls the simulation step rate.

To reuse robot and terrain GLBs when refreshing or reconnecting, run
`python scripts/tools/patch_viser.py` once in the play environment and pass
`--viser --viser-cache-assets`. Use the same browser profile and viewer address.
The first visit downloads assets via HTTP. Each viewer restart gets new asset URLs.

Robot geometry comes from assetx's bundled `model.xml`. For other USD assets,
set `mjcf_path` to a matching XML/MJCF file.

## Controls

| Location | Purpose |
| --- | --- |
| Controls → Environment | Select environments or enable robot selection by clicking |
| Controls | Pause, reset and playback speed |
| Controls → Camera | Camera tracking, FOV and WASDQE movement speed |
| Controls → debug_viz | Master switches for sensors, commands, push arrows and motion ghosts |
| Controls → Image observation | Preview the selected environment's `depth_image` observation |
| Controls → Replay | Record and scrub robot motion, or capture a video of the 3D scene |
| Visualization → Robot | Visual/collision meshes, convex hulls and distance LOD |
| Visualization → Terrain | Height coloring or solid color |
| Visualization → Sensors | Per-sensor display options |

Sensor options are **Point cloud** (on by default in standard mode), **Origin axes** (off), and
**Camera frustum** (cameras only, off). The `debug_viz` master switch controls all
layers and preserves these settings when toggled. Layers follow the selected
environments. Visualization settings do not change physics or RL observations.

Restart the play process after editing Python visualization configs; refreshing
the browser does not reload them.

## Replay

Expand **Controls → Replay** (collapsed by default), click **Start recording**, run the simulation, then click
**Stop recording**. The status shows elapsed simulation seconds even with Replay collapsed.
Drag **Frame** to inspect a pose or click **Play recording** to play at simulation speed.
Each frame stores the robot link poses for all environments. Replay uses the current
terrain. The **debug_viz** switches control overlays such as virtual obstacles,
including while scrubbing or paused on a recorded frame. These overlays read the
current simulation state. The depth preview is live-only.

**Return to live** restores the current simulation view. Use **Controls → Play**
to continue stepping. The viewer keeps one clip in memory and stops recording
at 12,000 frames or 256 MiB of recorded arrays, whichever comes first.
Starting a new clip replaces the previous clip.

Click **Record a video** to capture the 3D scene during live viewing or replay.
The video uses the viewer's background color. The pointer and Viser panels stay
outside the video. Click **Stop video** to download the MP4. Its filename includes
the task, the mode at the start of recording (`live` or `replay`), and the local
start time. Use a browser with MP4 canvas recording support.

## Play-script integration

1. Call `prepare_play_viewer` before creating the environment.
2. Call `open_play_viewer` after the environment exists.
3. Call `viewer.after_step(obs)` in the play loop. If `viewer.paces` is true,
   skip the script's own realtime sleep.

Without `--viser`, `open_play_viewer` returns an idle viewer.
