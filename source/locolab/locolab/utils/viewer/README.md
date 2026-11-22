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
| Scene / Environment | Select environments, track a robot, adjust FOV; click a robot to select it |
| Controls | Pause, reset and playback speed |
| Controls → Navigation | WASDQE movement speed |
| Controls → debug_viz | Master switches for sensors, commands, push arrows and motion ghosts |
| Controls → Depth observation | Preview the selected environment's `depth_image` observation |
| Controls → Replay | Record, scrub and export a clip |
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
By default each frame stores only the robot link poses for all environments.
Before starting a clip, enable **Record command debug** for visible command arrows,
goals and motion ghosts, or **Record other debug** for visible virtual obstacles,
sensor geometry and other debug layers. **Include terrain** shows the scene's
static terrain mesh or ground grid during replay without copying it per frame.
Debug layers keep their recorded world positions and environment selection.
You can move the camera or mute recorded layers in **Controls → debug_viz** during replay.
The 2D depth preview is live-only.

**Return to live** restores the current simulation view. Use **Controls → Play**
to continue stepping. The viewer keeps one clip in memory and stops recording
at 12,000 frames or 256 MiB of recorded arrays, whichever comes first.
Starting a new clip replaces the previous clip.

For checkpoint playback, expand **Replay → Export** after stopping the recording.
Both exports are saved in the `recordings/` folder beside the loaded checkpoint.

- **MP4 quality** selects 480p, 720p or 1080p. **Export MP4** renders the full clip
  from the requesting browser's 3D viewport. The output keeps the initial viewport
  aspect ratio and follows camera movement, tracking and auto orbit during export.
  Only the 3D scene appears in the video. The mouse pointer and GUI are excluded.
  Keep that browser connected during export. Encoding uses FFmpeg with H.264.
- **Trajectory env ID** selects one environment. **Export Blender trajectory (.usdc)**
  writes a standalone USD animation with robot geometry and world-space link transforms
  at the recorded simulation timestamps. In Blender, choose **File → Import → Universal Scene Description**
  and play in a 24 fps scene (Blender's default).

## Play-script integration

1. Call `prepare_play_viewer` before creating the environment.
2. Call `open_play_viewer` after the environment exists.
3. Call `viewer.after_step(obs)` in the play loop. If `viewer.paces` is true,
   skip the script's own realtime sleep.

Without `--viser`, `open_play_viewer` returns an idle viewer.
