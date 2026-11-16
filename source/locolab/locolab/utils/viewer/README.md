# Viewer

Add `--viser` to a play command to open the [mjviser](https://github.com/mujocolab/mjviser)
browser viewer at `http://localhost:8080`. Use `--viser-host` and `--viser-port` to
change the address.

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
| Visualization → Robot | Visual/collision meshes, convex hulls and distance LOD |
| Visualization → Terrain | Height coloring or solid color |
| Visualization → Sensors | Per-sensor display options |

Sensor options are **Point cloud** (on by default), **Origin axes** (off), and
**Camera frustum** (cameras only, off). The `debug_viz` master switch controls all
layers and preserves these settings when toggled. Layers follow the selected
environments. Visualization settings do not change physics or RL observations.

Restart the play process after editing Python visualization configs; refreshing
the browser does not reload them.

## Play-script integration

1. Call `prepare_play_viewer` before creating the environment.
2. Call `open_play_viewer` after the environment exists.
3. Call `viewer.after_step(obs)` in the play loop. If `viewer.paces` is true,
   skip the script's own realtime sleep.

Without `--viser`, `open_play_viewer` returns an idle viewer.
