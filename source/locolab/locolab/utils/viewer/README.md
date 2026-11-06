# Viewer

Optional browser view for headless play. Isaac Lab keeps simulating;
[mjviser](https://github.com/mujocolab/mjviser) draws robots from simulation body
poses. It loads the `model.xml` saved alongside the spawned USD by assetx, so
fixed-link merges and collision-shape edits match the converted asset. For other
USD assets, provide a matching `.xml`/`.mjcf` in `mjcf_path`.

**This package is fully vibe coded.**

Pass `--viser` to a play script. The startup message is Viser's own URL box. The default bind is `0.0.0.0`, so the box shows `http://localhost:8080` (`--viser-host`, `--viser-port`). Importing `locolab.utils.viewer` does not import Viser. The server starts only inside `open_play_viewer`.

Play scripts call three functions:

- `prepare_play_viewer` before `gym.make`, so the terrain mesh is kept for the browser.
- `open_play_viewer` after the env exists. Without `--viser` this returns an idle object.
- `viewer.after_step()` in the loop. When `viewer.paces` is true, skip the script's own `--realtime` sleep.

Robot layers are **Visual** and **Collision** (green enabled, gray disabled by the
environment). **Convex hull** selects the shared mesh mode for Visual and browser
motion ghosts: off uses the original visual meshes with matching mjviser grouping
and distance LOD; on replaces both with visual-only convex hulls. **Distance LOD**
in **Visualization → Robot** defaults to on; turn it off to keep full mesh detail
at every distance for both Visual and browser ghosts. This toggle is independent
of the convex-hull mode. Visual primitives
and meshes without valid hulls retain their original geometry. The collision layer
always shows the assetx-exported simulation collision shapes and is unaffected by
the visual mesh mode. Ghost visibility remains a separate task debug control.
Kit/Newton ghosts always keep their original visual meshes. These controls affect
display only; they do not change simulation.

Camera tracking, FOV and environment selection use mjviser's native Scene GUI.
Terrain and task overlays are children of its `/fixed_bodies` frame, so tracking
moves the whole scene consistently. Playback pause/reset/speed and task-specific
debug/terrain controls remain in LocoLab. **Controls → Navigation → Move speed (m/s)**
adjusts WASDQE camera translation from 0.1 to 20 m/s (default 2 m/s).
It runs in the browser and is independent of simulation playback speed.

Velocity Play configs enable push arrows in both native and browser viewers.
The thin blue arrow starts at the pushed articulation's root link and shows
the applied world-frame linear velocity increment (0.5 m per m/s). It follows
that link and fades linearly from opacity 1 to 0 over
3 s of simulation time. Each environment fades independently; a new push restarts
its fade, and an episode reset clears it. In Viser, toggle it under
**debug_viz → event/push_robot_by_setting_velocity**.
For custom configs, set the push event's `params["debug_vis"] = True`; training
configs leave it off and retain the upstream push implementation.

Both viewers use `PushDebugVisCfg` from `locolab.utils.debug_viz.push.push_debug_vis_cfg`
in the push event's `params["debug_vis_cfg"]`.
Configure its duration, arrow scale (length per velocity, Y/Z thickness), and
material before creating the environment:

```python
push_vis = cfg.events.push_robot_by_setting_velocity.params["debug_vis_cfg"]
push_vis.duration_s = 3.0
push_vis.visualizer_cfg.markers["arrow"].scale = (0.5, 0.05, 0.05)
push_vis.visualizer_cfg.markers["arrow"].visual_material.diffuse_color = (0.2, 0.2, 0.9)
```

| File | Role |
| --- | --- |
| `play.py` | CLI flags and the idle stand-in |
| `viser_play.py` | Isaac play-loop adapter, playback controls and terrain/debug UI; native mjviser scene GUI |
| `_viser.py` | Import Viser with the environment's `websockets`. Kit's fast importer otherwise keeps Isaac Sim's older copy |
| `browser.py` | Browser-side WASDQE speed adjustment |
| `assets.py` | Resolve the MJCF from the spawned assetx bundle or an explicit XML source |
| `mjviser_robot.py` | Map simulation body poses to mjviser; reconstruct merged fixed bodies |
| `mjcf.py` | Reference-motion poses via MuJoCo native FK; ghost geometry via mjviser's converters |
| `terrain.py` | Keep and color the terrain mesh |
| `debug_vis.py` | Velocity arrows, ray hits, and terrain-origin frames in the browser |
