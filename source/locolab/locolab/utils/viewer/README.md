# Viewer

Optional browser view for headless play. Isaac Sim keeps simulating. This package poses the same URDF from the articulation state and serves it with [Viser](https://github.com/nerfstudio-project/viser).

**This package is fully vibe coded.**

Pass `--viser` to a play script. The startup message is Viser's own URL box. The default bind is `0.0.0.0`, so the box shows `http://localhost:8080` (`--viser-host`, `--viser-port`). Importing `locolab.utils.viewer` does not import Viser. The server starts only inside `open_play_viewer`.

Play scripts call three functions:

- `prepare_play_viewer` before `gym.make`, so the terrain mesh is kept for the browser.
- `open_play_viewer` after the env exists. Without `--viser` this returns an idle object.
- `viewer.after_step()` in the loop. When `viewer.paces` is true, skip the script's own `--real-time` sleep.

| File | Role |
| --- | --- |
| `play.py` | CLI flags and the idle stand-in |
| `viser_play.py` | Server, GUI, camera follow, playback pace |
| `_viser.py` | Import Viser with the environment's `websockets`. Kit's fast importer otherwise keeps Isaac Sim's older copy |
| `urdf.py` | Load the robot once and pose every environment from the same meshes |
| `terrain.py` | Keep and color the terrain mesh |
| `debug_vis.py` | Velocity arrows, ray hits, and terrain-origin frames in the browser |
| `browser.py` | Page script for camera keys and shortcuts |
