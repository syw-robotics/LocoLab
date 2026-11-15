# MID360

GPU LiDAR and IMU simulation. Defaults: 200,000 rays/s, 10 Hz point clouds
(20,000 points/frame), and 200 Hz IMU. Includes non-repetitive scans, motion
distortion, measurement noise and reflectivity-dependent range.

## Configuration

Import `Mid360Cfg`, `Mid360ImuCfg` and `mid360_output` from
`locolab.assets.sensors.mid360`. Set the parent `prim_path`, mount `offset` (xyzw),
and LiDAR `mesh_prim_paths` in your robot or task configuration.

- `update_period`: sampling interval; must equal `sim.dt` for both sensors (default 0.005 s).
- `frame_rate`: LiDAR publication rate (default 10 Hz). One frame accumulates 20 scan batches at 5 ms.
- `noise=False`: disables noise; range limits and quantization remain.
- `target_reflectivity`: reflectivity by `mesh_prim_paths` index (default 0.5).

G1 mounting and scene configuration live in
[`unitree_g1/mid360_env_cfg.py`](../../../tasks/manager_based/locomotion/velocity/config/unitree_g1/mid360_env_cfg.py),
outside this sensor package. Run the example with:

```bash
python scripts/tools/random_agent.py --task Velocity-Flat-G1-Mid360-Play --num_envs 1 --headless --viser
```

In Viser, `Controls → debug_viz` toggles visibility;
`Visualization → Sensors` configures point clouds and origin axes.

## Data and RL

For the G1 example, use `frame = env.scene["mid360_lidar"].data.frame` and
`imu = env.scene["mid360_imu"].data.output`.

| Value | Shape | Meaning |
| --- | --- | --- |
| `frame.points` | `(N, 20000, 3)` | Sensor coordinates, metres |
| `frame.valid` | `(N, 20000)` | Valid returns; invalid points are zero |
| `frame.points_w` | `(N, 20000, 3)` | World coordinates using each batch's true pose |
| `imu["gyro"]`, `imu["accel_g"]` | `(N, 3)` each | Angular velocity (rad/s), proper acceleration (g) |

Outputs are live GPU views, held between publications and cleared on reset.
Clone tensors to retain them. The example exposes a `mid360` observation group;
using it for RL requires a point-cloud encoder or reduction in the policy.
Start with a small environment count at full scan rate.

Motion is resolved per physics step. IMU bias/drift, atmospheric effects and
multipath are not modeled; real-device calibration is still needed.

Model adapted from `livox-mid360-lidar-unitree-g1-mjlab`; scan table from
[MuJoCo-LiDAR](https://github.com/discoverse-dev/MuJoCo-LiDAR/blob/e6b879df4b96daaed30ef10fe02650d29048fa9b/src/mujoco_lidar/scan_mode/mid360.npy).
Upstream MIT notices are in [LICENSE](LICENSE); neither package is a runtime dependency.
