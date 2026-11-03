<h2 align="center"> LocoLab</h2>

<div align="center">

<!-- [[Video]](https://www.youtube.com/watch?v=tu7LSNYWDTs&ab_channel=LeCARLabatCMU) -->



<!-- [![IsaacSim](https://img.shields.io/badge/IsaacSim-5.1.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html) -->
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-3.0-brightgreen?logo=nvidia&logoColor=white)](https://isaac-sim.github.io/IsaacLab)
[![Z RL](https://img.shields.io/badge/Z_RL-Flexible_RL_Lib-orange?logo=python&logoColor=white)](https://github.com/syw-robotics/z_rl)

<!-- [![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![Windows platform](https://img.shields.io/badge/platform-windows--64-orange.svg)](https://www.microsoft.com/en-us/) -->
<!-- [![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg)](https://opensource.org/license/apache-2-0) -->


</div>

## 🎯 Overview
To develop high-performance locomotion algorithms, a well-configured and stable RL environment is essential.

Existing locomotion env projects are fragmented, lacking expandability, and oftenly do not guarantee robust sim2real transfer.

We introduce **LocoLab**, a RL locomotion **env benchmark** with sim2real robustness guarantees. It enables researchers to focus on algorithm iterations instead of tedious environment setup.


**Key Features:**

- `Flexibility`: ManagerBased Rl env, easy to read, modify, and adapt to new tasks
- `Faster Rollout speed`: Collisions of robot models are properly simplified.
- `Experiment Friendly`: Functions like adding terrain levels logging per terrain types help to experiment and debug.

**Tested and deployable tasks are:**
- Velocity-Flat-G1
- Velocity-Flat-Go2
- Velocity-Rough-Go2


## 📦 Installation

Clone [Isaac Lab 3.0](https://github.com/isaac-sim/IsaacLab), [Z-RL](https://github.com/syw-robotics/z_rl), and LocoLab into separate directories. Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then create and activate a Python 3.12 environment (or activate an existing one):

```bash
uv venv --python 3.12 /path/to/venv
source /path/to/venv/bin/activate
cd /path/to/LocoLab
```

**Required stack — one command:**

```bash
bash scripts/install.sh /path/to/IsaacLab /path/to/z_rl
```

Both local checkout paths are required. This installs Isaac Lab into the activated environment, with **Newton MJWarp + OV PhysX** physics and **Newton GL + Viser** visualization available.

It also installs the supplied Z-RL checkout, LocoLab in editable mode, including LocoLab's [assetx](https://github.com/syw-robotics/assetx) dependency.

**Optional stack — one command:**

```bash
bash scripts/install.sh /path/to/IsaacLab /path/to/z_rl --all
```

This installs the required stack plus **Newton RTX** and **Isaac Sim / Kit**. It can be run directly on a fresh environment or after the required installation.

| Capability | Required installation | With `--all` |
|------------|-----------------------|--------------|
| Newton MJWarp physics (`--physics newton_mjwarp`) | Included | Included |
| OV PhysX physics (`--physics ovphysx`) | Included | Included |
| Newton GL visualizer (`--viz newton_gl`) | Included | Included |
| Viser (`--viz viser` or LocoLab play's `--viser`) | Included | Included |
| Newton RTX visualizer (`--viz newton_rtx`) | | Included |
| Isaac Sim PhysX / Kit (`--physics isaacsim_physx`, `--viz kit`) | | Included |

<!-- The installer uses Isaac Lab's uv workspace configuration and preserves unrelated installed packages with `--inexact`; conflicting dependency versions may still change. `--all` refers to the optional stack above, not every Isaac Lab extra. GPU and driver requirements follow the selected backends; see the installation guide in your Isaac Lab checkout (`docs/source/isaaclab/installation.rst`). -->

<!-- Keep the environment activated and run scripts directly with `python`; `uv run` is not prefered. Installing a visualizer does not enable it: training remains headless unless selected with `--viz`. -->

Check the installation and start a small training run:

```bash
python scripts/list_envs.py

# train example
python scripts/z_rl/train.py --task=Velocity-Flat-Go2 --physics ovphysx --num_envs 16 --max_iterations 50 agent.save_interval=50

# play example
python scripts/z_rl/play.py --task=Velocity-Flat-Go2 --physics newton_mjwarp --num_envs 16 --viz newton_gl  # `--viz viser` for isaaclab viser viewer; barely `--viser` for locolab viser simplied viewer  
```

## 🚀 Usage

- **Training:**

    ```bash
    python scripts/z_rl/train.py --task=<TASK_NAME>
    ```

    Same flags work with `scripts/rsl_rl/train.py`. Isaac Lab 3.0 no longer accepts `--headless`; omit `--viz` for a headless run.

- **Playing:**

    ```bash
    python scripts/z_rl/play.py --task=<TASK_NAME>
    ```

    <!-- Use `scripts/rsl_rl/play.py` for RSL-RL checkpoints. Play tasks are usually `<TASK_NAME>-Play`; `z_rl/play.py` appends `-Play` when missing. -->

### Common CLI options (train & play)

Run `python scripts/z_rl/train.py --help` (or `play.py`) for every flag. The table below covers the options people use most often.

| Option | Purpose |
|--------|---------|
| `--task` | Registered Gym task id (required). |
| `--num_envs` | Number of parallel environments. |
| `--device` | Simulation device, e.g. `cuda:0` or `cpu` (default `cuda:0` on Linux). |
| `--physics` | Physics preset: `newton_mjwarp` (env default), `physx`, `ovphysx`, `isaacsim_physx`. Aliases and Hydra overrides are described in [`physics.py`](source/locolab/locolab/utils/physics.py). |
| `--mdp_backend torch\|warp` | Environment runtime; defaults to `torch`. Warp supports Go2 Flat/Rough velocity, B2/G1 Flat velocity, G1 Flat tracking and their Play configs with Newton MJWarp. |
| `--viz NAME` / `--visualizer NAME` | Isaac Lab visualizer preset(s). See [**`--viz` presets**](#viz--visualizer-presets) below. Comma-separated names run multiple viewers (e.g. `--viz kit,rerun`). Omit `--viz` for the task default (LocoLab velocity tasks: headless). `--video` still pulls in Kit recording via `viz:kit`. |
| `--info` | Print startup logs to the terminal and skip hiding them behind the Loco Lab startup banner; also enables INFO-level SimulationApp logging. |
| `--verbose` | Same banner behavior as `--info`, plus DEBUG-level SimulationApp logs. |
| `--video` | Record video via Kit (`viz:kit`). |
| `--video_length` | Steps per recorded clip (play; train also uses `--video_interval`). |

`--mdp_backend warp` selects the full experimental environment and managers without a `-Warp` task suffix:

```bash
python scripts/z_rl/train.py --task Velocity-Flat-Go2 --mdp_backend warp
python scripts/z_rl/train.py --task Velocity-Rough-Go2 --mdp_backend warp
python scripts/z_rl/train.py --task Velocity-Flat-B2 --mdp_backend warp
python scripts/z_rl/train.py --task Velocity-Flat-G1 --mdp_backend warp
python scripts/z_rl/train.py --task Tracking-Flat-G1 --mdp_backend warp
```

Z-RL and RSL-RL train/play entrypoints share the selector. Supported configs are
`Velocity-Flat-Go2`, `Velocity-Rough-Go2`, `Velocity-Flat-B2`, `Velocity-Flat-G1`,
`Tracking-Flat-G1`, and their `-Play` variants. Other configs fail explicitly
when Warp is selected. Newton MJWarp is required; the default Torch runtime keeps its existing
physics choices. Add the flag explicitly for playback as well. The experimental runtime does not
support video recording.

Z-RL training and checkpoint playback are the validation path. In the current checkout, RSL-RL
training stops before environment creation because its existing config sanitizer imports the removed
`isaaclab_rl.rsl_rl.handle_deprecated_rsl_rl_cfg`; that separate compatibility issue is not changed here.

LocoLab defaults Warp MDP managers to eager execution (`manager_call_config={"default": 1}`),
because captured managers can skip lazy height-scanner updates. No environment variable is needed.
An explicit config takes precedence over `MANAGER_CALL_CONFIG`; setting
`MANAGER_CALL_CONFIG='{"default":2}'` opts into the experimental manager capture path.
Newton's physics CUDA graphs are independent of this setting.
G1 tracking supports explicit manager capture: link FK, velocity and contact-history caches refresh
outside the graph before each replay. Changed input storage invalidates the affected graph.
The G1 path also allows Scene writes/reset and interval-event/reset capture; startup randomization
stays outside graphs. Command scheduling and environment reset selection still execute on the CPU,
so this mode does not capture the entire environment step as one graph.
Tracking command progression, reference
alignment, diagnostic metrics, adaptive sampling/EMA, and reset-state preparation use Warp with persistent buffers.
The motion loader and simulator state-writing APIs are retained. A CPU frame mirror detects clip ends;
GPU frame readback occurs during sampling/state writes, not normal progression. Reset metrics share
one persistent buffer and batched reduction; joint positions/velocities use the fused state writer.
Eager tracking term launches cache argument packing while checking live storage and scalar parameters.
Tracking action processing fuses scaling, clipping and encoder calibration once per control step;
physics substeps reuse the calibrated target.
Custom timers that can expire within an episode retain the original timer scheduling.
Warp sampling preserves the distributions, not Torch's exact random sequence.
See the [tracking setup notes](source/locolab/locolab/tasks/manager_based/locomotion/tracking/README.md)
for motion files and CUDA library setup.

Warp covers observations, per-frame noise/clip/scale and oldest-to-newest term-major history,
processed-action delay, fixed encoder bias, rewards, terminations, reset/interval events, and
manager CUDA graphs. Commands and terrain curriculum keep their Torch managers, as in
`basic_loco_lib`. Startup gain/joint randomization delegates to the standard backend writers once;
the event manager and scene use the upstream capture limitations. Missing custom terms are errors,
rather than silently falling back to incompatible Torch callables. Z-RL retains history metadata
for time slicing and symmetry augmentation. The regular task configs, reward weights, joint order,
and terrain definitions are shared by both runtimes.

Warp adapts a deep copy of the resolved config. The original task configs and Torch terms are
unchanged; omitting `--mdp_backend warp` keeps the original Gym environment and Z-RL wrapper.
The adapter lives outside the task registration tree and is imported only when Warp is selected.

Run the same warmed-up benchmark for both backends (no policy inference or optimizer):

```bash
python scripts/benchmark_mdp.py --task Velocity-Flat-Go2 --mdp_backend torch --num_envs 1024 --viz none
python scripts/benchmark_mdp.py --task Velocity-Flat-Go2 --mdp_backend warp --num_envs 1024 --viz none --check
# Repeat with --task Velocity-Rough-Go2.
```

`--check` compares live observation frames, stateless rewards, and terminations with the Torch
functions on the same physical state. Regression tests additionally cover delayed targets,
encoder bias, history/noise, partial resets, action smoothness, and graph replay after input changes.
The benchmark reports five batches of 100 steps after 50 warmup steps; faster environment steps
do not imply identical random trajectories or establish training convergence.

RTX 4090 capture-mode benchmark at 1024 environments (zero actions, Newton MJWarp, headless):

| Task | Torch step | Warp step | Throughput ratio |
|------|------------|-----------|------------------|
| Go2 Flat | 6.30 ms | 4.55 ms | 1.38x |
| Go2 Rough | 7.04 ms | 5.91 ms | 1.19x |

Live checks passed for 34 Flat and 36 Rough observation/reward/termination terms. Both tasks
captured nine action, observation, reward, and termination stages, including resets. These figures
measure environment stepping; PPO policy inference and optimization are excluded.

**Play-only**

| Option | Purpose |
|--------|---------|
| `--viser` | Browser-based Viser viewer for headless play ([details](source/locolab/locolab/utils/viewer/README.md)). Independent of `--viz`. |
| `--viser-port`, `--viser-host` | Viser bind port and address (defaults `8080`, `0.0.0.0`). |
| `--realtime` | Sleep so wall time matches sim step time (ignored while Viser pacing is active). |
| `--checkpoint`, `--load_run` | Which run/checkpoint to load (see Z-RL/RSL-RL groups in `--help`). |
| `--use_pretrained_checkpoint` | Load published Nucleus checkpoint when available. |

#### `--viz` / `--visualizer` presets

These names come from Isaac Lab’s `MultiBackendVisualizerCfg` (Hydra alias: `visualizer=NAME`, CLI alias: `--viz NAME` or `--visualizer NAME`).

| Preset | Viewer |
|--------|--------|
| `kit` | Omniverse Kit viewport (Isaac Sim window; used automatically when recording with `--video`). |
| `newton_gl` | Newton OpenGL interactive window. |
| `newton_rtx` | Newton RTX interactive window. |
| `rerun` | [Rerun](https://rerun.io/) viewer. |
| `viser` | Isaac Lab’s Viser **backend** (integrated with the sim visualizer stack). Not the same as LocoLab play’s [`--viser`](#play-only) flag. |
| `none` | Explicitly disable all visualizers. |
| `default` | Task preset default (LocoLab locomotion envs: headless / no viewer). |

**Alias:** `newton` → `newton_gl` (deprecated name, still accepted).

**Examples:**

```bash
python scripts/z_rl/play.py --task=... --viz kit
python scripts/z_rl/train.py --task=... --viz newton_gl
python scripts/z_rl/play.py --task=... --viz kit,rerun
python scripts/z_rl/train.py --task=... visualizer=rerun   # Hydra-style override
```

**Z-RL train** (see `scripts/z_rl/cli_args.py`): `--resume`, `--load_run`, `--checkpoint`, `--experiment_name`, `--run_name`, `--max_iterations`, `--logger`, `--distributed`, `--teacher_checkpoint` (distillation).

Z-RL and RSL-RL run directories use `<timestamp>_<resolved-physics>`, with the motion
filename stem appended for single-clip tracking. `--run_name` adds a custom suffix,
for example, `<timestamp>_newton_mjwarp_walk_trial1`. Automatic PhysX selection is
recorded as the actual backend (`ovphysx` or `isaacsim_physx`).

Comet logging (optional): install `pip install comet-ml`, set `COMET_API_KEY` (and optionally `COMET_WORKSPACE`), then train with `--logger comet --log_project_name <project>`. Local TensorBoard logs are retained. Agent configs also accept `logger = {"class_name": "CometLogWriter", "project_name": "<project>"}`.

### Play checkpoints trained on a server

Pass the experiment's browser URL directly to `--checkpoint`; no manual checkpoint copy or
`--logger` selection is required. The task and local agent configuration must match training.

```bash
# Copy the run URL from W&B or Comet. Automatically downloads the highest model_<iteration>.pt.
python scripts/z_rl/play.py --task <task> --checkpoint "https://wandb.ai/<team>/<project>/runs/<run-id>"
python scripts/z_rl/play.py --task <task> --checkpoint "https://www.comet.com/<workspace>/<project>/<experiment-key>"

# Pin a particular checkpoint by appending a fragment (quote the URL).
python scripts/z_rl/play.py --task <task> --checkpoint "comet://<workspace>/<project>/<experiment-key>#model_1000.pt"
python scripts/z_rl/play.py --task <task> --checkpoint "neptune://<workspace>/<project>/<run-id>"
```

Supported short references: `wandb://team/project/run-id`, `comet://workspace/project/experiment-key`,
and `neptune://workspace/project/run-id`. Neptune browser links of the form
`https://app.neptune.ai/workspace/project/e/RUN-123` also work. Omit the fragment or use `#latest`
for the latest uploaded checkpoint; `#model_1000.pt` selects an exact iteration.

Install and authenticate only the SDK you use, on the local machine:

| Service | Install | Authentication |
| --- | --- | --- |
| W&B | `pip install wandb` | `wandb login` or `WANDB_API_KEY` |
| Comet | `pip install comet-ml` | `COMET_API_KEY` or existing Comet SDK configuration |
| Neptune (legacy writer) | `pip install 'neptune<2'` | `NEPTUNE_API_TOKEN` |

The training logger must have uploaded the checkpoints, and the local account must have read access.
Downloads are cached under `logs/z_rl/<experiment>/remote/<service>/<run-hash>/`.
Each invocation queries the remote run again to discover new iterations, while already cached
checkpoint files are reused. Interrupted downloads are not cached. An explicit remote `--checkpoint`
takes precedence over `--load_run`; local paths and ordinary HTTP checkpoint file URLs retain
existing behavior. The downloaded model uses the local task configuration; remote environment
configuration is not automatically applied. TensorBoard and SwanLab remote run downloads are
not supported by this entry point.


**Startup banner:** In an interactive terminal, train/play show a **Loco Lab · train/play** summary while the sim starts. Until `screen.close()`, stdout from Kit and env creation is buffered and then summarized; use **`--info`** to see all of it.

- **Helpful scripts:**

    - Listing registered tasks (includes installed external projects):

        ```bash
        python scripts/list_envs.py
        ```

    - Running a task with a random agent for testing:

        ```bash
        python scripts/random_agent.py --task=<TASK_NAME>
        ```

    <!-- - Fetch a training logging from remote server: -->
    <!--     ```bash -->
    <!--     ./scripts/sync_logs.sh --help  # check this script for usage -->
    <!--     ``` -->

- **Preset terrains:**

    LocoLab provides additional preset terrain types beyond IsaacLab's defaults, including custom height-field and mesh terrains. See the [terrain README](source/locolab/locolab/utils/terrains/locolab_terrains/README.md).


## Create an external project (TBD)

LocoLab sits on Isaac Lab the same way your own project can sit on LocoLab:

```text
Isaac Lab  ->  LocoLab  ->  your external project
```

Generate a standalone, pip-installable task package **outside** this repository:

```bash
python -m locolab.template
# or
python scripts/new_project.py
```

The generator asks for a parent directory, project folder name, Python package name, config/task name, and RL library (`z_rl`, `rsl_rl`, or `both`). These names are independent: `Go2ParkourLab` can install as `go2_parkour` and register `Velocity-Flat-MyGo2`. Avoid config names like `go2` that collide with LocoLab built-ins.

Then install the generated package with the same Python interpreter that has LocoLab:

```bash
python -m pip install -e <generated-project>/source/<package-name>
```

Installed projects register Gym tasks through the `locolab.tasks` entry point.
Train and play from the generated repository (same interpreter that has LocoLab):

```bash
python scripts/list_envs.py
python scripts/z_rl/train.py --task=Velocity-Flat-<ConfigName>
python scripts/z_rl/play.py --task=Velocity-Flat-<ConfigName>
```

The generated `env_cfg.py` is a placeholder: it registers with Gym so `list_envs` works, but you must fill in scene and MDP terms before training.


<!-- ## 📂 Task Organization -->
<!--  -->
<!-- The tasks are organized in the following hierarchy: -->
<!--  -->
<!-- ``` -->
<!-- tasks -->
<!-- ├── manager_based -->
<!-- │   ├── locomotion -->
<!-- │   │   ├── velocity -->
<!-- │   │   │   ├── config -->
<!-- │   │   │   │   ├── unitree_go2 -->
<!-- │   │   │   │   │   ├── agents -->
<!-- │   │   │   │   │   │   ├── z_rl_ppo_cfg.py -->
<!-- │   │   │   │   │   │   └── ... -->
<!-- │   │   │   │   │   ├── mdp_cfg (mdp configurations) -->
<!-- │   │   │   │   │   ├── flat_env_cfg.py -->
<!-- │   │   │   │   │   ├── rough_env_cfg.py -->
<!-- │   │   │   │   │   └── __init__.py (register the environment cfgs) -->
<!-- │   │   │   └── mdp (mdp components) -->
<!-- │   │   └── ... -->
<!-- │   └── ... -->
<!-- └── ... -->
<!-- ``` -->
<!--  -->
<!-- Different types of tasks are organized in different sub-directories, such that tasks are clearly separated and mdp components are easily reusable. -->

## 📝 TODO
- [x] **Symmetry Config Definition**: Alongside with which in Z RL
- [x] **Muon Optimizer Support**: Alongside with which in Z RL
- [ ] **G1**: Add Velocity-Flat-AMP-G1 and Velocity-Rough-G1, G1 tracking task, G1 supermimic implementation
- [ ] **Go2**: Refine Velocity-Rough-Go2
- [ ] **B2**: Add Velocity-Rough-B2
<!-- - [ ] **Active Adaptation Port**: Migrate LocoLab tasks to (AA)[https://github.com/Agent-3154/active-adaptation] -->


