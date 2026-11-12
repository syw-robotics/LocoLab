<h2 align="center"> LocoLab</h2>

<div align="center">

<!-- [[Video]](https://www.youtube.com/watch?v=tu7LSNYWDTs&ab_channel=LeCARLabatCMU) -->



<!-- [![IsaacSim](https://img.shields.io/badge/IsaacSim-5.1.0-7D828A.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html) -->
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-3.0-6F8575?logo=nvidia&logoColor=white)](https://isaac-sim.github.io/IsaacLab)
[![Z RL](https://img.shields.io/badge/Z_RL-Flexible_RL_Lib-9B866B?logo=python&logoColor=white)](https://github.com/syw-robotics/z_rl)

<!-- [![Linux platform](https://img.shields.io/badge/platform-linux--64-9B866B.svg)](https://releases.ubuntu.com/22.04/)
[![Windows platform](https://img.shields.io/badge/platform-windows--64-9B866B.svg)](https://www.microsoft.com/en-us/) -->
<!-- [![License](https://img.shields.io/badge/license-BSD--3--Clause-7D828A.svg)](LICENSE) -->


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

We recommand to use [uv](https://docs.astral.sh/uv/getting-started/installation/), local [Isaac Lab 3.0](https://github.com/isaac-sim/IsaacLab) and [Z-RL](https://github.com/syw-robotics/z_rl), and a compatible NVIDIA GPU/driver ([Isaac Lab requirements](https://isaac-sim.github.io/IsaacLab/main/source/isaaclab/installation.html#installation-system-requirements)). Isaac Sim is optional.

Clone [Isaac Lab 3.0](https://github.com/isaac-sim/IsaacLab), [Z-RL](https://github.com/syw-robotics/z_rl), and LocoLab into separate directories. Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then create and activate a Python 3.12 environment (or activate an existing one):

```bash
# create a uv env, put .venv in anywhere you like
uv venv --python 3.12 <path-to-place-the-uv-env>
source .venv/bin/activate

# cd to the path you clone LocoLab
cd <path-to-locolab>
bash install.sh <path-to-isaaclab-3.0> <path-to-z_rl>

# Verify backend imports and task registration.
python -c "import ovphysx.types, z_rl, z_rl_loco_lab, locolab; print('Imports OK')"
python scripts/tools/list_envs.py
```

<details>
<summary>📄 Extra dependencies</summary>

The installer uses Isaac Lab's lockfile and installs Z-RL, LocoLab and its Z-RL extension in editable mode, including [assetx](https://github.com/syw-robotics/assetx). `--inexact` retains unrelated packages; conflicting dependency versions may still change.


| Module | Default install | Add-on |
|--------|-----------------|--------|
| Newton MJWarp physics + Newton GL viewer | Included by Isaac Lab | |
| OVPhysX physics + Viser viewer | Included via `ovphysx` / `viser` extras | |
| OVRTX runtime / Newton RTX viewer | | `--extra ovrtx` |
| Isaac Sim PhysX / Kit viewer | | `--extra isaacsim` |


Add both optional modules with `--all` (also works on a fresh environment), or install just one extra:

```bash
bash install.sh <path-to-isaaclab> <path-to-z_rl> --all

# Alternatively, add only OVRTX; replace ovrtx with isaacsim or rerun as needed.
uv sync --project <path-to-isaaclab-3.0> --active --inexact --locked --extra ovrtx
```

</details>



## 🚀 Usage


### Train & Play

<!-- **Startup banner:** In an interactive terminal, train/play show a **Loco Lab · train/play** summary while the sim starts. Until `screen.close()`, stdout from Kit and env creation is buffered and then summarized; use **`--info`** to see all of it. -->

<!-- - **Helpful scripts:** -->
<!--  -->
<!--     - Listing registered tasks (includes installed external projects): -->
<!--  -->
<!--         ```bash -->
<!--         python scripts/tools/list_envs.py -->
<!--         ``` -->
<!--  -->
<!--     - Running a task with a random agent for testing: -->
<!--  -->
<!--         ```bash -->
<!--         python scripts/tools/random_agent.py --task=<TASK_NAME> -->
<!--         ``` -->

    <!-- - Fetch a training logging from remote server: -->
    <!--     ```bash -->
    <!--     ./scripts/tools/sync_logs.sh --help  # check this script for usage -->
    <!--     ``` -->

- **Training:**

    Keep the environment activated and run LocoLab with `python`.

    ```bash
    python scripts/z_rl/train.py --task=<TASK_NAME> --num_envs 4096 --max_iterations 100 --physics ovphysx
    ```

    Same flags work with `scripts/rsl_rl/train.py`. Isaac Lab 3.0 no longer accepts `--headless`; omit `--viz` for a headless run. Add `--info` or `--verbose` to see more terminal loggings, this is recommanded when developing.

- **Playing:**

    ```bash
    python scripts/z_rl/play.py --task=<TASK_NAME>  --num_envs 32 --physics newton_mjwarp --viz newton_gl --realtime
    ```
    
    Add `--viz <viewer-you-like>` (**newton_gl**, **viser** are light weight; orthers are **rerun**, **newton_rtx**, **kit**: isaacsim_rtx) or `--viser` (locolab native mjviser viewer, you will enjoy it!) to view the policy rollout.

    **Z-RL train** (see `scripts/z_rl/cli_args.py`): `--resume`, `--load_run`, `--checkpoint`, `--experiment_name`, `--run_name`, `--max_iterations`, `--logger`, `--distributed`, `--teacher_checkpoint` (distillation).

    Z-RL and RSL-RL run directories use `<timestamp>_<resolved-physics>`, with the motion filename stem appended for single-clip tracking. `--run_name` is appended last, after the motion filename: for example, `<timestamp>_newton_mjwarp_walk_trial1`. Automatic PhysX selection is recorded as the actual backend (`ovphysx` or `isaacsim_physx`).

- **Preset terrains:**

    LocoLab provides additional preset terrain types beyond IsaacLab's defaults, including custom height-field and mesh terrains. See the [terrain README](source/locolab/locolab/utils/terrains/locolab_terrains/README.md).

### External projects

Installed external projects can use the shared entry points from their own directory:

```bash
python -m locolab.entrypoints.z_rl.train --task <TASK_NAME>
python -m locolab.entrypoints.z_rl.play --task <TASK_NAME> --viser
python -m locolab.entrypoints.rsl_rl.train --task <TASK_NAME>
python -m locolab.entrypoints.rsl_rl.play --task <PLAY_TASK_NAME> --viser
```

The existing `scripts/{z_rl,rsl_rl}/{train,play}.py` commands call these same implementations.
External projects register tasks through the `locolab.tasks` package entry point; they do not need
copies or symlinks of the training, playback, or CLI helper scripts. Logs remain under the current
directory's `logs/`, and training records Git state for both LocoLab and the current project directory.

For custom arguments, configuration callbacks, and runner classes, see [the external project guide](./scripts/README.md).

### TrackNest logging (Z-RL)

[TrackNest](https://github.com/LearningFromEmbodiedExperienceLab/TrackNest) wraps Trackio with saved
connection settings and upload status. Use a Z-RL version that supports `TrackioLogWriter(use_tracknest=True)`.
Install its client into the training Python environment and log in once:

```bash
python -m pip install /path/to/TrackNest/client
python -m tracknest login --server-url 'http://localhost:9000/u/<username>/'
```

Replace the URL with your personal or team upload URL and start the corresponding instance in TrackNest.
Remove `TRACKIO_SPACE_ID` if set, then train with:

```bash
python scripts/z_rl/train.py --task=<TASK_NAME> --logger tracknest --log_project_name my-project
```

This selects `TrackioLogWriter` with `use_tracknest=True`, retaining local TensorBoard logs.
Without `--log_project_name`, the project defaults to the agent's `trackio_project` (usually `isaaclab`).
Agent configs may also set:

```python
logger = {"class_name": "TrackioLogWriter", "project_name": "my-project", "use_tracknest": True}
```

Use the saved login to fetch checkpoints for playback or training resume:

```bash
python scripts/z_rl/play.py --task YOUR_TASK_ID --checkpoint 'tracknest://my-project/RUN_ID'
python scripts/z_rl/train.py --task YOUR_TASK_ID --resume --checkpoint 'tracknest://my-project/RUN_ID#model_1000.pt'
```

Use `python -m tracknest runs my-project` to find a run ID. A unique run name also works.
The short reference uses the space saved by `tracknest login`; a TrackNest run-detail URL such as
`http://localhost:9000/u/alice/?project=my-project&selected_run_id=RUN_ID` also works, including `/p/<team-id>/` URLs
on the logged-in server. The corresponding instance must be running (`python -m tracknest start`).
The highest numbered `model_<iteration>.pt` is selected unless `#model_<iteration>.pt` pins a file.
Downloads use the saved Token, bypass environment proxies, verify the SHA-256 and size, and cache locally.

<details>
<summary>📄 More details</summary>

- Script layout:

Run scripts from the repository root with the activated Python environment.

| Path | Purpose |
|------|---------|
| `install.sh` | Install the local Isaac Lab, Z-RL and LocoLab stack. |
| `source/locolab/locolab/entrypoints/` | Shared Z-RL and RSL-RL training, playback and CLI helpers. |
| `scripts/z_rl/` | Thin entry points for Z-RL. |
| `scripts/rsl_rl/` | Thin entry points for RSL-RL. |
<!-- | `scripts/evaluation/` | Velocity and tracking checkpoint evaluation with Z-RL. | -->
<!-- | `scripts/validation/` | AMP/tracking correctness checks and MDP performance benchmarks. | -->
| `scripts/tools/` | List environments, run random actions and sync logs. |


- Common CLI options (train & play)

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


### Play checkpoints trained on a server

Pass the experiment's browser URL directly to `--checkpoint`; no manual checkpoint copy or
`--logger` selection is required. 

```bash
# Copy the run URL from W&B or Comet. Automatically downloads the highest model_<iteration>.pt.

python scripts/z_rl/play.py --task <task> --checkpoint "https://wandb.ai/<team>/<project>/runs/<run-id>"
python scripts/z_rl/play.py --task <task> --checkpoint "https://www.comet.com/<workspace>/<project>/<experiment-key>"

# Pin a particular checkpoint by appending a fragment (quote the URL).
python scripts/z_rl/play.py --task <task> --checkpoint "comet://<workspace>/<project>/<experiment-key>#model_1000.pt"
python scripts/z_rl/play.py --task <task> --checkpoint "neptune://<workspace>/<project>/<run-id>"
```

Supported short references: `wandb://team/project/run-id`, `comet://workspace/project/experiment-key`,
`neptune://workspace/project/run-id`, and `tracknest://project/run-id`. TrackNest uses the login configured above.
Neptune browser links of the form
`https://app.neptune.ai/workspace/project/e/RUN-123` also work. Omit the fragment or use `#latest`
for the latest uploaded checkpoint; `#model_1000.pt` selects an exact iteration.

Install and authenticate only the SDK you use, on the local machine:

| Service | Install | Authentication |
| --- | --- | --- |
| W&B | `pip install wandb` | `wandb login` or `WANDB_API_KEY` |
| Comet | `pip install comet-ml` | `COMET_API_KEY` or existing Comet SDK configuration |
| Neptune (legacy writer) | `pip install 'neptune<2'` | `NEPTUNE_API_TOKEN` |
| TrackNest | Install the TrackNest client wheel or `/path/to/TrackNest/client` | `python -m tracknest login --server-url <personal-or-team-URL>` |



</details>


<!-- Z-RL and RSL-RL train/play entrypoints share the selector. Supported configs are -->
<!-- `Velocity-Flat-Go2`, `Velocity-Rough-Go2`, `Velocity-Flat-B2`, `Velocity-Flat-G1`, -->
<!-- `Tracking-Flat-G1`, and their `-Play` variants. Other configs fail explicitly -->
<!-- when Warp is selected. Newton MJWarp is required; the default Torch runtime keeps its existing -->
<!-- physics choices. Add the flag explicitly for playback as well. The experimental runtime does not -->
<!-- support video recording. -->
<!--  -->
<!-- On CUDA, Go2 Flat automatically captures action, termination, reward and observation managers. -->
<!-- Go2 configurations with ray casters (including Rough) capture only actions and terminations: -->
<!-- reward/observation calls stay eager so lazy scanner data is refreshed. Other tasks and CPU runs -->
<!-- retain eager defaults. This selection happens once during configuration, with no per-step checks. -->
<!-- An explicit config takes precedence over `MANAGER_CALL_CONFIG`; use -->
<!-- `MANAGER_CALL_CONFIG='{"default":1}'` to force eager execution. Full capture via -->
<!-- `MANAGER_CALL_CONFIG='{"default":2}'` remains experimental and fails live parity checks on Go2 Rough. -->
<!-- Newton's physics CUDA graphs are independent of this setting. -->
<!-- G1 tracking supports explicit manager capture: link FK, velocity and contact-history caches refresh -->
<!-- outside the graph before each replay. Changed input storage invalidates the affected graph. -->
<!-- The G1 path also allows Scene writes/reset and interval-event/reset capture; startup randomization -->
<!-- stays outside graphs. Command scheduling and environment reset selection still execute on the CPU, -->
<!-- so this mode does not capture the entire environment step as one graph. -->
<!-- Tracking command progression, reference -->
<!-- alignment, diagnostic metrics, adaptive sampling/EMA, and reset-state preparation use Warp with persistent buffers. -->
<!-- The motion loader and simulator state-writing APIs are retained. A CPU frame mirror detects clip ends; -->
<!-- GPU frame readback occurs during sampling/state writes, not normal progression. Reset metrics share -->
<!-- one persistent buffer and batched reduction; joint positions/velocities use the fused state writer. -->
<!-- Eager tracking term launches cache argument packing while checking live storage and scalar parameters. -->
<!-- Tracking action processing fuses scaling, clipping and encoder calibration once per control step; -->
<!-- physics substeps reuse the calibrated target. -->
<!-- Custom timers that can expire within an episode retain the original timer scheduling. -->
<!-- Warp sampling preserves the distributions, not Torch's exact random sequence. -->
<!-- See the [tracking setup notes](source/locolab/locolab/tasks/manager_based/locomotion/tracking/README.md) -->
<!-- for motion files and CUDA library setup. -->

<!-- AMP/SMP tasks, motion-prior utilities, Warp adapters and offline pretraining live in the -->
<!-- external [motion_prior_locolab](../motion_prior_locolab/README.md) project. -->
<!-- WM/SHAC tracking and its PPO comparison live in -->
<!-- [supermimic_locolab](../supermimic_locolab/README.md). -->
<!-- Install either project in the same Python environment to register its original task IDs. -->
<!-- Ordinary `Tracking-Flat-G1` and shared motion loading/kinematics remain in LocoLab. -->

<!-- Warp covers observations, per-frame noise/clip/scale and oldest-to-newest term-major history, -->
<!-- processed-action delay, fixed encoder bias, rewards, terminations, reset/interval events, and -->
<!-- manager CUDA graphs. Commands and terrain curriculum keep their Torch managers, as in -->
<!-- `basic_loco_lib`. Startup gain/joint randomization delegates to the standard backend writers once; -->
<!-- the event manager and scene use the upstream capture limitations. Missing custom terms are errors, -->
<!-- rather than silently falling back to incompatible Torch callables. Z-RL retains history metadata -->
<!-- for time slicing and symmetry augmentation. The regular task configs, reward weights, joint order, -->
<!-- and terrain definitions are shared by both runtimes. -->
<!--  -->
<!-- Warp adapts a deep copy of the resolved config. The original task configs and Torch terms are -->
<!-- unchanged; omitting `--mdp_backend warp` keeps the original Gym environment and Z-RL wrapper. -->
<!-- The adapter lives outside the task registration tree and is imported only when Warp is selected. -->
<!--  -->
<!-- Run the same warmed-up benchmark for both backends (no policy inference or optimizer): -->
<!--  -->
<!-- ```bash -->
<!-- python scripts/validation/benchmark_mdp.py --task Velocity-Flat-Go2 --mdp_backend torch --num_envs 1024 --viz none -->
<!-- python scripts/validation/benchmark_mdp.py --task Velocity-Flat-Go2 --mdp_backend warp --num_envs 1024 --viz none -->
<!-- # Repeat with --task Velocity-Rough-Go2; force the previous eager default for comparison: -->
<!-- MANAGER_CALL_CONFIG='{"default":1}' python scripts/validation/benchmark_mdp.py --task Velocity-Flat-Go2 --mdp_backend warp --num_envs 1024 --viz none -->
<!-- # Validate executed outputs separately, or collect an untimed CPU/CUDA stage trace: -->
<!-- python scripts/validation/benchmark_mdp.py --task Velocity-Rough-Go2 --mdp_backend warp --num_envs 128 --viz none --check -->
<!-- python scripts/validation/benchmark_mdp.py --task Velocity-Flat-Go2 --mdp_backend warp --num_envs 1024 --viz none --profile /tmp/mdp-trace.json -->
<!-- ``` -->
<!--  -->
<!-- `--check` snapshots actual manager outputs before invoking Torch references. Each stage is checked -->
<!-- in a separate rollout with changing actions and partial timeouts, so reference sensor refreshes -->
<!-- cannot conceal another stage's stale data. Observation noise is disabled for these validation runs; -->
<!-- do not compare their timings against normal noisy runs. History raw frames, stateless rewards and -->
<!-- terminations are checked live; regression tests cover history/noise, action delay/bias and stateful -->
<!-- rewards. `--profile` is Warp-only and records a separate 20-step run after timing, without adding -->
<!-- instrumentation to training. Open its JSON in a Chrome trace viewer or Perfetto. -->
<!--  -->
<!-- The benchmark reports five batches of 100 steps after 50 warmup steps. RTX 4090 results at 1024 -->
<!-- environments (zero actions, Newton MJWarp, headless, observation noise retained): -->
<!--  -->
<!-- | Task | Warp eager step | Automatic capture step | Throughput ratio | -->
<!-- |------|-----------------|------------------------|------------------| -->
<!-- | Go2 Flat | 7.01 ms | 6.02 ms | 1.16x | -->
<!-- | Go2 Rough | 10.54 ms | 10.44 ms | 1.01x (within run variability) | -->
<!--  -->
<!-- Live checks passed for 34 Flat and 36 Rough terms across 100 steps per stage with partial timeouts. -->
<!-- Flat captured nine stages; Rough captured five action/termination stages. Full Rough capture was -->
<!-- excluded after a stale height-scanner reward failed parity. These measurements exclude policy -->
<!-- inference and optimization and do not establish training convergence or identical random trajectories. -->
<!--  -->
<!-- **Play-only** -->
<!--  -->
<!-- | Option | Purpose | -->
<!-- |--------|---------| -->
<!-- | `--viser` | Browser-based Viser viewer for headless play ([details](source/locolab/locolab/utils/viewer/README.md)). Independent of `--viz`. | -->
<!-- | `--viser-port`, `--viser-host` | Viser bind port and address (defaults `8080`, `0.0.0.0`). | -->
<!-- | `--realtime` | Sleep so wall time matches sim step time (ignored while Viser pacing is active). | -->
<!-- | `--checkpoint`, `--load_run` | Which run/checkpoint to load (see Z-RL/RSL-RL groups in `--help`). | -->
<!-- | `--use_pretrained_checkpoint` | Load published Nucleus checkpoint when available. | -->
<!--  -->


<!-- Comet logging (optional): install `pip install comet-ml`, set `COMET_API_KEY` (and optionally `COMET_WORKSPACE`), then train with `--logger comet --log_project_name <project>`. Local TensorBoard logs are retained. Agent configs also accept `logger = {"class_name": "CometLogWriter", "project_name": "<project>"}`. -->
<!--  -->
<!-- Trackio logging (optional): install `pip install trackio` (or the Z-RL extra `pip install -e '/path/to/z_rl[trackio]'`), then train with `--logger trackio --log_project_name <project>`. Agent configs also accept `logger = {"class_name": "TrackioLogWriter", "project_name": "<project>"}`. Metrics, configs, checkpoints, code diffs, and videos are logged alongside local TensorBoard logs. Automatic GPU/CPU monitoring is disabled. Trackio stores runs locally by default; view them with `trackio show --project <project>`. For remote logging, set `TRACKIO_SPACE_ID=owner/space` and authenticate with Hugging Face, or set `TRACKIO_SERVER_URL` for a self-hosted server. See the [Trackio documentation](https://huggingface.co/docs/trackio). -->





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

## 📄 License

LocoLab code is licensed under the [BSD 3-Clause License](LICENSE), following Isaac Lab.
Existing upstream copyright and license notices are retained, including those from
[Isaac Lab](https://github.com/isaac-sim/IsaacLab) and
[RSL-RL](https://github.com/leggedrobotics/rsl_rl).
Third-party robot models, meshes, textures, datasets, and other assets remain
subject to their respective licenses; the code license does not grant additional
rights to those assets.

## 📝 TODO
- [x] **Symmetry Config Definition**: Alongside with which in Z RL
- [x] **Muon Optimizer Support**: Alongside with which in Z RL
- [] **G1**: Add single motion clip tracking task baseline, motion adopts [any4hdmi](https://github.com/syw-robotics/any4hdmi) format
- [ ] **G1**: Refine Velocity-Flat-G1 baseline, add AMP, SMP baselines
- [ ] **WM**: Add supermimic style world model [SHAC](https://arxiv.org/abs/2204.07137) baseline 
- [ ] **Go2**: Refine Velocity-Rough-Go2 baseline
- [ ] **B2**: Add Velocity-Rough-B2 baseline
- [ ] **Warp MDP**: This is isaaclab3.0 experimental feature, not recommended as daily use. This accelerates rollout a bit, but probably comes with less simulation stability
- [ ] **MjWarp**: Study more about `newton_mjwarp`, we observe physx still shows more stable simulation in especially non-flat terrain tasks
- [ ] **AGENTS.md / SKILLS**: Comprehensive and clean guidance for ai agents

<!-- - [ ] **Active Adaptation Port**: Migrate LocoLab tasks to (AA)[https://github.com/Agent-3154/active-adaptation] -->


