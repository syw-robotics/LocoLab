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

- Install Isaac Lab 3.0 with uv by following the [installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html).
  From the Isaac Lab checkout, `uv sync --extra isaacsim` creates the environment. `isaaclab.sh` and the conda installer are no longer part of Isaac Lab 3.0.

- Clone `LocoLab` separately from the Isaac Lab installation (i.e. outside the `IsaacLab` directory):

    ```bash
    git clone https://github.com/syw-robotics/LocoLab.git
    ```

- Using the Isaac Lab 3.0 interpreter, install the library in editable mode:

    ```bash
    uv run --project /path/to/IsaacLab python -m pip install -e source/locolab
    ```

- Install [`Z RL`](https://github.com/syw-robotics/z_rl.git), which is built on [`RSL-RL`](https://github.com/leggedrobotics/rsl_rl) but more flexible for developing algorithms.
    ```bash
    git clone https://github.com/syw-robotics/z_rl.git
    cd z_rl
    uv run --project /path/to/IsaacLab python -m pip install -e .
    ```

## 🚀 Usage
- **Training:**

    ```bash
    python scripts/z_rl/train.py --task=<TASK_NAME>
    ```

    - Isaac Lab 3.0 no longer accepts `--headless`;
    - The default backend is PhysX (`--physics physx`). `--physics newton_mjwarp` runs the same task on Newton with MJWarp. Presets are defined in [`physics.py`](source/locolab/locolab/utils/physics.py).

- **Playing:**
    ```bash
    python scripts/z_rl/play.py --task=<TASK_NAME>
    ```

    - Add `--viz xxx` to use xxx front end viewer. A run without `--viz` stays headless.
    - Still keeps `--viser`, this is a previous implemented viser viewer option.
    - `--video` records through `VideoRecorderCfg` with the Kit visualizer (`viz:kit`).

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


