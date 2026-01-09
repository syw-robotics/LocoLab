<h2 align="center"> LocoLab: Starter Environment Suite for Locomotion Task Training and Evaluation</h2>

<div align="center">

<!-- [[Video]](https://www.youtube.com/watch?v=tu7LSNYWDTs&ab_channel=LeCARLabatCMU) -->



<!-- [![IsaacSim](https://img.shields.io/badge/IsaacSim-5.1.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html) -->
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-2.3.1-brightgreen?logo=nvidia&logoColor=white)](https://isaac-sim.github.io/IsaacLab)
[![RSL RL](https://img.shields.io/badge/RSL_RL-3.0.1-silver?logo=pytorch&logoColor=white)](https://isaac-sim.github.io/IsaacLab)
[![Python](https://img.shields.io/badge/python-3.11-blue?logo=python&logoColor=white)](https://docs.python.org/3/whatsnew/3.11.html)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-orange?logo=pre-commit&logoColor=white)](https://pre-commit.com/)

<!-- [![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![Windows platform](https://img.shields.io/badge/platform-windows--64-orange.svg)](https://www.microsoft.com/en-us/) -->
<!-- [![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg)](https://opensource.org/license/apache-2-0) -->


</div>

## Overview

This codebase serves as a starter **manager based rl** environment for building locomotion projects on Isaac Lab.

**Key Features:**

- `Flexibility`: Easy to modifiy, reuse existing modules, and adapt to new tasks
- `Experiment Friendly`: Happy to do experiments

**Tested and deployable tasks are:**
- Velocity-Flat-Go2
- Velocity-Rough-Go2


## Installation

- Install Isaac Lab by following the [installation guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html).
  We recommend using the conda or uv installation as it simplifies calling Python scripts from the terminal.

- Clone `LocoLab` separately from the Isaac Lab installation (i.e. outside the `IsaacLab` directory):

    ```bash
    git clone https://github.com/syw-robotics/LocoLab.git
    ```

- Using a python interpreter that has Isaac Lab installed, install the library in editable mode using:

    ```bash
    python -m pip install -e source/loco_lab
    ```

## Usage
- **Helpful scripts:**

    - Listing the available tasks:

        ```bash
        python scripts/list_envs.py
        ```

    - Running a task with a random agent for testing:

        ```bash
        python scripts/random_agent.py --task=<TASK_NAME>
        ```

    - Fetch a trainning logging from remote server:
        ```bash
        ./scripts/sync_logs.sh --help  # check this script for usage
        ```

- **Training:**

    ```bash
    python scripts/rsl_rl/train.py --task=<TASK_NAME>
    ```

- **Playing:**
    ```bash
    python scripts/rsl_rl/train.py --task=<TASK_NAME>
    ```

- **Deployment:**

    For sim2im and sim2real deployment, see [go2_isaaclab_deploy](https://github.com/syw-robotics/go2_isaaclab_deploy)

## Task Organization

The tasks are organized in the following hierarchy:

```
tasks
├── manager_based
│   ├── locomotion
│   │   ├── velocity
│   │   │   ├── config
│   │   │   │   ├── unitree_go2
│   │   │   │   │   ├── agents
│   │   │   │   │   │   ├── rsl_rl_ppo_cfg.py
│   │   │   │   │   │   └── ...
│   │   │   │   │   ├── mdp_cfg (mdp configurations)
│   │   │   │   │   ├── flat_env_cfg.py
│   │   │   │   │   ├── rough_env_cfg.py
│   │   │   │   │   └── __init__.py (register the environment cfgs)
│   │   │   └── mdp (mdp components)
│   │   └── ...
│   └── ...
└── ...
```

Different types of tasks are organized in different sub-directories, such that tasks are clearly separated and mdp components are easily reusable.

<!-- ### Set up IDE (Optional)

To setup the IDE, please follow these instructions:

- Run VSCode Tasks, by pressing `Ctrl+Shift+P`, selecting `Tasks: Run Task` and running the `setup_python_env` in the drop down menu.
  When running this task, you will be prompted to add the absolute path to your Isaac Sim installation.

If everything executes correctly, it should create a file .python.env in the `.vscode` directory.
The file contains the python paths to all the extensions provided by Isaac Sim and Omniverse.
This helps in indexing all the python modules for intelligent suggestions while writing code. -->

<!-- ### Setup as Omniverse Extension (Optional)

We provide an example UI extension that will load upon enabling your extension defined in `source/loco_lab/loco_lab/ui_extension_example.py`.

To enable your extension, follow these steps:

1. **Add the search path of this project/repository** to the extension manager:
    - Navigate to the extension manager using `Window` -> `Extensions`.
    - Click on the **Hamburger Icon**, then go to `Settings`.
    - In the `Extension Search Paths`, enter the absolute path to the `source` directory of this project/repository.
    - If not already present, in the `Extension Search Paths`, enter the path that leads to Isaac Lab's extension directory directory (`IsaacLab/source`)
    - Click on the **Hamburger Icon**, then click `Refresh`.

2. **Search and enable your extension**:
    - Find your extension under the `Third Party` category.
    - Toggle it to enable your extension. -->

## Code formatting

We have a pre-commit template to automatically format your code.
To install pre-commit:

```bash
pip install pre-commit
```

Then you can run pre-commit with:

```bash
pre-commit run --all-files
```

<!-- ### Troubleshooting -->
<!--  -->
<!-- #### Pylance Missing Indexing of Extensions -->
<!--  -->
<!-- In some VsCode versions, the indexing of part of the extensions is missing. -->
<!-- In this case, add the path to your extension in `.vscode/settings.json` under the key `"python.analysis.extraPaths"`. -->
<!--  -->
<!-- ```json -->
<!-- { -->
<!--     "python.analysis.extraPaths": [ -->
<!--         "<path-to-ext-repo>/source/loco_lab" -->
<!--     ] -->
<!-- } -->
<!-- ``` -->
<!--  -->
<!-- #### Pylance Crash -->
<!--  -->
<!-- If you encounter a crash in `pylance`, it is probable that too many files are indexed and you run out of memory. -->
<!-- A possible solution is to exclude some of omniverse packages that are not used in your project. -->
<!-- To do so, modify `.vscode/settings.json` and comment out packages under the key `"python.analysis.extraPaths"` -->
<!-- Some examples of packages that can likely be excluded are: -->
<!--  -->
<!-- ```json -->
<!-- "<path-to-isaac-sim>/extscache/omni.anim.*"         // Animation packages -->
<!-- "<path-to-isaac-sim>/extscache/omni.kit.*"          // Kit UI tools -->
<!-- "<path-to-isaac-sim>/extscache/omni.graph.*"        // Graph UI tools -->
<!-- "<path-to-isaac-sim>/extscache/omni.services.*"     // Services tools -->
<!-- ... -->
<!-- ``` -->
