# Shared entry points and external projects

LocoLab owns the train/play implementation. An external project supplies tasks, environment and agent
configs, and any custom models, algorithms, or runners. Both Z-RL and RSL-RL use this arrangement.

## Register and run tasks

Declare LocoLab as a dependency and register the module that imports your Gym tasks in the external
package's `pyproject.toml`:

```toml
[project.entry-points."locolab.tasks"]
my_project = "my_project.tasks"
```

Install that package into the same environment as LocoLab. From the external project's directory:

```bash
python -m locolab.entrypoints.z_rl.train --task MyTask
python -m locolab.entrypoints.z_rl.play --task MyTask --viser
python -m locolab.entrypoints.rsl_rl.train --task MyTask
python -m locolab.entrypoints.rsl_rl.play --task MyTask-Play --viser
```

Task registrations must provide the corresponding `z_rl_cfg_entry_point` or `rsl_rl_cfg_entry_point`.
Z-RL playback appends `-Play` when absent; RSL-RL retains the task ID you pass, matching the existing
script behavior. All existing flags and Hydra overrides remain available. Use `--help` for details.

Logs and relative paths are resolved from the current working directory. Training registers the
LocoLab source location and the current directory for Git state logging. No fixed checkout layout,
symlinks, or copied CLI helpers are required.

## Customize an entry point

For existing configuration fields, prefer Hydra overrides, for example `agent.seed=42`.
If a project needs a new CLI argument, its script can be a small wrapper:

```python
from locolab.entrypoints.z_rl.train import main


def add_arguments(parser):
    parser.add_argument("--episode-seconds", type=float, default=None)


def configure(args, env_cfg, agent_cfg):
    if args.episode_seconds is not None:
        env_cfg.episode_length_s = args.episode_seconds


if __name__ == "__main__":
    main(add_arguments=add_arguments, configure=configure)
```

The same two keyword arguments are supported by all four train/play modules:

- `add_arguments(parser)` runs after the standard arguments are registered and before parsing.
  Project arguments participate in `--help` and are consumed before Hydra processes its overrides.
- `configure(args, env_cfg, agent_cfg)` mutates the resolved configs in place, after the standard
  task/agent CLI overrides and before seed/device setup, log path selection, and simulation startup.
  It returns nothing. Later checkpoint restoration and normal launch preparation still apply.

Importing an entry point does not parse arguments, register tasks, or start the simulation.
The wrapper only needs to import `main`; a local `cli_args.py` is unnecessary.

## Custom runners

Set the agent config's `class_name` to an importable `module:Class` reference:

```python
class_name = "my_project.runners:MyRunner"
```

The class must implement the selected RL library's runner interface, including the constructor,
training/checkpoint methods, and the inference/export methods used during playback. LocoLab also
continues to accept its existing built-in runner names. RSL-RL uses the current Isaac Lab 3.0 runner
API, including `export_policy_to_jit`, `export_policy_to_onnx`, and the inference policy's `reset`.

These callbacks customize arguments and configuration; they do not replace the training or playback
loop. Algorithm-specific behavior belongs in the runner or task implementation.

The archived `locolab_external_project_template` generator remains deprecated. Its copies of
train/play scripts are not needed for this workflow.

## Project-specific Z-RL playback

Z-RL playback also accepts two optional callbacks:

- `wrap_env(args, env)` returns the playback environment, optionally wrapped for diagnostics.
  It runs after policy loading and exports, before the viewer opens. A wrapper must forward
  environment attributes and methods; its `step` and `close` can collect and report diagnostics.
- `export_onnx(runner, policy, path)` owns project-specific ONNX artifacts when `--export_onnx`
  is passed. It replaces the default ONNX/deployment bundle. Playback closes the environment
  and exits after this callback, including on export failure.

Neither callback changes training or adds work to the default playback loop.
