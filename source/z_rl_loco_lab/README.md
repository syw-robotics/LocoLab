# z_rl_loco_lab

`z_rl_loco_lab` is a `z_rl` plugin with locomotion-specific PPO variants and actor models.

It currently provides:

- Barlow Twins history-encoder PPO
- DreamWaQ-style VAE history-encoder PPO
- Isaac Lab config classes for the custom algorithms and models

## Install

From this package directory:

```bash
python -m pip install -e .
```

## Structure

```text
z_rl_loco_lab/
├── algorithms/
│   ├── barlowtwins_encoder_ppo.py
│   └── dreamwaq_encoder_ppo.py
├── models/
│   ├── barlowtwins_model.py
│   └── dreamwaq_model.py
└── rl_cfg.py
```

## Algorithms

Both algorithms follow the `ComposablePPO` pattern:

- define a `PPOLossSpec`
- subclass `ComposablePPO`
- build auxiliary-loss targets from `env.obs_format` and `env.obs_group_time_slice_map`

### Barlow Twins

`BarlowTwinsPPO` trains a policy history encoder with two auxiliary losses:

- linear velocity estimation from privileged critic observations
- Barlow Twins representation loss between shifted history windows

Use `BarlowTwinsEncoderMLPModelCfg` and `BarlowTwinsPpoAlgorithmCfg` from `z_rl_loco_lab.rl_cfg`.

### DreamWaQ

`DreamWaQPPO` trains a VAE-style history encoder with:

- linear velocity estimation from privileged critic observations
- reconstruction plus KL VAE loss for selected critic observation terms

Use `DreamWaQEncoderMLPModelCfg` and `DreamWaQPpoAlgorithmCfg` from `z_rl_loco_lab.rl_cfg`.

## Go2 Configs

Go2 task-specific runner configs live in the LocoLab task package:

```text
locolab/tasks/manager_based/locomotion/velocity/config/unitree_go2/agents/
├── z_rl_barlowtwins_ppo_cfg.py
└── z_rl_dreamwaq_ppo_cfg.py
```

Registered task IDs:

- `Velocity-Rough-Go2-ZRL-BarlowTwins`
- `Velocity-Rough-Go2-ZRL-DreamWaQ`
