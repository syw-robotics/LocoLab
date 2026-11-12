# LocoLab export for unitree-deploy

This bundle is for unitree-deploy. Copy this **entire directory** to
`ckpt/{{robot}}/<policy>/` in unitree-deploy. LocoLab and Isaac Lab are not required there.

- `policy.onnx`: actor with observation normalization included.
- `policy.yaml`: joint layout, observations, actions, timing, and PD gains.
- `multi_ckpt.yaml`, `state_machine.yaml`: locomotion/tracking switching.

Reference motion is embedded in ONNX; no source NPZ or Isaac Lab is needed.
The bundle supplies reference joints and body trajectories for alignment,
residual actions, and the motion ghost. The actor includes its trained normalization.
Keep a compatible velocity policy in `../vanilla_ppo_flat`, or edit its path in
`multi_ckpt.yaml`. A moves to the default pose, Start enters locomotion, B prepares
the arms and starts tracking; the final frame returns to locomotion. X enters damping.

Run from unitree-deploy, using separate terminals:

```sh
unitree-sim-bridge --robot {{robot}}
unitree-controller --mode sim --robot {{robot}} --multi-ckpt ckpt/{{robot}}/<policy>/multi_ckpt.yaml
```

`policy.yaml` records the exported training layout. Changing observation order,
joint order, action scale, or timestep requires a correspondingly trained policy.
Use `--mode real --net <interface>` for the controller on hardware after sim validation.
