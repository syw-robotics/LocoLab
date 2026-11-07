# LocoLab export for unitree-deploy

This bundle is for unitree-deploy. Copy this **entire directory** to
`ckpt/{{robot}}/<policy>/` in unitree-deploy. LocoLab and Isaac Lab are not required there.

- `policy.onnx`: actor with observation normalization included.
- `policy.yaml`: joint layout, observations, actions, timing, and PD gains.

Use a unitree-deploy version with `unitree_deploy.policy.velocity:VelocityPolicy`.
The deployment-side policy preserves clipped previous actions and per-joint position-target limits.
Training noise, encoder-bias randomization, and simulated actuator delays are not
applied at deployment. Joint velocity observations require zero default joint velocities.
A moves to the default pose, Start runs the policy, and X enters damping.
Joystick limits are in `observations` -> `command_range` in `policy.yaml`.

Run from unitree-deploy, using separate terminals:

```sh
unitree-sim-bridge --robot {{robot}}
unitree-controller --mode sim --robot {{robot}} --ckpt ckpt/{{robot}}/<policy>
```

`policy.yaml` records the exported training layout. Changing observation order,
joint order, action scale, or timestep requires a correspondingly trained policy.
Use `--mode real --net <interface>` for the controller on hardware after sim validation.
