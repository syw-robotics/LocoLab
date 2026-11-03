"""LocoLab observation terms not supplied by the experimental package."""

from types import SimpleNamespace

import warp as wp

from isaaclab_experimental.envs.utils.io_descriptors import generic_io_descriptor_warp
from isaaclab_experimental.managers import ManagerTermBase, SceneEntityCfg


@wp.kernel(enable_backward=False)
def _gather(data: wp.array2d(dtype=float), ids: wp.array(dtype=int), out: wp.array2d(dtype=float)):
    i, j = wp.tid()
    out[i, j] = data[i, ids[j]]


@generic_io_descriptor_warp(out_dim="joint")
def joint_acc(env, out, asset_cfg=SceneEntityCfg("robot")):
    wp.launch(
        _gather,
        dim=out.shape,
        inputs=[env.scene[asset_cfg.name].data.joint_acc.warp, asset_cfg.joint_ids_wp, out],
        device=env.device,
    )


@generic_io_descriptor_warp(out_dim="joint")
def joint_effort(env, out, asset_cfg=SceneEntityCfg("robot")):
    wp.launch(
        _gather,
        dim=out.shape,
        inputs=[env.scene[asset_cfg.name].actuators.applied_effort.warp, asset_cfg.joint_ids_wp, out],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _biased_position(
    position: wp.array2d(dtype=float),
    defaults: wp.array2d(dtype=float),
    bias: wp.array2d(dtype=float),
    ids: wp.array(dtype=int),
    out: wp.array2d(dtype=float),
):
    i, j = wp.tid()
    k = ids[j]
    out[i, j] = position[i, k] + bias[i, k] - defaults[i, k]


@generic_io_descriptor_warp(out_dim="joint")
def joint_pos_rel_biased(env, out, asset_cfg=SceneEntityCfg("robot")):
    asset = env.scene[asset_cfg.name]
    wp.launch(
        _biased_position,
        dim=out.shape,
        inputs=[
            asset.data.joint_pos.warp,
            asset.data.default_joint_pos.warp,
            asset.encoder_bias_wp,
            asset_cfg.joint_ids_wp,
            out,
        ],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _contact_flag(forces: wp.array2d(dtype=wp.vec3), ids: wp.array(dtype=int), out: wp.array2d(dtype=float)):
    i, j = wp.tid()
    f = forces[i, ids[j]]
    out[i, j] = float(f[0] > 1.0 or f[1] > 1.0 or f[2] > 1.0) - 0.5


@generic_io_descriptor_warp(out_dim="body:1")
def feet_contact_flag(env, out, sensor_cfg=SceneEntityCfg("contact_forces", body_names=".*_foot")):
    wp.launch(
        _contact_flag,
        dim=out.shape,
        inputs=[env.scene.sensors[sensor_cfg.name].data.net_forces_w.warp, sensor_cfg.body_ids_wp, out],
        device=env.device,
    )


@generic_io_descriptor_warp(out_dim="command")
def generated_commands(env, out, command_name):
    # Avoid the upstream function-global cache, which can retain a previous environment's command buffer.
    wp.copy(out, wp.from_torch(env.command_manager.get_command(command_name)))


@wp.kernel(enable_backward=False)
def _height_scan(
    position: wp.array(dtype=wp.vec3), hits: wp.array2d(dtype=wp.vec3), offset: float, out: wp.array2d(dtype=float)
):
    i, j = wp.tid()
    out[i, j] = position[i][2] - hits[i, j][2] - offset


class height_scan(ManagerTermBase):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self._sensor = env.scene.sensors[cfg.params["sensor_cfg"].name]
        self._descriptor = SimpleNamespace(out_dim=self._sensor.data.ray_hits_w.warp.shape[1])

    def __call__(self, env, out, sensor_cfg, offset=0.5):
        data = self._sensor.data
        wp.launch(
            _height_scan, dim=out.shape, inputs=[data.pos_w.warp, data.ray_hits_w.warp, offset, out], device=env.device
        )
