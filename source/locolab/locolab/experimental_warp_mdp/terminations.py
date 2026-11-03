"""LocoLab's orientation termination on the Warp manager interface."""

import warp as wp

from isaaclab_experimental.managers import SceneEntityCfg

from .rewards import _contact_count


@wp.kernel(enable_backward=False)
def _orientation(
    pose: wp.array(dtype=wp.transform), gravity: wp.array(dtype=wp.vec3), limit: float, out: wp.array(dtype=bool)
):
    i = wp.tid()
    projected = wp.quat_rotate_inv(wp.transform_get_rotation(pose[i]), wp.normalize(gravity[i]))
    out[i] = wp.abs(wp.acos(-projected[2])) > limit


def bad_orientation(env, out, limit_angle, asset_cfg=SceneEntityCfg("robot")):
    data = env.scene[asset_cfg.name].data
    wp.launch(
        _orientation,
        dim=env.num_envs,
        inputs=[data.root_link_pose_w.warp, data.GRAVITY_VEC_W.warp, limit_angle, out],
        device=env.device,
    )


@wp.kernel(enable_backward=False)
def _illegal_contact(
    forces: wp.array3d(dtype=wp.vec3), ids: wp.array(dtype=int), threshold: float, out: wp.array(dtype=bool)
):
    i = wp.tid()
    out[i] = _contact_count(forces, ids, threshold, i) > 0


def illegal_contact(env, out, threshold, sensor_cfg):
    wp.launch(
        _illegal_contact,
        dim=env.num_envs,
        inputs=[
            env.scene.sensors[sensor_cfg.name].data.net_forces_w_history.warp,
            sensor_cfg.body_ids_wp,
            threshold,
            out,
        ],
        device=env.device,
    )
