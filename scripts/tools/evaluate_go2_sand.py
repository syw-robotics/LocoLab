"""Go2 stepping across a small dry-sand insert, using Alloy's bundled Go2.

The standard coupling is the reference. Pass --coupling fast to compare the cheaper exchange.
Run with an isolated Python environment:
    uv run --no-project --isolated --python 3.12 --with alloy-engine==0.1.9 scripts/tools/evaluate_go2_sand.py
"""

import argparse
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import alloy
import numpy as np
import warp as wp
from alloy.cards.library import default_root
from scipy.spatial.transform import Rotation

DT = 0.02
DEPTH = 0.10
# A recess in the left foot's path. Top of the rigid surround and initial sand surface are z=0.
X0, X1 = 0.0, 0.15
Y0, Y1 = 0.077, 0.207
FOOT_RADIUS = 0.022


def build(terrain, device, coupling):
    scene = alloy.Scene(dt=DT)
    scene.add_ground(z=-DEPTH)

    def tile(name, x0, x1, y0, y1, **kwargs):
        scene.add_box(name, size=(x1 - x0, y1 - y0, DEPTH), pos=((x0 + x1) / 2, (y0 + y1) / 2, -DEPTH / 2), **kwargs)

    tile("left_edge", -1, X0, Y0, Y1, static=True)
    tile("right_edge", X1, 1, Y0, Y1, static=True)
    tile("near_edge", -1, 1, -1, Y0, static=True)
    tile("far_edge", -1, 1, Y1, 1, static=True)
    if terrain == "hard":
        tile("insert", X0, X1, Y0, Y1, static=True)
    elif terrain in ("sand", "unloaded"):
        tile("sand", X0, X1, Y0, Y1, material="sand_silica_dry")

    xml = ET.parse(default_root() / "robots/emb_e8_go2/scene.xml")
    world = xml.find("worldbody")
    world.remove(world.find("geom[@name='floor']"))
    world.remove(world.find("body[@name='obstacle']"))
    # The unloaded control runs the same articulated robot on the solid lane beside the insert.
    scene.add_robot(
        "go2", mjcf=ET.tostring(xml.getroot(), encoding="unicode"), pos=(0, -0.55 if terrain == "unloaded" else 0, 0)
    )
    return alloy.build(scene, device=device, seed=0, mpm_coupling=coupling)


def gait(t):
    # Open-loop diagonal foot trajectories, converted to the MJCF's standing-pose joint coordinates.
    amplitude = np.clip((t - 0.12) / 0.25, 0, 1)
    controls = {}
    for leg, offset in (("FL", 0), ("RR", 0), ("FR", 0.5), ("RL", 0.5)):
        phase = ((t - 0.12) / 0.46 + offset) % 1
        if phase < 0.6:
            x, lift = (0.5 - phase / 0.6) * 0.095, 0
        else:
            swing = (phase - 0.6) / 0.4
            x, lift = (swing - 0.5) * 0.095, 0.055 * math.sin(math.pi * swing)
        height = 0.426 * math.cos(0.9) - amplitude * lift
        calf = -2 * math.acos(math.hypot(amplitude * x, height) / 0.426)
        thigh = math.atan2(amplitude * x, height) - calf / 2
        controls[f"go2/{leg}_thigh"] = thigh - 0.9
        controls[f"go2/{leg}_calf"] = calf + 1.8
    return controls


def inside_insert(feet):
    return (
        (feet[:, 0] > X0 + FOOT_RADIUS)
        & (feet[:, 0] < X1 - FOOT_RADIUS)
        & (feet[:, 1] > Y0 + FOOT_RADIUS)
        & (feet[:, 1] < Y1 - FOOT_RADIUS)
    )


def run(terrain, device, coupling, frames=None):
    sim = build(terrain, device, coupling)
    steps = 65
    has_sand = terrain in ("sand", "unloaded")
    if has_sand and sim.routes()["sand"]["route"] != "grain_mpm":
        raise RuntimeError("Dry sand did not select the granular MPM solver")
    if frames is not None:
        from alloy.camera.view import write_png

        camera = alloy.Camera(
            "go2_sand",
            look_at=((0.52, 0.64, 0.48), (0.015, 0.055, 0.10)),
            width=960,
            height=640,
            fov=43,
            spp=16,
        )
        frames.mkdir(parents=True, exist_ok=False)
        write_png(frames / "frame_000.png", sim.render(camera)[0])
    feet, bases = [], []
    for k in range(steps):
        if k == 1:  # Exclude building the scene and first-step JIT / graph setup.
            wp.synchronize_device(device)
            tic = time.perf_counter()
        sim.set_controls(gait(k * DT))
        sim.step()
        state = sim.state(["go2/base", "go2/FL_calf"], world=0)
        calf_rot = Rotation.from_quat(state["go2/FL_calf.quat_xyzw"])
        toe = state["go2/FL_calf.pos"] + calf_rot.apply((-0.002, 0, -0.213))
        toe[2] -= FOOT_RADIUS
        feet.append(toe)
        bases.append(state["go2/base.pos"])
        if frames is not None:
            write_png(frames / f"frame_{k + 1:03d}.png", sim.render(camera)[0])
    wp.synchronize_device(device)
    rate = (steps - 1) / (time.perf_counter() - tic)
    feet = np.array(feet)
    in_bed = inside_insert(feet)
    points = sim.state("sand", world=0)["sand.points"] if has_sand else None
    health = sim.health()
    if (
        not sim.check()["ok"]
        or health["invalid_worlds"]
        or not np.isfinite(bases).all()
        or not np.isfinite(feet).all()
        or (has_sand and not np.isfinite(points).all())
    ):
        raise RuntimeError(f"{terrain} rollout failed: {health}")
    deepest = feet[in_bed][np.argmin(feet[in_bed, 2])] if in_bed.any() else None
    result = dict(
        control_steps_per_second=round(rate, 2) if frames is None else None,
        base_final_xyz_m=np.round(bases[-1].astype(float), 4).tolist(),
        samples_inside_insert=int(in_bed.sum()),
        min_base_z_m=round(float(np.min(np.array(bases)[:, 2])), 4),
        lowest_foot_z_mm=round(1000 * deepest[2], 2) if deepest is not None else None,
        particle_count=len(points) if has_sand else 0,
        health_warnings=[health["worlds"][i] for i in health["degraded_worlds"]],
    )
    if terrain == "unloaded" and deepest is not None:
        raise RuntimeError("The unloaded control stepped into the sand insert.")
    if terrain != "unloaded" and deepest is None:
        raise RuntimeError(f"The front left foot missed the {terrain} insert.")
    print(terrain, json.dumps(result), flush=True)
    return feet, points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--coupling", choices=("standard", "fast"), default="standard")
    parser.add_argument("--terrain", choices=("all", "hard", "sand", "empty", "unloaded"), default="all")
    parser.add_argument("--frames", type=Path, help="Write 50 fps PNG frames into a new directory for one terrain.")
    args = parser.parse_args()
    if args.frames is not None and args.terrain == "all":
        parser.error("--frames requires selecting one terrain with --terrain")
    terrains = ("hard", "empty", "sand", "unloaded") if args.terrain == "all" else (args.terrain,)
    results = {terrain: run(terrain, args.device, args.coupling, args.frames) for terrain in terrains}
    if args.terrain == "all":
        feet, loaded = results["sand"]
        within = feet[inside_insert(feet)]
        foot_xy = within[np.argmin(within[:, 2]), :2]

        # Median surface height over identical 5 mm cells beneath the foot, with empty cells at the tray floor.
        def surface(points):
            cell = 0.005
            shape = (round((X1 - X0) / cell), round((Y1 - Y0) / cell))
            tops = np.full(shape, -DEPTH)
            ix, iy = np.floor((points[:, :2] - [X0, Y0]) / cell).astype(int).T
            valid = (ix >= 0) & (ix < shape[0]) & (iy >= 0) & (iy < shape[1])
            np.maximum.at(tops, (ix[valid], iy[valid]), points[valid, 2])
            x, y = (np.indices(shape) + 0.5) * cell
            footprint = np.hypot(x + X0 - foot_xy[0], y + Y0 - foot_xy[1]) <= FOOT_RADIUS
            return float(np.median(tops[footprint]))

        unloaded_surface = surface(results["unloaded"][1])
        loaded_surface = surface(loaded)
        print(
            "footprint",
            json.dumps(
                dict(
                    center_xy_m=foot_xy.round(4).tolist(),
                    loaded_surface_mm=round(1000 * loaded_surface, 2),
                    unloaded_surface_mm=round(1000 * unloaded_surface, 2),
                    loaded_minus_unloaded_mm=round(1000 * (loaded_surface - unloaded_surface), 2),
                )
            ),
        )


if __name__ == "__main__":
    main()
