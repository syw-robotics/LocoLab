# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""URDF/MJCF → Isaac Lab USD via assetx.

``assetx_export.py`` runs this file as a subprocess. See README.md here for the
pipeline.
"""

from __future__ import annotations

import argparse
import re
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

from assetx import AddJoint, JointCfg, MergeBodiesParentChild, MujocoAsset, ReplaceCylinderWithCapsule
from assetx.conversion.urdf2mjcf import urdf_to_mjcf


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert URDF/MJCF to Isaac Lab USD via assetx.")
    parser.add_argument("urdf", type=Path)
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--no-merge-fixed", action="store_true")
    parser.add_argument("--keep-fixed", default=None)
    parser.add_argument("--no-honor-dont-collapse", action="store_true")
    parser.add_argument("--no-replace-cylinders", action="store_true")
    parser.add_argument("--fix-base", action="store_true")
    parser.add_argument("-p", "--package", action="append", default=None, metavar="NAME=PATH")
    args = parser.parse_args()

    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    if args.urdf.suffix.lower() in {".xml", ".mjcf"}:
        MujocoAsset.from_file(args.urdf).save(out, save_urdf=False, save_usd=True)
    else:
        packages: dict[str, Path] = {}
        for item in args.package or []:
            name, sep, path = item.partition("=")
            if not sep or not name or not path:
                parser.error(f"--package expects NAME=PATH, got {item!r}")
            packages[name] = Path(path)
        with tempfile.TemporaryDirectory(prefix="locolab-urdf-") as tmp:
            _, _, xml_path = urdf_to_mjcf(
                args.urdf.resolve(),
                out_xml=Path(tmp) / "raw.xml",
                package_paths=packages or None,
            )
            asset = MujocoAsset.from_file(xml_path)
            if not args.no_replace_cylinders:
                asset = ReplaceCylinderWithCapsule().transform(asset)
            if not args.fix_base:
                root = asset.spec.worldbody.first_body()
                if root is not None and not any(
                    joint.type == mujoco.mjtJoint.mjJNT_FREE for joint in root.joints
                ):
                    asset = AddJoint(
                        root.name,
                        JointCfg(name="floating_base_joint", type="free", limited=False),
                    ).transform(asset)
            if not args.no_merge_fixed:
                keep = _links_to_keep(args.urdf, args.keep_fixed, args.no_honor_dont_collapse)
                asset = _merge_fixed_bodies(asset, keep)
            asset.save(out, save_usd=True)

    usd_files = sorted(
        (path for path in (out / "usd").glob("*") if path.suffix in {".usd", ".usda", ".usdc"}),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not usd_files:
        raise RuntimeError(f"USD conversion produced no file under {out / 'usd'}")
    print(usd_files[0])


def _links_to_keep(urdf_path: Path, keep_fixed: str | None, skip_dont_collapse: bool) -> set[str]:
    """Child links of fixed joints that must stay separate bodies."""
    if keep_fixed is None and skip_dont_collapse:
        return set()
    pattern = re.compile(keep_fixed) if keep_fixed is not None else None
    keep: set[str] = set()
    for joint in ET.parse(urdf_path).getroot().iter():
        tag = joint.tag.rsplit("}", 1)[-1]
        if tag != "joint" or joint.get("type") != "fixed":
            continue
        child = next((elem for elem in joint if elem.tag.rsplit("}", 1)[-1] == "child"), None)
        link = None if child is None else child.get("link")
        if not link:
            continue
        name = joint.get("name") or ""
        if pattern is not None and pattern.fullmatch(name):
            keep.add(link)
        if not skip_dont_collapse and joint.get("dont_collapse") == "true":
            keep.add(link)
    return keep


def _merge_fixed_bodies(asset: MujocoAsset, keep: set[str]) -> MujocoAsset:
    """Leaf-first merge of jointless bodies via ``MergeBodiesParentChild``.

    Walks from ``worldbody`` children so the world body's ``.parent`` is never
    read (MuJoCo 3.11 segfault).
    """

    def pending(body: mujoco.MjsBody, groups: dict[str, list[str]], *, is_root: bool) -> bool:
        child_pending = False
        for child in body.bodies:
            if pending(child, groups, is_root=False):
                child_pending = True
        mergeable = not is_root and not body.joints and body.name not in keep
        if mergeable and not child_pending:
            groups.setdefault(body.parent.name, []).append(body.name)
        return mergeable or child_pending

    for _ in range(10_000):
        groups: dict[str, list[str]] = {}
        for body in asset.spec.worldbody.bodies:
            pending(body, groups, is_root=True)
        if not groups:
            return asset
        parent, children = next(iter(groups.items()))
        asset = MergeBodiesParentChild(parent, children).transform(asset)
    raise RuntimeError("fixed-joint merge did not finish")


if __name__ == "__main__":
    main()
