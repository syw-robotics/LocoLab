# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Build LocoLab's frontend fixes without changing Isaac Lab's Viser version."""

import hashlib
import importlib.metadata
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def main() -> None:
    version = importlib.metadata.version("viser")
    if version != "1.0.30":
        raise SystemExit(f"Viser {version}: frontend patches are currently validated only for 1.0.30.")

    from viser import _client_autobuild

    client = _client_autobuild.client_dir
    patches_dir = Path(__file__).resolve().parents[2] / "patches"
    patches = [
        ("client", patches_dir / "viser-1.0.30-batched-glb.patch"),
        ("client", patches_dir / "viser-1.0.30-resizable-images.patch"),
        (".", patches_dir / "viser-1.0.30-asset-cache.patch"),
    ]
    patch_hash = hashlib.sha256(b"".join(path.read_bytes() for _, path in patches)).hexdigest()
    index = client / "build/index.html"
    marker = client / "build/.locolab-patches"
    if marker.exists() and index.exists():
        if marker.read_text() == patch_hash + hashlib.sha256(index.read_bytes()).hexdigest():
            print("[INFO] Viser frontend patches are already installed.")
            return

    with tempfile.TemporaryDirectory(prefix="locolab-viser-") as temporary:
        staging = Path(temporary)
        staged_client = staging / "client"
        shutil.copytree(client, staged_client, ignore=shutil.ignore_patterns("node_modules", ".nodeenv", "build"))
        backend_files = ("_viser.py", "infra/_infra.py")
        for relative in backend_files:
            target = staging / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(client.parent / relative, target)
        for directory, patch in patches:
            applied = subprocess.run(
                ["git", "apply", "--reverse", "--check", str(patch)], cwd=staging / directory, capture_output=True
            ).returncode == 0
            if not applied:
                subprocess.run(["git", "apply", str(patch)], cwd=staging / directory, check=True)

        build_env = os.environ.copy()
        npm = shutil.which("npm")
        if npm is None:
            _client_autobuild.client_dir = staged_client
            node_bin = _client_autobuild._install_sandboxed_node()
            build_env["PATH"] = str(node_bin) + os.pathsep + build_env.get("PATH", "")
            npm = str(node_bin / "npm")
        subprocess.run(
            [npm, "ci", "--prefer-offline", "--no-audit", "--no-fund"], cwd=staged_client, env=build_env, check=True
        )
        subprocess.run([npm, "run", "build"], cwd=staged_client, env=build_env, check=True)

        for relative in (
            "client/src/mesh/BatchedGlbAsset.tsx",
            "client/src/components/Image.tsx",
            "client/src/mesh/GlbLoaderUtils.ts",
            "client/build/index.html",
            *backend_files,
        ):
            target = client.parent / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            # Replace files without modifying uv's shared hardlinks in place.
            staged = target.with_suffix(target.suffix + ".locolab-new")
            shutil.copy2(staging / relative, staged)
            staged.replace(target)
        marker.write_text(patch_hash + hashlib.sha256(index.read_bytes()).hexdigest())
    print("[INFO] Installed Viser frontend fixes and HTTP asset cache support.")


if __name__ == "__main__":
    main()
