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
    patches = [patches_dir / f"viser-1.0.30-{name}.patch" for name in ("batched-glb", "resizable-images")]
    patch_hash = hashlib.sha256(b"".join(path.read_bytes() for path in patches)).hexdigest()
    index = client / "build/index.html"
    marker = client / "build/.locolab-patches"
    if marker.exists() and index.exists():
        if marker.read_text() == patch_hash + hashlib.sha256(index.read_bytes()).hexdigest():
            print("[INFO] Viser frontend patches are already installed.")
            return

    with tempfile.TemporaryDirectory(prefix="locolab-viser-") as temporary:
        staging = Path(temporary) / "client"
        shutil.copytree(client, staging, ignore=shutil.ignore_patterns("node_modules", ".nodeenv", "build"))
        for patch in patches:
            applied = subprocess.run(
                ["git", "apply", "--reverse", "--check", str(patch)], cwd=staging, capture_output=True
            ).returncode == 0
            if not applied:
                subprocess.run(["git", "apply", str(patch)], cwd=staging, check=True)

        # Use Viser's own isolated Node version, not a system Node installation.
        _client_autobuild.client_dir = staging
        node_bin = _client_autobuild._install_sandboxed_node()
        build_env = os.environ.copy()
        build_env["PATH"] = str(node_bin) + os.pathsep + build_env.get("PATH", "")
        npm = node_bin / "npm"
        subprocess.run([str(npm), "ci", "--no-audit", "--no-fund"], cwd=staging, env=build_env, check=True)
        subprocess.run([str(npm), "run", "build"], cwd=staging, env=build_env, check=True)

        for relative in ("src/mesh/BatchedGlbAsset.tsx", "src/components/Image.tsx", "build/index.html"):
            target = client / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            # Replace files without modifying uv's shared hardlinks in place.
            staged = target.with_suffix(target.suffix + ".locolab-new")
            shutil.copy2(staging / relative, staged)
            staged.replace(target)
        marker.write_text(patch_hash + hashlib.sha256(index.read_bytes()).hexdigest())
    print("[INFO] Installed Viser GLB and resizable-image fixes.")


if __name__ == "__main__":
    main()
