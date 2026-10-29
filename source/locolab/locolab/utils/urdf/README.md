# URDF / MJCF → USD

`assetx_export_usd_cfg` (`locolab.assets.robots.common`) exports a URDF or MJCF with [assetx](https://github.com/syw-robotics/assetx.git) and returns an Isaac Lab `UsdFileCfg`. locolab installs assetx from that git URL. `uv` must be on `PATH`; Go2 / B2 DAE also need `pycollada` (see the repo README). Fix conversion bugs in assetx.

## Call

`assetx_export_usd_cfg` calls `assetx_export.assetx_export_usd`. A valid cache returns the existing USD. Otherwise the current interpreter runs:

```text
python convert.py <source> -o <cache> [urdf flags]
```

The parent reads the newest `.usd` / `.usda` / `.usdc` under `cache/usd/`. assetx writes that file in an isolated `uv run`, so the converter's `pxr` stays out of the Isaac environment.

## convert.py

`.xml` / `.mjcf` (G1) is saved as written. URDF flags are dropped.

A URDF is edited, then saved:

1. `urdf_to_mjcf`, resolving `package://` (`-p NAME=PATH`, repeatable).
2. Cylinders become capsules (`--no-replace-cylinders` keeps them).
3. Add `floating_base_joint` unless the root already has a free joint (`--fix-base` skips this).
4. Merge jointless bodies leaf-first with `MergeBodiesParentChild` (`--no-merge-fixed` skips this). The walk starts at `worldbody` children and does not read the world body's `.parent` (MuJoCo 3.11 segfault).

These fixed-joint children stay separate: `--keep-fixed REGEX` (full match; Go2 / B2 use `.*_foot_joint`), and joints marked `dont_collapse="true"` (`--no-honor-dont-collapse` ignores that).

Collision geoms land under `{link}/collisions`, visuals under `visuals`. Foot friction should match `collisions`. Viser reads `LocoLabArticulationCfg.source_asset` (the spawned MJCF or URDF), not this USD.

## Cache

`<source_dir>/.locolab_usd_cache/<hash>/`, gitignored. The hash covers CLI flags, the source, referenced meshes, `convert.py`, and the installed assetx sources.

```bash
python source/locolab/locolab/utils/urdf/convert.py path/to/robot.urdf -o /tmp/out
```

USD is written to `/tmp/out/usd/`.
