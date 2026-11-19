"""Generate offline previews for all LocoLab custom terrains.

Use the real IsaacLab terrain generators and one CPU ray-casting renderer for
both height-field and mesh terrains. No IsaacSim app or display is needed.
"""

from __future__ import annotations

import argparse
import html
import importlib
import os
import sys
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-locolab-preview")
os.environ.setdefault("WARP_CACHE_PATH", "/tmp/warp-locolab-preview")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
import trimesh
from matplotlib import colors

from isaaclab.terrains.height_field import HfTerrainBaseCfg
from isaaclab.utils.warp import convert_to_warp_mesh, raycast_mesh

from preview_locolab_terrains_cfg import ABSTRACT_CFG_NAMES, PREVIEW_CASES


# Paths are fixed relative to this preview folder, so the script can be run
# either from this directory or from the parent terrains package.
PREVIEW_DIR = Path(__file__).resolve().parent
PACKAGE_PARENT_DIR = PREVIEW_DIR.parent.parent
README_PATH = PREVIEW_DIR.parent / "README.md"
DEFAULT_OUT_DIR = PREVIEW_DIR / "terrain_previews"
README_BEGIN = "<!-- BEGIN_AUTO_PREVIEWS -->"
README_END = "<!-- END_AUTO_PREVIEWS -->"
FIGURE_BG = "#f7f5ef"
HEIGHT_CMAP = colors.LinearSegmentedColormap.from_list(
    "locolab_height",
    ["#24364b", "#316d72", "#8ca980", "#d7c48d", "#b86b4d"],
)


def import_locolab_terrains():
    """Import the actual terrain package; never substitute approximate geometry."""
    sys.path.insert(0, str(PACKAGE_PARENT_DIR))
    return importlib.import_module("locolab_terrains")


def discover_cfgs(pkg) -> dict[str, type]:
    """Find concrete terrain cfg classes exported by locolab_terrains."""
    cfgs = {}
    for name in dir(pkg):
        if name in ABSTRACT_CFG_NAMES or not name.endswith("TerrainCfg"):
            continue
        obj = getattr(pkg, name)
        if isinstance(obj, type):
            cfgs[name] = obj
    return dict(sorted(cfgs.items()))


def build_cfg(cfg_cls: type, common_kwargs: dict, hf_kwargs: dict, case_kwargs: dict):
    """Merge representative parameters using the same sampling scales as training."""
    from locolab_terrains import MeshRoughTerrainCfg

    kwargs = dict(common_kwargs)
    if issubclass(cfg_cls, HfTerrainBaseCfg):
        kwargs.update(hf_kwargs)
    elif issubclass(cfg_cls, MeshRoughTerrainCfg):
        kwargs.update({name: hf_kwargs[name] for name in ("horizontal_scale", "vertical_scale")})
    kwargs.update(case_kwargs)
    return cfg_cls(**kwargs)


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------


def assert_mesh(mesh: trimesh.Trimesh, origin: np.ndarray):
    """Fail fast if a generated terrain is not a usable mesh/origin pair."""
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.faces)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) == 0:
        raise RuntimeError("generated mesh has invalid vertices")
    if faces.ndim != 2 or faces.shape[1] != 3 or len(faces) == 0:
        raise RuntimeError("generated mesh has invalid faces")
    if not np.isfinite(vertices).all():
        raise RuntimeError("generated mesh has non-finite vertices")
    if np.asarray(origin).shape != (3,) or not np.isfinite(origin).all():
        raise RuntimeError(f"generated origin is invalid: {origin}")


def height_normalizer(heights: np.ndarray) -> colors.Normalize:
    """Create a stable color scale even when the terrain has near-constant height."""
    min_height = float(np.min(heights))
    max_height = float(np.max(heights))
    if np.isclose(min_height, max_height):
        pad = max(abs(min_height) * 0.05, 0.05)
        min_height -= pad
        max_height += pad
    return colors.Normalize(vmin=min_height, vmax=max_height)


def add_height_colorbar(fig, ax, normalizer: colors.Normalize):
    """Add the compact height legend used by every preview image."""
    scalar_map = plt.cm.ScalarMappable(norm=normalizer, cmap=HEIGHT_CMAP)
    scalar_map.set_array([])
    colorbar = fig.colorbar(scalar_map, ax=ax, fraction=0.045, pad=0.035)
    colorbar.set_label("height (m)", fontsize=8)
    colorbar.ax.tick_params(labelsize=7, length=2)


def render_mesh(
    mesh: trimesh.Trimesh,
    output_path: Path,
    title: str,
    image_size: int = 640,
    *,
    view: str = "3d",
):
    """Render the nearest actual surface with orthographic rays and metric XYZ scale.

    Both representations use this path, including vertical faces, undersides and
    holes. Colors use intersection height rather than a triangle's mean height.
    """
    vertices = np.asarray(mesh.vertices)
    center = mesh.bounds.mean(axis=0)
    if view == "top":
        right, up, back = np.eye(3)
    else:
        back = np.array([-1.0, -1.0, 1.0]) / np.sqrt(3.0)
        right = np.array([1.0, -1.0, 0.0]) / np.sqrt(2.0)
        up = np.cross(back, right)

    projected = (vertices - center) @ np.array([right, up]).T
    span = max(float(np.ptp(projected, axis=0).max()) * 1.08, 0.1)
    screen_center = (projected.min(axis=0) + projected.max(axis=0)) / 2.0
    distance = float(np.linalg.norm(mesh.extents)) + 1.0
    camera = center + screen_center @ np.array([right, up]) + distance * back
    pixels = ((np.arange(image_size) + 0.5) / image_size - 0.5) * span
    x, y = np.meshgrid(pixels, pixels[::-1])
    starts = camera + x[..., None] * right + y[..., None] * up
    directions = np.broadcast_to(-back, starts.shape).copy()
    warp_mesh = convert_to_warp_mesh(vertices, np.asarray(mesh.faces), device="cpu")
    hits, _, normals, _ = raycast_mesh(
        torch.from_numpy(starts.astype(np.float32)),
        torch.from_numpy(directions.astype(np.float32)),
        warp_mesh,
        max_dist=2.0 * distance,
        return_normal=True,
    )
    heights = hits.numpy()[..., 2]
    visible = np.isfinite(heights)
    if not visible.any():
        raise RuntimeError("preview rays did not hit the terrain")
    normalizer = height_normalizer(heights[visible])
    rgba = np.empty((*heights.shape, 4))
    rgba[:] = colors.to_rgba(FIGURE_BG)
    surface_colors = HEIGHT_CMAP(normalizer(heights[visible]))
    if view == "3d":
        # Camera-side lighting also illuminates back-facing, single-sided surfaces.
        light = np.array([-0.3, -0.5, 1.0])
        light /= np.linalg.norm(light)
        surface_normals = normals.numpy()[visible]
        surface_normals *= np.where(surface_normals @ back < 0, -1.0, 1.0)[:, None]
        shade = 0.45 + 0.55 * np.clip(surface_normals @ light, 0.0, 1.0)
        surface_colors[:, :3] *= shade[:, None]
    rgba[visible] = surface_colors

    # Crop only empty background; retain the orthographic proportions.
    rows, cols = np.nonzero(visible)
    margin = max(int(image_size * 0.04), 1)
    rgba = rgba[
        max(rows.min() - margin, 0) : rows.max() + margin + 1,
        max(cols.min() - margin, 0) : cols.max() + margin + 1,
    ]
    figure_height = image_size * rgba.shape[0] / rgba.shape[1] + 40
    fig, ax = plt.subplots(figsize=(image_size / 100, figure_height / 100), dpi=100)
    fig.patch.set_facecolor(FIGURE_BG)
    ax.imshow(rgba, interpolation="antialiased")
    ax.set_title(title, fontsize=9)
    ax.set_axis_off()
    add_height_colorbar(fig, ax, normalizer)
    fig.tight_layout(pad=0.1)
    fig.savefig(output_path)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Output files
# ---------------------------------------------------------------------------


def write_index(output_dir: Path, rows: list[dict]):
    """Write a lightweight HTML gallery next to the generated preview files."""
    cards = []
    for row in rows:
        image_rel = html.escape(row["png"].name)
        label = html.escape(row["label"])
        cards.append(
            f'<article><a href="{image_rel}"><img src="{image_rel}" alt="{label}"></a><h2>{label}</h2></article>'
        )
    body = "\n".join(cards)
    (output_dir / "index.html").write_text(
        f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>LocoLab Terrain Previews</title>
  <style>
    body {{ font-family: sans-serif; margin: 24px; background: #f6f7f8; color: #1f2933; }}
    main {{ display: grid; grid-template-columns: repeat(auto-fill, minmax(260px, 1fr)); gap: 16px; }}
    article {{ background: white; border: 1px solid #d8dee4; border-radius: 6px; padding: 10px; }}
    img {{ width: 100%; display: block; background: #eef1f4; }}
    h2 {{ font-size: 14px; margin: 8px 0 4px; }}
    p {{ margin: 0; font-size: 13px; }}
  </style>
</head>
<body>
  <h1>LocoLab Terrain Previews</h1>
  <main>
{body}
  </main>
</body>
</html>
""",
        encoding="utf-8",
    )


def path_relative_to(path: Path, base: Path) -> str:
    """Return a POSIX relative path for Markdown links."""
    return path.resolve().relative_to(base.resolve()).as_posix()


def write_readme_preview_section(readme_path: Path, output_dir: Path, rows: list[dict]):
    """Refresh the generated gallery block in the package README."""
    try:
        rows_rel = [
            {
                "label": row["label"],
                "png": path_relative_to(row["png"], readme_path.parent),
            }
            for row in rows
        ]
    except ValueError:
        print(f"[WARN] Skipping README update because output is outside {readme_path.parent}.")
        return

    lines = [
        README_BEGIN,
        "",
        "This section is generated by `preview/preview_locolab_terrains.py`.",
        "",
        "| Terrain | Preview |",
        "| --- | --- |",
    ]
    for row in rows_rel:
        label = row["label"]
        lines.append(f"| `{label}` | ![{label}]({row['png']}) |")
    lines.extend(["", README_END])
    generated = "\n".join(lines)

    if readme_path.exists():
        content = readme_path.read_text(encoding="utf-8")
    else:
        content = "# LocoLab Terrains\n\n## Preview Gallery\n\n" + README_BEGIN + "\n" + README_END + "\n"

    if README_BEGIN in content and README_END in content:
        start = content.index(README_BEGIN)
        end = content.index(README_END) + len(README_END)
        content = content[:start] + generated + content[end:]
    else:
        content = content.rstrip() + "\n\n## Preview Gallery\n\n" + generated + "\n"
    readme_path.write_text(content.rstrip() + "\n", encoding="utf-8")
    print(f"[OK] Updated README previews: {readme_path}")


def parse_args() -> argparse.Namespace:
    """Parse preview generation options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out",
        type=Path,
        default=DEFAULT_OUT_DIR,
        help="Output directory for PNG/index files. Defaults to this preview folder's terrain_previews directory.",
    )
    parser.add_argument("--difficulties", type=float, nargs="+", default=[0.5])
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--size", type=float, nargs=2, default=[8.0, 8.0], metavar=("X", "Y"))
    parser.add_argument("--horizontal-scale", type=float, default=0.1)
    parser.add_argument("--vertical-scale", type=float, default=0.01)
    parser.add_argument("--slope-threshold", type=float, default=0.75)
    parser.add_argument("--image-size", type=int, default=640)
    parser.add_argument(
        "--view",
        choices=("top", "3d"),
        default="3d",
        help="Camera view for every terrain, including height fields (default: 3d).",
    )
    parser.add_argument(
        "--no-readme-update",
        action="store_true",
        help="Do not refresh the README preview gallery.",
    )
    return parser.parse_args()


def main():
    """Generate meshes, images, and README links for all configured preview cases."""
    args = parse_args()
    output_dir = args.out.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    pkg = import_locolab_terrains()
    cfgs = discover_cfgs(pkg)
    missing_cases = sorted(set(cfgs) - set(PREVIEW_CASES))
    stale_cases = sorted(set(PREVIEW_CASES) - set(cfgs))
    if missing_cases:
        raise RuntimeError(f"Missing preview parameters for: {', '.join(missing_cases)}")
    if stale_cases:
        raise RuntimeError(f"Preview parameters reference missing terrain cfgs: {', '.join(stale_cases)}")

    common_kwargs = {
        "size": tuple(args.size),
    }
    hf_kwargs = {
        "horizontal_scale": args.horizontal_scale,
        "vertical_scale": args.vertical_scale,
        "slope_threshold": args.slope_threshold,
    }
    rows: list[dict] = []
    failures: list[str] = []

    for case_index, (cfg_name, cfg_cls) in enumerate(cfgs.items()):
        for difficulty_index, difficulty in enumerate(args.difficulties):
            label = cfg_name if len(args.difficulties) == 1 else f"{cfg_name} d={difficulty:g}"
            stem = cfg_name if len(args.difficulties) == 1 else f"{cfg_name}_d{difficulty:g}".replace(".", "p")
            png_path = output_dir / f"{stem}.png"
            try:
                np.random.seed(args.seed + case_index * 100 + difficulty_index)
                cfg = build_cfg(cfg_cls, common_kwargs, hf_kwargs, PREVIEW_CASES[cfg_name])
                meshes, origin = cfg.function(float(difficulty), cfg)
                mesh = trimesh.util.concatenate(meshes)
                assert_mesh(mesh, origin)
                render_mesh(
                    mesh,
                    png_path,
                    label,
                    args.image_size,
                    view=args.view,
                )
                rows.append({"label": label, "png": png_path})
                print(f"[OK] {label}: vertices={len(mesh.vertices)} faces={len(mesh.faces)} origin={origin.tolist()}")
            except Exception as exc:
                failures.append(f"{label}: {exc}")
                print(f"[FAIL] {label}: {exc}")

    if failures:
        raise RuntimeError("Preview generation failed:\n" + "\n".join(failures))

    write_index(output_dir, rows)
    if not args.no_readme_update:
        write_readme_preview_section(README_PATH, output_dir, rows)

    print(f"[DONE] output={output_dir}")


if __name__ == "__main__":
    main()
