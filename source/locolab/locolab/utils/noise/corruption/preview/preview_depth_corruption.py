# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Generate offline previews for depth-corruption visual patterns.

The script builds one synthetic normalized depth image, paints each mode with
``apply_selected_depth_corruption_modes``, and writes PNG strips plus an
HTML gallery. It does not touch training configs. Isaac Sim is not required;
torch, numpy, and matplotlib are enough.

Run from anywhere:

    python preview_depth_corruption.py
"""

from __future__ import annotations

import argparse
import html
import os
import sys
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-locolab-preview")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
from preview_depth_corruption_cfg import (
    DEFAULT_HEIGHT,
    DEFAULT_SAMPLES,
    DEFAULT_SEED,
    DEFAULT_WIDTH,
    DEPTH_CMAP,
    FIGURE_BG,
    MODE_BLURBS,
)

# Paths are fixed relative to this preview folder, so the script can be run
# from this directory or from anywhere else.
PREVIEW_DIR = Path(__file__).resolve().parent
NOISE_DIR = PREVIEW_DIR.parent.parent
README_PATH = PREVIEW_DIR.parent / "README.md"
DEFAULT_OUT_DIR = PREVIEW_DIR / "pattern_previews"
README_BEGIN = "<!-- BEGIN_AUTO_PREVIEWS -->"
README_END = "<!-- END_AUTO_PREVIEWS -->"

if str(PREVIEW_DIR) not in sys.path:
    sys.path.insert(0, str(PREVIEW_DIR))
if str(NOISE_DIR) not in sys.path:
    sys.path.insert(0, str(NOISE_DIR))

from corruption import (  # noqa: E402
    DEPTH_CORRUPTION_MODE_NAMES,
    DepthCorruptionPatternCfg,
    apply_selected_depth_corruption_modes,
)


def make_reference_depth(height: int, width: int) -> np.ndarray:
    """Build a deterministic forward-camera depth image in ``[0, 1]``.

    The upper band is a far wall. Below the horizon the ground gets closer
    toward the bottom of the image. A few boxes and a pole give the corruption
    patterns something structured to cover.
    """
    rows = np.linspace(0.0, 1.0, height, dtype=np.float32)[:, None]
    cols = np.linspace(0.0, 1.0, width, dtype=np.float32)[None, :]
    horizon = 0.30
    wall = 0.74 + 0.10 * cols
    ground_t = np.clip((rows - horizon) / (1.0 - horizon), 0.0, 1.0)
    ground = 0.16 + 0.60 * (1.0 - ground_t) ** 1.35
    ground = ground * (0.94 + 0.06 * cols)
    depth = np.broadcast_to(np.where(rows < horizon, wall, ground), (height, width)).copy()

    def blit(row0: float, row1: float, col0: float, col1: float, value: float) -> None:
        top = int(round(row0 * (height - 1)))
        bottom = int(round(row1 * (height - 1)))
        left = int(round(col0 * (width - 1)))
        right = int(round(col1 * (width - 1)))
        if bottom <= top or right <= left:
            return
        depth[top:bottom, left:right] = value

    blit(0.55, 0.86, 0.10, 0.32, 0.28)
    blit(0.42, 0.70, 0.62, 0.86, 0.46)
    blit(0.18, 0.80, 0.46, 0.53, 0.58)
    return np.clip(depth, 0.0, 1.0)


def paint_mode(
    clean: torch.Tensor,
    mode_index: int,
    count: int,
    seed: int,
    cfg: DepthCorruptionPatternCfg,
) -> torch.Tensor:
    """Paint ``count`` independent samples of one mode. ``clean`` is not modified."""
    torch.manual_seed(seed)
    batch = clean.unsqueeze(0).repeat(count, 1, 1, 1)
    modes = torch.full((count,), mode_index, dtype=torch.long)
    painted = apply_selected_depth_corruption_modes(batch, cfg, modes)
    if DEPTH_CORRUPTION_MODE_NAMES[mode_index] != "full_frame_mask":
        return painted

    # A short random draw can land on only one of the two replacement values.
    # Keep sampling until the strip shows both 0 and 1.
    seen = {_constant_value(frame) for frame in painted}
    attempt = 0
    while seen != {0, 1} and attempt < 32:
        torch.manual_seed(seed + 1000 + attempt)
        extra = apply_selected_depth_corruption_modes(clean.unsqueeze(0), cfg, [mode_index])[0]
        value = _constant_value(extra)
        if value not in seen:
            painted = painted.clone()
            painted[-1] = extra
            seen.add(value)
        attempt += 1
    return painted


def _constant_value(frame: torch.Tensor) -> int:
    return int(round(float(frame.reshape(-1)[0].item())))


def _as_image(frame: torch.Tensor) -> np.ndarray:
    array = frame.detach().cpu().numpy()
    if array.ndim == 3:
        array = array[..., 0]
    return array


def _sample_label(index: int, image: np.ndarray) -> str:
    if float(image.max() - image.min()) < 1e-5:
        return f"sample {index}\nvalue {float(image.mean()):.0f}"
    return f"sample {index}"


def _style_depth_axis(axis, image: np.ndarray, label: str):
    artist = axis.imshow(image, cmap=DEPTH_CMAP, vmin=0.0, vmax=1.0, interpolation="nearest", aspect="equal")
    axis.set_title(label, fontsize=10, color="#1f2933", pad=8)
    axis.set_xticks([])
    axis.set_yticks([])
    for spine in axis.spines.values():
        spine.set_color("#c8c2b6")
    return artist


def render_strip(
    clean: np.ndarray,
    samples: list[np.ndarray],
    title: str,
    blurb: str,
    output_path: Path,
) -> None:
    """Write one PNG: the clean frame followed by independent samples of one mode."""
    panels = [clean, *samples]
    labels = ["clean"] + [_sample_label(index, sample) for index, sample in enumerate(samples, start=1)]
    fig = plt.figure(figsize=(2.35 * len(panels) + 0.7, 3.5), dpi=140, facecolor=FIGURE_BG)
    grid = fig.add_gridspec(
        1,
        len(panels) + 1,
        width_ratios=[1.0] * len(panels) + [0.06],
        left=0.02,
        right=0.98,
        top=0.78,
        bottom=0.16,
        wspace=0.18,
    )
    artist = None
    for index, (panel, label) in enumerate(zip(panels, labels)):
        artist = _style_depth_axis(fig.add_subplot(grid[0, index]), panel, label)
    colorbar = fig.colorbar(artist, cax=fig.add_subplot(grid[0, -1]))
    colorbar.set_label("normalized depth", labelpad=8)
    fig.suptitle(title, fontsize=14, color="#1f2933", y=0.94)
    fig.text(0.5, 0.035, blurb, ha="center", va="center", fontsize=10, color="#3d4a57")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, facecolor=FIGURE_BG)
    plt.close(fig)


def render_overview(
    clean: np.ndarray,
    samples_by_mode: list[tuple[str, np.ndarray]],
    output_path: Path,
) -> None:
    """Write a single sheet with the clean frame and one sample of every mode."""
    panels = [("clean", clean), *samples_by_mode]
    columns = 3 if len(panels) <= 6 else 4
    rows = int(np.ceil(len(panels) / columns))
    fig = plt.figure(figsize=(3.3 * columns + 0.7, 3.15 * rows), dpi=140, facecolor=FIGURE_BG)
    grid = fig.add_gridspec(
        rows,
        columns + 1,
        width_ratios=[1.0] * columns + [0.05],
        left=0.03,
        right=0.97,
        top=0.88,
        bottom=0.04,
        wspace=0.16,
        hspace=0.32,
    )
    artist = None
    for index, (label, image) in enumerate(panels):
        row, column = divmod(index, columns)
        artist = _style_depth_axis(fig.add_subplot(grid[row, column]), image, label)
    colorbar = fig.colorbar(artist, cax=fig.add_subplot(grid[:, -1]))
    colorbar.set_label("normalized depth", labelpad=8)
    fig.suptitle("Depth corruption patterns", fontsize=15, color="#1f2933", y=0.97)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, facecolor=FIGURE_BG)
    plt.close(fig)


def write_index(output_dir: Path, rows: list[dict], overview_name: str) -> None:
    """Write a lightweight HTML gallery next to the generated preview files."""
    cards = [
        (
            "<article class='overview'>"
            f"<a href='{html.escape(overview_name)}'><img src='{html.escape(overview_name)}' alt='overview'></a>"
            "<h2>overview</h2>"
            "<p>Clean synthetic depth, then one sample of each pattern.</p>"
            "</article>"
        )
    ]
    for row in rows:
        image_rel = html.escape(row["png"].name)
        label = html.escape(row["label"])
        blurb = html.escape(row["blurb"])
        cards.append(
            f"<article><a href='{image_rel}'><img src='{image_rel}' alt='{label}'></a>"
            f"<h2>{label}</h2><p>{blurb}</p></article>"
        )
    body = "\n".join(cards)
    (output_dir / "index.html").write_text(
        f"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Depth Corruption Pattern Previews</title>
  <style>
    body {{ font-family: sans-serif; margin: 24px; background: #f6f7f8; color: #1f2933; }}
    main {{ display: flex; flex-direction: column; gap: 16px; max-width: 1100px; }}
    article {{ background: white; border: 1px solid #d8dee4; border-radius: 6px; padding: 10px; }}
    img {{ width: 100%; display: block; background: #f7f5ef; }}
    h2 {{ font-size: 16px; margin: 10px 0 4px; }}
    p {{ margin: 0; font-size: 14px; }}
  </style>
</head>
<body>
  <h1>Depth Corruption Pattern Previews</h1>
  <p>Each strip starts from the same synthetic depth image. Color only displays normalized depth in [0, 1].</p>
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


def write_readme_preview_section(readme_path: Path, output_dir: Path, rows: list[dict], overview_path: Path) -> None:
    """Refresh the generated gallery block in the package README."""
    try:
        overview_rel = path_relative_to(overview_path, readme_path.parent)
        rows_rel = [
            {
                "label": row["label"],
                "blurb": row["blurb"],
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
        "This section is generated by `preview/preview_depth_corruption.py`.",
        "",
        f"![overview]({overview_rel})",
        "",
        "| Pattern | What it does | Preview |",
        "| --- | --- | --- |",
    ]
    for row in rows_rel:
        label = row["label"]
        lines.append(f"| `{label}` | {row['blurb']} | ![{label}]({row['png']}) |")
    lines.extend(["", README_END])
    generated = "\n".join(lines)

    if readme_path.exists():
        content = readme_path.read_text(encoding="utf-8")
    else:
        content = "# Depth Corruption\n\n## Preview Gallery\n\n" + README_BEGIN + "\n" + README_END + "\n"

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
        help="Output directory for PNG/index files. Defaults to this preview folder's pattern_previews directory.",
    )
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT, help="Synthetic depth image height.")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH, help="Synthetic depth image width.")
    parser.add_argument("--samples", type=int, default=DEFAULT_SAMPLES, help="Independent samples drawn per pattern.")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument(
        "--modes",
        nargs="+",
        default=list(DEPTH_CORRUPTION_MODE_NAMES),
        help="Pattern names to render. Defaults to every mode, in index order.",
    )
    parser.add_argument(
        "--no-readme-update",
        action="store_true",
        help="Do not refresh the README preview gallery.",
    )
    return parser.parse_args()


def _check_blurbs() -> None:
    missing = [name for name in DEPTH_CORRUPTION_MODE_NAMES if name not in MODE_BLURBS]
    stale = [name for name in MODE_BLURBS if name not in DEPTH_CORRUPTION_MODE_NAMES]
    if missing:
        raise RuntimeError(f"Missing preview blurbs for: {', '.join(missing)}")
    if stale:
        raise RuntimeError(f"Preview blurbs reference unknown modes: {', '.join(stale)}")


def main() -> None:
    """Generate pattern strips, an overview sheet, and the README gallery."""
    args = parse_args()
    if args.height < 2 or args.width < 2:
        raise RuntimeError(f"Image size must be at least 2x2, got {args.height}x{args.width}.")
    if args.samples < 1:
        raise RuntimeError("--samples must be at least 1.")
    _check_blurbs()

    unknown = [name for name in args.modes if name not in DEPTH_CORRUPTION_MODE_NAMES]
    if unknown:
        known = ", ".join(DEPTH_CORRUPTION_MODE_NAMES)
        raise RuntimeError(f"Unknown modes: {', '.join(unknown)}. Known modes: {known}.")

    output_dir = args.out.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    cfg = DepthCorruptionPatternCfg()
    clean = torch.as_tensor(make_reference_depth(args.height, args.width), dtype=torch.float32).unsqueeze(-1)

    selected = [name for name in DEPTH_CORRUPTION_MODE_NAMES if name in set(args.modes)]
    rows: list[dict] = []
    overview_samples: list[tuple[str, np.ndarray]] = []
    for mode_index, mode_name in enumerate(DEPTH_CORRUPTION_MODE_NAMES):
        if mode_name not in selected:
            continue
        painted = paint_mode(clean, mode_index, args.samples, args.seed + mode_index * 100, cfg)
        samples = [_as_image(frame) for frame in painted]
        png_path = output_dir / f"{mode_name}.png"
        title = f"{mode_index}  {mode_name}"
        render_strip(_as_image(clean), samples, title, MODE_BLURBS[mode_name], png_path)
        rows.append({"label": mode_name, "blurb": MODE_BLURBS[mode_name], "png": png_path})
        overview_samples.append((title, samples[0]))
        print(f"[OK] {title}: samples={len(samples)} -> {png_path.name}")

    overview_path = output_dir / "overview.png"
    render_overview(_as_image(clean), overview_samples, overview_path)
    write_index(output_dir, rows, overview_path.name)
    if not args.no_readme_update and selected == list(DEPTH_CORRUPTION_MODE_NAMES):
        write_readme_preview_section(README_PATH, output_dir, rows, overview_path)
    elif not args.no_readme_update:
        print("[WARN] Skipping README update because --modes does not include every pattern.")

    print(f"[DONE] output={output_dir}")
    print(f"[DONE] open {output_dir / 'index.html'}")


if __name__ == "__main__":
    main()
