#!/usr/bin/env bash
# Install LocoLab into an activated Python 3.12 virtual environment.
# Stop on command failures, unset variables, or failures within pipelines.
set -euo pipefail

# Require both local checkout paths, followed by an optional full-stack installation flag.
if [[ $# -lt 2 || $# -gt 3 || ( $# -eq 3 && "$3" != "--all" ) ]]; then
    echo "Usage: bash install.sh /path/to/IsaacLab /path/to/z_rl [--all]" >&2
    exit 1
fi

# Require an activated environment so installation targets the user's chosen environment.
if [[ -z "${VIRTUAL_ENV:-}" ]]; then
    echo "Activate a Python 3.12 virtual environment before installing." >&2
    exit 1
fi

# Check that the supplied path contains the Isaac Lab workspace and core source package.
if [[ ! -f "$1/pyproject.toml" || ! -d "$1/source/isaaclab" ]]; then
    echo "Expected an Isaac Lab 3.0 source checkout: $1" >&2
    exit 1
fi

# Check that the supplied Z-RL path contains its package metadata and source package.
if [[ ! -f "$2/pyproject.toml" || ! -d "$2/z_rl" ]]; then
    echo "Expected a Z-RL source checkout: $2" >&2
    exit 1
fi

# Check Python compatibility before uv sync can change the environment.
"$VIRTUAL_ENV/bin/python" -c '
import sys
if sys.version_info[:2] != (3, 12):
    sys.exit("Isaac Lab 3.0 requires a Python 3.12 environment.")
'

# Locate LocoLab relative to this script, independent of the current working directory.
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# Add required OV PhysX and Viser dependencies; Newton MJWarp and GL are in the base install.
extras=(--extra ovphysx --extra viser)

# With --all, also install Newton RTX and Isaac Sim / Kit dependencies.
if [[ "${3:-}" == "--all" ]]; then
    extras+=(--extra ovrtx --extra isaacsim)
fi

# Sync Isaac Lab into the active environment while retaining unrelated installed packages.
uv sync --project "$1" --active --inexact "${extras[@]}"

# Install the supplied Z-RL checkout, LocoLab, and its extension in editable mode with dependencies.
uv pip install --python "$VIRTUAL_ENV/bin/python" \
    -e "$2" \
    -e "$repo_root/source/locolab" \
    -e "$repo_root/source/z_rl_loco_lab"
