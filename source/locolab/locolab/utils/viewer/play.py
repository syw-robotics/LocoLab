# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""The small surface play scripts call. The viewer itself stays in ``viser_play``."""

from __future__ import annotations

import argparse


class _IdlePlayViewer:
    """Stand-in used when ``--viser`` was not passed. Every method is a no-op."""

    paused = False
    paces = False

    def apply_reset(self, env, reset_policy) -> bool:
        return False

    def after_step(self) -> bool:
        return False

    def close(self) -> None:
        return None


def add_play_viewer_args(parser: argparse.ArgumentParser) -> None:
    """Register ``--viser``. The viewer stays off unless the flag is passed."""
    parser.add_argument(
        "--viser",
        action="store_true",
        default=False,
        help="Serve a Viser robot view for remote play. Off unless this flag is set.",
    )
    parser.add_argument("--viser-port", type=int, default=8080, help="Viser port. Used only with --viser.")
    parser.add_argument(
        "--viser-host",
        type=str,
        default="0.0.0.0",
        help="Viser bind address. Used only with --viser. 0.0.0.0 prints http://localhost.",
    )


def prepare_play_viewer(args, env_cfg) -> None:
    """Keep a CPU copy of the terrain mesh when the viewer will draw it.

    Call this after the play config is finalized and before ``gym.make``.
    """
    if not getattr(args, "viser", False):
        return
    from .terrain import retain_terrain_mesh_for_viewer

    retain_terrain_mesh_for_viewer(env_cfg)


def open_play_viewer(env, args):
    """Open the viewer, or return an idle object with the same methods."""
    if not getattr(args, "viser", False):
        return _IdlePlayViewer()
    from .viser_play import ViserPlayViewer

    return ViserPlayViewer(env, host=args.viser_host, port=args.viser_port)
