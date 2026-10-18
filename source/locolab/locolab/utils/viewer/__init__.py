# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Play-script entry points for the optional Viser viewer.

Importing this module does not import Viser or Isaac Sim. The viewer module
loads only when ``--viser`` is actually set.
"""

from .play import add_play_viewer_args, open_play_viewer, prepare_play_viewer

__all__ = ["add_play_viewer_args", "open_play_viewer", "prepare_play_viewer"]
