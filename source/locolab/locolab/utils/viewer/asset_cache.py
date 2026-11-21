# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

"""Serve the current scene's GLBs from the viewer's HTTP origin."""

import gzip
import hashlib
from http import HTTPStatus
from secrets import token_hex
from threading import Lock

from websockets.datastructures import Headers
from websockets.http11 import Request, Response


class SceneAssetCache:
    """Same-process asset URLs with browser caching across connections."""

    def __init__(self) -> None:
        self._prefix = f"/_locolab/assets/{token_hex(8)}/"
        self._assets: dict[str, tuple[str, bytes]] = {}
        self._lock = Lock()

    def mesh_reference(self, name: str, mesh) -> bytes:
        glb = mesh.export(file_type="glb")
        path = f"{self._prefix}{hashlib.sha256(glb).hexdigest()}.glb"
        compressed = gzip.compress(glb, compresslevel=3, mtime=0)
        with self._lock:
            # A terrain appearance change replaces the previous version of that node.
            self._assets[name] = (path, compressed)
        return f"LocoLabGLB1:{path.lstrip('/')}".encode("ascii")

    def respond(self, request: Request) -> Response | None:
        if not request.path.startswith("/_locolab/assets/"):
            return None
        with self._lock:
            body = next((data for path, data in self._assets.values() if path == request.path), None)
        if body is None:
            return Response(HTTPStatus.NOT_FOUND, "Not Found", Headers({"Cache-Control": "no-store"}))
        headers = Headers(
            {
                "Content-Type": "model/gltf-binary",
                "Cache-Control": "private, max-age=86400, immutable",
                "Vary": "Accept-Encoding",
            }
        )
        if "gzip" in request.headers.get("Accept-Encoding", ""):
            headers["Content-Encoding"] = "gzip"
        else:
            body = gzip.decompress(body)
        headers["Content-Length"] = str(len(body))
        return Response(HTTPStatus.OK, "OK", headers, body)
