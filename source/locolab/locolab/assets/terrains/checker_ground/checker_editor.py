#!/usr/bin/env python3
"""Launch the local checker texture editor (Python standard library only)."""

import errno
from datetime import datetime
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import shutil
from tempfile import NamedTemporaryFile
from threading import Lock


TEXTURE = Path(__file__).resolve().parent / "Materials/Textures/default_ground_plane_albedo.png"
SAVE_LOCK = Lock()
PORT = 6006


class EditorHandler(SimpleHTTPRequestHandler):
    def send_json(self, status, payload):
        data = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/texture.png":
            data = TEXTURE.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)
        else:
            super().do_GET()

    def do_POST(self):
        if self.path != "/save-texture":
            self.send_error(404)
            return
        if self.headers.get("Content-Type") != "image/png":
            self.send_json(415, {"error": "Expected image/png."})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self.send_json(400, {"error": "Invalid content length."})
            return
        if not 0 < length <= 5 * 1024 * 1024:
            self.send_json(413, {"error": "PNG must be smaller than 5 MB."})
            return
        data = self.rfile.read(length)
        if (len(data) != length or not data.startswith(b"\x89PNG\r\n\x1a\n")
                or data[12:16] != b"IHDR" or not data.endswith(b"\x00\x00\x00\x00IEND\xaeB`\x82")):
            self.send_json(400, {"error": "Invalid PNG."})
            return
        temporary = None
        try:
            with SAVE_LOCK:
                if data[16:24] != TEXTURE.read_bytes()[16:24]:
                    self.send_json(400, {"error": "PNG dimensions must match the original texture."})
                    return
                backup_dir = TEXTURE.parent / "backups"
                backup_dir.mkdir(exist_ok=True)
                backup = backup_dir / f"{TEXTURE.stem}.{datetime.now():%Y%m%d-%H%M%S-%f}.png"
                with NamedTemporaryFile(dir=TEXTURE.parent, prefix=".checker-", suffix=".png", delete=False) as output:
                    temporary = Path(output.name)
                    output.write(data)
                temporary.chmod(TEXTURE.stat().st_mode)
                shutil.copy2(TEXTURE, backup)
                temporary.replace(TEXTURE)
            self.send_json(200, {"backup": str(backup.relative_to(TEXTURE.parent))})
        except OSError as error:
            self.send_json(500, {"error": str(error)})
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)


def main():
    handler = partial(EditorHandler, directory=str(Path(__file__).with_suffix("")))
    try:
        server = ThreadingHTTPServer(("127.0.0.1", PORT), handler)
    except OSError as error:
        if error.errno != errno.EADDRINUSE:
            raise
        print(f"Port {PORT} is in use; choosing an available port.", flush=True)
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    with server:
        print(f"Checker editor: http://127.0.0.1:{server.server_port}\nPress Ctrl+C to stop.", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
