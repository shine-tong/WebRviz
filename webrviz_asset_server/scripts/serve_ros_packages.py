#!/usr/bin/env python3
import argparse
import mimetypes
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, Optional
from urllib.parse import unquote, urlparse

import rospkg


class PackageResolver:
    def __init__(self) -> None:
        self._rospack = rospkg.RosPack()
        self._cache: Dict[str, Path] = {}

    def resolve(self, package_name: str) -> Optional[Path]:
        cached = self._cache.get(package_name)
        if cached is not None:
            return cached

        try:
            path = Path(self._rospack.get_path(package_name)).resolve()
        except rospkg.ResourceNotFound:
            return None

        self._cache[package_name] = path
        return path


class RosPackageAssetHandler(BaseHTTPRequestHandler):
    resolver = PackageResolver()

    def do_OPTIONS(self) -> None:  # noqa: N802
        self.send_response(204)
        self._send_cors_headers()
        self.end_headers()

    def do_HEAD(self) -> None:  # noqa: N802
        self._serve_request(send_body=False)

    def do_GET(self) -> None:  # noqa: N802
        self._serve_request(send_body=True)

    def log_message(self, format: str, *args) -> None:
        print("%s - - [%s] %s" % (self.address_string(), self.log_date_time_string(), format % args))

    def _serve_request(self, send_body: bool) -> None:
        parsed = urlparse(self.path)
        request_path = unquote(parsed.path)

        if request_path == "/healthz":
            payload = b"ok\n"
            self.send_response(200)
            self._send_cors_headers()
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            if send_body:
                self.wfile.write(payload)
            return

        file_path = self._resolve_request_path(request_path)
        if file_path is None:
            self._send_error(404, "asset not found")
            return

        if not file_path.exists() or not file_path.is_file():
            self._send_error(404, "asset not found")
            return

        try:
            payload = file_path.read_bytes()
        except OSError:
            self._send_error(500, "failed to read asset")
            return

        content_type, _ = mimetypes.guess_type(str(file_path))
        self.send_response(200)
        self._send_cors_headers()
        self.send_header("Content-Type", content_type or "application/octet-stream")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        if send_body:
            self.wfile.write(payload)

    def _resolve_request_path(self, request_path: str) -> Optional[Path]:
        prefix = "/ros_pkgs/"
        if not request_path.startswith(prefix):
            return None

        relative = request_path[len(prefix):]
        parts = [part for part in relative.split("/") if part]
        if len(parts) < 2:
            return None

        package_name = parts[0]
        package_dir = self.resolver.resolve(package_name)
        if package_dir is None:
            return None

        candidate = package_dir.joinpath(*parts[1:]).resolve()
        try:
            candidate.relative_to(package_dir)
        except ValueError:
            return None
        return candidate

    def _send_cors_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, HEAD, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _send_error(self, code: int, message: str) -> None:
        payload = (message + "\n").encode("utf-8")
        self.send_response(code)
        self._send_cors_headers()
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve ROS package assets for WebRviz.")
    parser.add_argument("--host", default="0.0.0.0", help="bind host (default: 0.0.0.0)")
    parser.add_argument("--port", default=8081, type=int, help="bind port (default: 8081)")
    args, unknown_args = parser.parse_known_args(sys.argv[1:])

    server = ThreadingHTTPServer((args.host, args.port), RosPackageAssetHandler)
    print("WebRviz ROS asset server configuration:")
    print(f"  url    : http://{args.host}:{args.port}")
    print("  route  : /ros_pkgs/<package>/<path>")
    if unknown_args:
        print(f"  ignored launch args: {' '.join(unknown_args)}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
