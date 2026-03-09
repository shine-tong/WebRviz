#!/usr/bin/env python3
import argparse
import posixpath
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from typing import Dict
from urllib.parse import unquote, urlparse


def parse_mount(values):
    mounts = {}
    for item in values:
        if "=" not in item:
            raise ValueError(f"invalid --mount value: {item}, expected <package>=<path>")
        package, raw_path = item.split("=", 1)
        package = package.strip().strip("/")
        if not package:
            raise ValueError(f"invalid package name in mount: {item}")

        directory = Path(raw_path).expanduser().resolve()
        if not directory.exists() or not directory.is_dir():
            raise ValueError(f"mount path does not exist or is not a directory: {directory}")

        mounts[package] = directory
    return mounts


class WebRvizHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, dist_dir: Path, package_mounts: Dict[str, Path], **kwargs):
        self.dist_dir = dist_dir
        self.package_mounts = package_mounts
        super().__init__(*args, directory=str(dist_dir), **kwargs)

    def translate_path(self, path: str) -> str:
        parsed = urlparse(path)
        request_path = unquote(parsed.path)

        pkg_prefix = "/ros_pkgs/"
        if request_path.startswith(pkg_prefix):
            rel = request_path[len(pkg_prefix):]
            rel_parts = [p for p in rel.split("/") if p]
            if not rel_parts:
                return str(self.dist_dir / "__missing__")

            package = rel_parts[0]
            remainder = rel_parts[1:]
            mount_dir = self.package_mounts.get(package)
            if mount_dir is None:
                return str(self.dist_dir / "__missing__")

            candidate = mount_dir
            for part in remainder:
                if part in (".", ".."):
                    return str(self.dist_dir / "__missing__")
                candidate = candidate / part

            try:
                candidate.resolve().relative_to(mount_dir)
            except Exception:
                return str(self.dist_dir / "__missing__")

            return str(candidate)

        safe_path = posixpath.normpath(request_path)
        words = [w for w in safe_path.split("/") if w and w not in (".", "..")]
        result = self.dist_dir
        for word in words:
            result = result / word
        return str(result)


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve web_rviz dist and ROS package assets.")
    parser.add_argument("--host", default="0.0.0.0", help="bind host (default: 0.0.0.0)")
    parser.add_argument("--port", default=8080, type=int, help="bind port (default: 8080)")
    parser.add_argument("--dist", default="dist", help="dist directory path (default: dist)")
    parser.add_argument(
        "--mount",
        action="append",
        default=[],
        metavar="PACKAGE=PATH",
        help="mount ROS package directory under /ros_pkgs/<PACKAGE>/",
    )

    args = parser.parse_args()

    dist_dir = Path(args.dist).expanduser().resolve()
    if not dist_dir.exists() or not dist_dir.is_dir():
        raise SystemExit(f"dist directory does not exist: {dist_dir}")

    try:
        mounts = parse_mount(args.mount)
    except ValueError as exc:
        raise SystemExit(str(exc))

    print("WebRviz server configuration:")
    print(f"  dist   : {dist_dir}")
    print(f"  url    : http://{args.host}:{args.port}")
    if mounts:
        print("  mounts :")
        for pkg, path in mounts.items():
            print(f"    /ros_pkgs/{pkg}/ -> {path}")
    else:
        print("  mounts : (none)")

    handler = lambda *h_args, **h_kwargs: WebRvizHandler(*h_args, dist_dir=dist_dir, package_mounts=mounts, **h_kwargs)

    server = HTTPServer((args.host, args.port), handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
