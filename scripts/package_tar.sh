#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." >/dev/null 2>&1 && pwd)"
WEB_RVIZ_DIR="$ROOT_DIR/web_rviz"
OUT_DIR="$ROOT_DIR/dist-packages"

cd "$WEB_RVIZ_DIR"
npm ci
npm run build

VERSION="$(node -p "require('./package.json').version")"
PKG_NAME="webrviz-$VERSION"
STAGING_DIR="$OUT_DIR/$PKG_NAME"
ARCHIVE_PATH="$OUT_DIR/$PKG_NAME.tar.gz"

rm -rf "$STAGING_DIR"
mkdir -p "$STAGING_DIR/tools"

cp -R "$WEB_RVIZ_DIR/dist" "$STAGING_DIR/dist"
cp "$WEB_RVIZ_DIR/tools/serve_webrviz.py" "$STAGING_DIR/tools/serve_webrviz.py"
cp "$WEB_RVIZ_DIR/start.sh" "$STAGING_DIR/start.sh"
cp "$ROOT_DIR/README.md" "$STAGING_DIR/README.md"
cp "$ROOT_DIR/LICENSE" "$STAGING_DIR/LICENSE"

chmod +x "$STAGING_DIR/start.sh" "$STAGING_DIR/tools/serve_webrviz.py"

mkdir -p "$OUT_DIR"
tar -C "$OUT_DIR" -czf "$ARCHIVE_PATH" "$PKG_NAME"

echo "Created: $ARCHIVE_PATH"