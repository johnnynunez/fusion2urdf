#!/usr/bin/env bash
# Vendor the fusion2urdf core package into the Fusion 360 add-in folder and
# the GitHub Pages web app so both stay in sync with src/.
# Non-destructive: overwrites tracked .py files in place, deletes nothing.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$REPO_DIR/src/fusion2urdf"

for DEST in "$REPO_DIR/addin/URDF_Exporter_Plus/fusion2urdf" \
            "$REPO_DIR/docs/pkg/fusion2urdf"; do
  mkdir -p "$DEST"
  cp -f "$SRC"/*.py "$DEST/"
  echo "Vendored fusion2urdf core -> $DEST"
done
