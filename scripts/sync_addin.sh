#!/usr/bin/env bash
# Vendor the fusion2urdf core package into the Fusion 360 add-in folder so the
# script is self-contained inside Fusion's embedded Python (no pip needed).
# Non-destructive: overwrites tracked .py files in place, deletes nothing.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$REPO_DIR/src/fusion2urdf"
DEST="$REPO_DIR/addin/URDF_Exporter_Plus/fusion2urdf"

mkdir -p "$DEST"
cp -f "$SRC"/*.py "$DEST/"
echo "Vendored fusion2urdf core -> $DEST"
