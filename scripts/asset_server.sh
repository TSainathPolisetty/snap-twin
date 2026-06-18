#!/bin/bash
# snap/local/asset_server.sh
# Serves final_twin.urdf and assets/*.stl from $SNAP_COMMON on port 8080.
# $SNAP_COMMON is writable and holds the IP-patched URDF copy.
# If the patched URDF isn't there yet (demo not started), fall back to $SNAP.
set -e

SERVE_DIR="$SNAP_COMMON"
if [ ! -f "$SERVE_DIR/final_twin.urdf" ]; then
    echo "Patched URDF not found in SNAP_COMMON, falling back to SNAP."
    SERVE_DIR="$SNAP"
fi

cd "$SERVE_DIR"
exec python3 "$SNAP/bin/simple_cors_server.py" --port 8080
