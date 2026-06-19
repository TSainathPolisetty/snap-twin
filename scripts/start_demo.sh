#!/bin/bash
# snap/local/start_demo.sh
# Snap-aware entry point for the main demo.
# Runs as the snap-twin daemon; replaces start_gesture_demo.sh.
set -e

ENGINE="$SNAP/models/depth_anything_v2_small.engine"
URDF_SRC="$SNAP/final_twin.urdf"
URDF_DEST="$SNAP_COMMON/final_twin.urdf"

# ── Verify TRT engine exists (bundled in snap at build time) ──────────────────
if [ ! -f "$ENGINE" ]; then
    echo "ERROR: TRT engine not found at $ENGINE"
    echo "The engine is built into the snap at snapcraft build time."
    echo "This likely means the build failed during the onnx-model part."
    echo "Rebuild with: sudo snapcraft pack --destructive-mode"
    exit 1
fi

# ── Patch URDF with current machine IP (copy to writable $SNAP_COMMON) ────────
MACHINE_IP=$(hostname -I | awk '{print $1}')
if [ ! -f "$URDF_DEST" ] || ! grep -q "$MACHINE_IP" "$URDF_DEST"; then
    echo "Patching URDF with IP $MACHINE_IP → $URDF_DEST"
    mkdir -p "$(dirname "$URDF_DEST")"
    cp "$URDF_SRC" "$URDF_DEST"
    sed -i "s|http://[0-9.]*:8080|http://${MACHINE_IP}:8080|g" "$URDF_DEST"
fi

# ── Copy STL assets alongside patched URDF if not already there ───────────────
if [ ! -d "$SNAP_COMMON/assets" ]; then
    echo "Copying mesh assets to $SNAP_COMMON/assets/"
    cp -r "$SNAP/assets" "$SNAP_COMMON/assets"
fi

if [ ! -d "$SNAP_COMMON/calibration" ]; then
    echo "Copying calibration data to $SNAP_COMMON/calibration/"
    cp -r "$SNAP/calibration" "$SNAP_COMMON/calibration"
fi

# ── Restart foxglove-bridge companion snap for fresh DDS discovery ────────────
echo "Restarting foxglove-bridge snap service..."
if command -v snap > /dev/null 2>&1; then
    snap restart foxglove-bridge 2>/dev/null || \
        echo "WARNING: foxglove-bridge snap not installed or not running"
fi

# ── Launch full demo ──────────────────────────────────────────────────────────
exec ros2 launch so101_ros2 full_demo_launch.py \
    engine_path:="$ENGINE"
