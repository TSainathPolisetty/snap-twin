#!/bin/bash
# snap/local/start_demo.sh
# Snap-aware entry point for the main demo.
# Runs as the snap-twin daemon; replaces start_gesture_demo.sh.
set -e

ENGINE="$SNAP_COMMON/models/depth_anything_v2_small.engine"
URDF_SRC="$SNAP/final_twin.urdf"
URDF_DEST="$SNAP_COMMON/final_twin.urdf"

# ── Verify TRT engine exists (built by install hook) ──────────────────────────
if [ ! -f "$ENGINE" ]; then
    echo "ERROR: TRT engine not found at $ENGINE"
    echo "The install hook may have failed. Check:"
    echo "  snap logs snap-twin --follow"
    echo "  cat $SNAP_COMMON/models/conversion.log"
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

# ── Launch full demo ──────────────────────────────────────────────────────────
exec ros2 launch so101_ros2 full_demo_launch.py \
    engine_path:="$ENGINE"
