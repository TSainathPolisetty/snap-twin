#!/bin/bash
# snap/local/start_demo.sh
# Snap-aware entry point for the main demo.
# Runs as the snap-twin daemon; replaces start_gesture_demo.sh.
set -e

# ── Restart foxglove-bridge companion snap for fresh DDS discovery ────────────
snap restart foxglove-bridge 2>/dev/null || true

# ── Patch URDF with current machine IP so Foxglove can fetch STL meshes ───────
# Done here (not only in install hook) so the URL stays correct after IP changes
# or when running Foxglove locally on this machine (use localhost in that case).
URDF="$SNAP_COMMON/final_twin.urdf"
if [ -f "$URDF" ]; then
    MACHINE_IP=$(hostname -I | awk '{print $1}')
    sed -i "s|http://[0-9.]*:8080|http://${MACHINE_IP}:8080|g" "$URDF"
fi

ros2 launch so101_ros2 full_demo_launch.py \
    engine_path:="$SNAP/models/depth_anything_v2_small.engine"
