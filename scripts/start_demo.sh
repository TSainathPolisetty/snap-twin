#!/bin/bash
# snap/local/start_demo.sh
# Snap-aware entry point for the main demo.
# Runs as the snap-twin daemon; replaces start_gesture_demo.sh.
set -e

ros2 launch so101_ros2 full_demo_launch.py \
    engine_path:="$SNAP/models/depth_anything_v2_small.engine"
