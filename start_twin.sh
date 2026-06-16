#!/bin/bash

# --- CONFIGURATION ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SNAP_DIR="$SCRIPT_DIR"
ROS_SNAP=/snap/ros-humble-ros-base/115
WS=/home/ubuntu/automate-demo/ros2_ws

# 1. SETUP ENVIRONMENT
echo "[1/3] Sourcing Environment..."
source $ROS_SNAP/opt/ros/humble/setup.bash
source $WS/install/setup.bash
export LD_LIBRARY_PATH=$ROS_SNAP/opt/ros/humble/lib/aarch64-linux-gnu:$ROS_SNAP/opt/ros/humble/lib:$ROS_SNAP/usr/lib/aarch64-linux-gnu:$ROS_SNAP/usr/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$WS/build/so101_ros2:$ROS_SNAP/opt/ros/humble/local/lib/python3.10/dist-packages:$ROS_SNAP/opt/ros/humble/lib/python3.10/site-packages:/home/ubuntu:$PYTHONPATH
export AMENT_PREFIX_PATH=$WS/install/so101_ros2:$ROS_SNAP/opt/ros/humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PATH=$ROS_SNAP/opt/ros/humble/bin:$PATH:/home/ubuntu/.local/bin

# 2. LAUNCH TIERS
echo "[2/3] Starting Asset Server & Foxglove Bridge..."

# Tier A: Asset Server (CORS) - Background
echo "  Starting Asset Server (port 8080)..."
cd "$SNAP_DIR"
python3 simple_cors_server.py > /tmp/asset_server.log 2>&1 &

# Tier B: Foxglove Bridge snap - restart to ensure clean state
echo "  Restarting Foxglove Bridge snap (port 8765)..."
sudo snap restart foxglove-bridge 2>/dev/null \
  && echo "  Foxglove Bridge restarted." \
  || echo "  Foxglove Bridge already running."

sleep 1

# Tier C: Hardware Drivers - Foreground
echo ""
echo "------------------------------------------------"
echo "  DIGITAL TWIN ONLINE"
echo "  Hardware:        /dev/ttyACM0 (follower)"
echo "                   /dev/ttyACM1 (leader)"
echo "  Foxglove Bridge: ws://$(hostname -I | awk '{print $1}'):8765"
echo "  Asset Server:    http://$(hostname -I | awk '{print $1}'):8080"
echo "------------------------------------------------"
echo ""
echo "[3/3] Launching teleop (Ctrl+C to stop)..."

trap "echo 'Shutting down...'; jobs -p | xargs -r kill; exit" SIGINT

ros2 launch so101_ros2 teleop_launch.py
