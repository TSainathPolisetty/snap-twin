#!/bin/bash

# --- CONFIGURATION ---
BASE_DIR="/home/ubuntu/demo"
SNAP_DIR="$BASE_DIR/src/snap-twin"
SCRIPT_DIR="$SNAP_DIR/so101_ros2/scripts"

# 1. KILL ZOMBIES (Crucial for Serial Ports)
echo "[1/3] Clearing old serial locks (PIDs 7331, 7334, 7335, etc.)..."
pkill -9 -f so101_ros2 > /dev/null 2>&1
pkill -9 -f digital_twin > /dev/null 2>&1
pkill -9 -f simple_cors_server > /dev/null 2>&1
sleep 1 

# 2. SETUP ENVIRONMENT (The fix for your "New Terminal" issue)
echo "[2/3] Sourcing Environment..."
conda deactivate 2>/dev/null
conda deactivate 2>/dev/null

# Sourcing Jazzy (based on your error logs) and your workspace
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
source "$BASE_DIR/install/setup.bash"

# 3. LAUNCH TIERS
echo "[3/3] Starting Asset Server & Bridge..."

# Tier A: Asset Server (CORS) - Background
cd "$SNAP_DIR"
python3 simple_cors_server.py > /dev/null 2>&1 &

# Tier B: Digital Twin Bridge - Background
python3 "$SCRIPT_DIR/so101_digital_twin.py" > /dev/null 2>&1 &

# Tier C: Hardware Drivers - Foreground
echo "------------------------------------------------"
echo "DIGITAL TWIN ONLINE"
echo "Hardware: /dev/ttyACM0 & ACM1"
echo "Foxglove Bridge: Port 8765"
echo "Asset Server: Port 8080"
echo "------------------------------------------------"

# Ensure background tasks die when you Ctrl+C
trap "echo 'Shutting down...'; kill $(jobs -p); exit" SIGINT

ros2 launch so101_ros2 teleop_launch.py
