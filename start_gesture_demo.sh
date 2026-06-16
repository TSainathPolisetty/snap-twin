#!/bin/bash
# =============================================================================
# start_gesture_demo.sh
# SO-101 Gesture / Idle Demo
# -----------------------------------------------------------------------------
# Starts:
#   - Asset server (STL meshes for Foxglove 3D view)
#   - Digital twin bridge (Foxglove WebSocket on port 8765)
#   - Leader arm (teleop publisher)
#   - Follower arm (subscriber, respects gesture_active flag)
#   - Gesture node (idle animation after X seconds of no input)
#
# Usage:
#   bash start_gesture_demo.sh
#   bash start_gesture_demo.sh --idle-timeout 10
#
# Foxglove: connect to ws://<device-ip>:8765
# =============================================================================

set -e

# --- Derive paths relative to this script (works anywhere) ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SNAP_DIR="$SCRIPT_DIR"
ROS_SCRIPTS="$SNAP_DIR/so101_ros2/scripts"

# --- Parse optional args ---
IDLE_TIMEOUT="5.0"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --idle-timeout) IDLE_TIMEOUT="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--idle-timeout SECONDS]"
            echo "  --idle-timeout  seconds before gesture mode activates (default: 5.0)"
            exit 0 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

# --- 1. Kill any leftover processes from previous run ---
echo "[1/4] Clearing previous session..."
pkill -9 -f so101_ros2     > /dev/null 2>&1 || true
pkill -9 -f digital_twin   > /dev/null 2>&1 || true
pkill -9 -f simple_cors    > /dev/null 2>&1 || true
pkill -9 -f gesture_node   > /dev/null 2>&1 || true
sleep 1

# --- 2. Environment ---
echo "[2/4] Sourcing ROS2 environment..."
# Unset any stale environment from previous sessions
unset ROS_DISTRO AMENT_PREFIX_PATH AMENT_LIBRARIES
unset COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH RMW_IMPLEMENTATION

# ROS2 via Humble snap (dynamic revision)
ROS_SNAP_REV=$(ls /snap/ros-humble-ros-base/ \
    | grep -v current | sort -n | tail -1)
HUMBLE=/snap/ros-humble-ros-base/$ROS_SNAP_REV/opt/ros/humble

source "$HUMBLE/setup.bash"
source ~/automate-demo/ros2_ws/install/setup.bash

export LD_LIBRARY_PATH=$HUMBLE/lib/aarch64-linux-gnu:$HUMBLE/lib:\
/snap/ros-humble-ros-base/$ROS_SNAP_REV/usr/lib/aarch64-linux-gnu:\
/snap/ros-humble-ros-base/$ROS_SNAP_REV/usr/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$HUMBLE/local/lib/python3.10/dist-packages:\
$HUMBLE/lib/python3.10/site-packages:\
~/automate-demo/ros2_ws/build/so101_ros2:/home/ubuntu:$PYTHONPATH
export AMENT_PREFIX_PATH=~/automate-demo/ros2_ws/install/so101_ros2:$HUMBLE
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PATH=$HUMBLE/bin:$PATH:/home/ubuntu/.local/bin

# Verify Humble loaded correctly
if [ "$ROS_DISTRO" != "humble" ]; then
    echo "ERROR: ROS_DISTRO=$ROS_DISTRO — expected humble. Aborting."
    exit 1
fi

# --- 3. Background services ---
echo "[3/4] Starting Foxglove services..."

cd "$SNAP_DIR"

# Ensure foxglove-bridge snap is not occupying port 8765
sudo snap stop foxglove-bridge 2>/dev/null || true

# Patch URDF mesh URLs with current device IP (fixes hardcoded Rubik Pi address)
DEVICE_IP=$(hostname -I | awk '{print $1}')
sed -i "s|http://[0-9.]*:8080|http://$DEVICE_IP:8080|g" \
    "$SNAP_DIR/final_twin.urdf"
echo "  URDF mesh URLs updated → http://$DEVICE_IP:8080/assets/"

# Asset server — serves STL meshes and URDF to Foxglove on port 8080
python3 simple_cors_server.py > /tmp/asset_server.log 2>&1 &
ASSET_PID=$!
echo "  Asset server PID: $ASSET_PID (port 8080)"

# Digital twin bridge — FK transforms + Foxglove WebSocket on port 8765
python3 "$ROS_SCRIPTS/so101_digital_twin.py" > /tmp/digital_twin.log 2>&1 &
TWIN_PID=$!
echo "  Digital twin PID: $TWIN_PID (port 8765)"

sleep 1

# --- 4. Launch ---
echo "[4/4] Launching gesture demo..."
echo ""
echo "  ┌─────────────────────────────────────────────────┐"
echo "  │           SO-101 GESTURE DEMO ONLINE            │"
echo "  │                                                  │"
echo "  │  Idle timeout   : ${IDLE_TIMEOUT}s                        │"
echo "  │  Gesture loop   : look → wave → beckon → wiggle │"
echo "  │  Foxglove       : ws://$(hostname -I | awk '{print $1}'):8765      │"
echo "  │  Assets         : http://$(hostname -I | awk '{print $1}'):8080    │"
echo "  │                                                  │"
echo "  │  Move leader arm to resume teleop at any time.  │"
echo "  └─────────────────────────────────────────────────┘"
echo ""

# Graceful shutdown on Ctrl+C
cleanup() {
    echo ""
    echo "Shutting down gesture demo..."
    kill "$ASSET_PID" "$TWIN_PID" 2>/dev/null || true
    pkill -f so101_ros2     2>/dev/null || true
    pkill -f gesture_node   2>/dev/null || true
    pkill -f digital_twin   2>/dev/null || true
    echo "Done."
    exit 0
}
trap cleanup SIGINT SIGTERM

# Grant serial port access for this session
sudo chmod a+rw /dev/ttyACM0 /dev/ttyACM1 2>/dev/null || true

# Foreground: ROS2 launch (leader + follower + gesture node)
IDLE_TIMEOUT_FLOAT=$(python3 -c "print(float('$IDLE_TIMEOUT'))")
ros2 launch so101_ros2 teleop_gesture_launch.py \
    idle_timeout:="$IDLE_TIMEOUT_FLOAT"
