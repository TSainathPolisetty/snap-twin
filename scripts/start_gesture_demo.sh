#!/bin/bash
# =============================================================================
# start_gesture_demo.sh
# SO-101 Gesture / Idle Demo
# -----------------------------------------------------------------------------
# Starts:
#   - Asset server (STL meshes for Foxglove 3D view)
#   - foxglove-bridge snap (humble/stable channel, port 8765)
#   - Leader arm (teleop publisher)
#   - Follower arm (subscriber, respects gesture_active flag)
#   - Gesture node (idle animation after X seconds of no input)
#   - robot_state_publisher (computes /tf for Foxglove)
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
ROS_SCRIPTS="$SNAP_DIR/scripts"

# --- Parse optional args ---
IDLE_TIMEOUT="15.0"
TABLE_HEIGHT=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --idle-timeout) IDLE_TIMEOUT="$2"; shift 2 ;;
        --table-height) TABLE_HEIGHT="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--idle-timeout SECONDS] [--table-height METERS]"
            echo "  --idle-timeout  seconds before gesture mode activates (default: 15.0)"
            echo "  --table-height  camera-to-table distance in meters (required for sim population)"
            exit 0 ;;
        *) echo "Unknown arg: $1"; exit 1 ;;
    esac
done

# --- 1. Kill any leftover processes from previous run ---
echo "[1/4] Clearing previous session..."
pkill -9 -f so101_ros2          > /dev/null 2>&1 || true
pkill -9 -f robot_state_pub     > /dev/null 2>&1 || true
pkill -9 -f foxglove_bridge     > /dev/null 2>&1 || true
pkill -9 -f tf_static_relay     > /dev/null 2>&1 || true
pkill -9 -f simple_cors         > /dev/null 2>&1 || true
pkill -9 -f gesture_node        > /dev/null 2>&1 || true
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
echo "[3/4] Starting Foxglove services (foxglove-bridge, native Humble)..."

cd "$SNAP_DIR"

# Stop the foxglove-bridge snap service — we run the bridge binary directly in our
# own ROS2 environment (same user, same FastDDS config as all other nodes).
# This avoids snap isolation that prevents TRANSIENT_LOCAL /tf_static from being
# received by the bridge, which caused the 3D panel "No coordinate frames found" issue.
sudo snap stop foxglove-bridge 2>/dev/null || true

# Patch URDF mesh URLs with current device IP
DEVICE_IP=$(hostname -I | awk '{print $1}')
sed -i "s|http://[0-9.]*:8080|http://$DEVICE_IP:8080|g" \
    "$SNAP_DIR/final_twin.urdf"
echo "  URDF mesh URLs updated → http://$DEVICE_IP:8080/assets/"

# Asset server — serves STL meshes and URDF to Foxglove on port 8080
python3 "$SNAP_DIR/scripts/simple_cors_server.py" > /tmp/asset_server.log 2>&1 &
ASSET_PID=$!
echo "  Asset server PID: $ASSET_PID (port 8080)"

# foxglove-bridge — run the snap's binary directly in our ROS2 environment so it
# shares the same DDS participant as robot_state_publisher and can receive TRANSIENT_LOCAL /tf_static.
# Run in a subshell so the extra lib paths don't pollute the ros2 launch environment.
(
  FG_SNAP=/snap/foxglove-bridge/current
  export LD_LIBRARY_PATH=$FG_SNAP/opt/ros/snap/lib:$FG_SNAP/opt/ros/humble/lib:$LD_LIBRARY_PATH
  export AMENT_PREFIX_PATH=$FG_SNAP/opt/ros/snap:$AMENT_PREFIX_PATH
  ros2 run foxglove_bridge foxglove_bridge \
      --ros-args \
      -p port:=8765 \
      -p address:=0.0.0.0 \
      -p capabilities:="[clientPublish,parameters,parametersSubscribe,services,connectionGraph,assets]" \
      -p topic_whitelist:="['.*']" \
      -p asset_uri_allowlist:="['^package://(?:\\w+/)*\\w+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|urdf|xacro)$']"
) > /tmp/foxglove_bridge.log 2>&1 &
BRIDGE_PID=$!
echo "  foxglove-bridge PID: $BRIDGE_PID (port 8765)"

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
    kill "$ASSET_PID"     2>/dev/null || true
    kill "$BRIDGE_PID"    2>/dev/null || true
    pkill -f so101_ros2         2>/dev/null || true
    pkill -f robot_state_pub    2>/dev/null || true
    pkill -f foxglove_bridge    2>/dev/null || true
    pkill -f gesture_node       2>/dev/null || true
    echo "Done."
    exit 0
}
trap cleanup SIGINT SIGTERM

# Grant serial port access for this session
sudo chmod a+rw /dev/ttyACM0 /dev/ttyACM1 2>/dev/null || true

# Foreground: ROS2 launch (leader + follower + gesture + depth + display + robot_state_publisher)
# NOTE: For obstacle sim population in Foxglove (/obstacle_markers channel), set table_height_m to the
# measured distance (meters) from the Brio camera optical centre to the table surface.
# Example: ros2 launch ... table_height_m:=0.74
# Without this, overhead_vision cannot backproject obstacle positions to 3D.
IDLE_TIMEOUT_FLOAT=$(python3 -c "print(float('$IDLE_TIMEOUT'))")
ros2 launch so101_ros2 full_demo_launch.py \
    idle_timeout:="$IDLE_TIMEOUT_FLOAT" \
    engine_path:=/home/ubuntu/models/depth_anything_v2_small.engine \
    ${TABLE_HEIGHT:+table_height_m:=$TABLE_HEIGHT}

# (ros2 launch process manages all node lifecycles including robot_state_publisher)
