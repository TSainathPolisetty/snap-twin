#!/bin/bash

SNAP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS=/home/ubuntu/automate-demo/ros2_ws

# Dynamic snap revision
unset ROS_DISTRO AMENT_PREFIX_PATH AMENT_LIBRARIES
unset COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH RMW_IMPLEMENTATION
ROS_SNAP_REV=$(ls /snap/ros-humble-ros-base/ | grep -v current | sort -n | tail -1)
SNAP=/snap/ros-humble-ros-base/$ROS_SNAP_REV

echo "[1/3] Setting up environment (snap rev $ROS_SNAP_REV)..."
source $SNAP/opt/ros/humble/setup.bash
source $WS/install/setup.bash

export LD_LIBRARY_PATH=$SNAP/opt/ros/humble/lib/aarch64-linux-gnu:$SNAP/opt/ros/humble/lib:$SNAP/usr/lib/aarch64-linux-gnu:$SNAP/usr/lib:$LD_LIBRARY_PATH
export PYTHONPATH=$WS/build/so101_ros2:$SNAP/opt/ros/humble/local/lib/python3.10/dist-packages:$SNAP/opt/ros/humble/lib/python3.10/site-packages:/home/ubuntu:$PYTHONPATH
export AMENT_PREFIX_PATH=$WS/install/so101_ros2:$SNAP/opt/ros/humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PATH=$SNAP/opt/ros/humble/bin:$PATH:/home/ubuntu/.local/bin

echo "[2/3] Starting Foxglove bridge..."
sudo snap restart foxglove-bridge
echo "  Foxglove bridge running on port 8765"

sleep 2

echo ""
echo "------------------------------------------------"
echo "DIGITAL TWIN ONLINE"
echo "Hardware: /dev/ttyACM0 (follower) & /dev/ttyACM1 (leader)"
echo "Foxglove Bridge: ws://$(hostname -I | awk '{print $1}'):8765"
echo "------------------------------------------------"
echo ""
echo "[3/3] Launching teleop (leader + follower arms)..."

ros2 launch so101_ros2 teleop_launch.py
