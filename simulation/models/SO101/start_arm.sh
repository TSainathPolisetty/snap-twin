#!/bin/bash

# --- Configuration Paths ---
BASE_DIR="/home/ubuntu/demo/snap-twin/test-base"
MODELS_DIR="/home/ubuntu/demo/snap-twin/simulation/models"
# The original Xacro that includes the table, floor, and arm
XACRO_FILE="$BASE_DIR/scene_description.xacro"

# --- Environment Setup ---
# Register the manual index so ROS tools find the SO101 package
export AMENT_PREFIX_PATH=$HOME/.local:$AMENT_PREFIX_PATH
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$MODELS_DIR
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$MODELS_DIR

# Start tmux session
tmux new-session -d -s digital_twin

# Pane 1: Gazebo Server
# Loads the physics world
tmux send-keys -t digital_twin "gz sim -s -r $BASE_DIR/demo_world.sdf" C-m

# Pane 2: ROS-GZ Bridge (Joints Only)
# Bridging only joints prevents the 'Transform Tree Cycle' loop
tmux split-window -h -t digital_twin
tmux send-keys -t digital_twin "ros2 run ros_gz_bridge parameter_bridge \
'/model/so101_arm/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model' \
--ros-args -r /model/so101_arm/joint_state:=/joint_states" C-m

# Pane 3: Foxglove Bridge (The Asset Server)
# We ensure the Ament index is available so the bridge finds the meshes
tmux select-pane -t 0
tmux split-window -v -t digital_twin
tmux send-keys -t digital_twin "export AMENT_PREFIX_PATH=\$HOME/.local:\$AMENT_PREFIX_PATH && \
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
send_buffer_limit:=100000000 \
asset_uri_allowlist:=['package://.*']" C-m

# Pane 4: Robot State Publisher (The Complete Scene)
# 1. Process Xacro 2. Convert to package:// paths 3. Launch
tmux select-pane -t 1
tmux split-window -v -t digital_twin
tmux send-keys -t digital_twin "export AMENT_PREFIX_PATH=\$HOME/.local:\$AMENT_PREFIX_PATH && \
xacro $XACRO_FILE > /tmp/final_scene.urdf && \
sed -i 's|filename=\"assets/|filename=\"package://SO101/assets/|g' /tmp/final_scene.urdf && \
ros2 run robot_state_publisher robot_state_publisher /tmp/final_scene.urdf" C-m

# Attach to view logs
tmux attach-session -t digital_twin

