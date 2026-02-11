#!/bin/bash

# --- Configuration Paths ---
BASE_DIR="/home/ubuntu/demo/snap-twin/test-base"
MODELS_DIR="/home/ubuntu/demo/snap-twin/simulation/models"
XACRO_FILE="$BASE_DIR/scene_description.xacro"
FINAL_URDF="$BASE_DIR/final_scene.urdf"

# Start tmux session
tmux new-session -d -s digital_twin

# Pane 1: Gazebo Server (Physics)
tmux send-keys -t digital_twin "gz sim -s -r $BASE_DIR/demo_world.sdf" C-m

# Pane 2: ROS-GZ Bridge (Joint Messaging)
tmux split-window -h -t digital_twin
tmux send-keys -t digital_twin "ros2 run ros_gz_bridge parameter_bridge '/model/so101_arm/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model' --ros-args -r /model/so101_arm/joint_state:=/joint_states" C-m

# Pane 3: Process Xacro and Start Python Viz Server
# 1. Run Xacro to merge the files
# 2. Use sed to add 'package://SO101/' to mesh paths so the SDK can find them
# 3. Start the visualization server
tmux select-pane -t 0
tmux split-window -v -t digital_twin
tmux send-keys -t digital_twin "xacro $XACRO_FILE > $FINAL_URDF && \
sed -i 's|filename=\"assets/|filename=\"package://SO101/assets/|g' $FINAL_URDF && \
python3 $BASE_DIR/viz_server.py" C-m

tmux attach-session -t digital_twin
