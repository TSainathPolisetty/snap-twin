#!/bin/bash

# --- Configuration Paths for Host ---
BASE_DIR="/home/ubuntu/demo/snap-twin/test-base"
VENV_PYTHON="/home/ubuntu/demo/snap-twin/host_venv/bin/python3"
XACRO_FILE="$BASE_DIR/scene_description.xacro"
FINAL_URDF="$BASE_DIR/final_scene.urdf"

# Start tmux session
tmux new-session -d -s digital_twin

# Pane 1: Gazebo Server (Physics)
tmux send-keys -t digital_twin "gz sim -s -r $BASE_DIR/demo_world.sdf" C-m

# Pane 2: ROS-GZ Bridge
tmux split-window -h -t digital_twin
tmux send-keys -t digital_twin "ros2 run ros_gz_bridge parameter_bridge '/model/so101_arm/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model' --ros-args -r /model/so101_arm/joint_state:=/joint_states" C-m

# Pane 3: Process Xacro and Start Python Viz Server
tmux select-pane -t 0
tmux split-window -v -t digital_twin
tmux send-keys -t digital_twin "cd $BASE_DIR && \
xacro scene_description.xacro > final_scene.urdf && \
sed -i 's|filename=\"assets/|filename=\"package://SO101/assets/|g' final_scene.urdf && \
$VENV_PYTHON viz_server.py" C-m

# Attach to view the server output
tmux attach-session -t digital_twin
