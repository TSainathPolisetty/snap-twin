#!/bin/bash

# Define project path
BASE_DIR="$HOME/demo/snap-twin/test-base"

# Start a new tmux session named 'digital_twin'
tmux new-session -d -s digital_twin

# Pane 1: Gazebo Server
tmux send-keys -t digital_twin "gz sim -s -r $BASE_DIR/demo_world.sdf" C-m

# Split for Pane 2: ROS-GZ Bridge
tmux split-window -h -t digital_twin
tmux send-keys -t digital_twin "ros2 run ros_gz_bridge parameter_bridge '/world/demo_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' --ros-args -r /world/demo_world/dynamic_pose/info:=/tf" C-m

# Split for Pane 3: Foxglove Bridge
tmux select-pane -t 0
tmux split-window -v -t digital_twin
tmux send-keys -t digital_twin "ros2 launch foxglove_bridge foxglove_bridge_launch.xml" C-m

# Split for Pane 4: Static TF Floor
tmux select-pane -t 1
tmux split-window -v -t digital_twin
tmux send-keys -t digital_twin "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world floor" C-m

# Split for Pane 5: Static TF Table
tmux select-pane -t 2
tmux split-window -v -t digital_twin
tmux send-keys -t digital_twin "ros2 run tf2_ros static_transform_publisher 0 0 0.75 0 0 0 world table" C-m

# Split for Pane 6: URDF Publisher
tmux select-pane -t 3
tmux split-window -v -t digital_twin
tmux send-keys -t digital_twin "ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"\$(cat $BASE_DIR/scene_description.urdf)\"" C-m

# Attach to the session
tmux attach-session -t digital_twin
