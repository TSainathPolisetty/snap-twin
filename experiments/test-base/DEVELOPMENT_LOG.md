# Technical Operational Log - Feb 2026

## Terminal Pane Configuration
To maintain the pipeline, run these 6 commands in separate panes:

| Pane | Role | Command | Key Learning |
| :--- | :--- | :--- | :--- |
| **1** | **Gazebo Server** | `gz sim -s -r demo_world.sdf` | Simulations start **Paused** by default in headless mode. |
| **2** | **ROS-GZ Bridge** | `ros2 run ros_gz_bridge parameter_bridge "/world/demo_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V" --ros-args -r /world/demo_world/dynamic_pose/info:=/tf` | Uses `Pose_V` to bridge multiple models to one `/tf` topic. |
| **3** | **Foxglove Bridge** | `ros2 launch foxglove_bridge foxglove_bridge_launch.xml` | Acts as the WebSocket gateway (Port 8765). |
| **4** | **Floor Static TF** | `ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world floor` | Anchors the floor to the absolute world origin. |
| **5** | **Table Static TF** | `ros2 run tf2_ros static_transform_publisher 0 0 0.75 0 0 0 world table` | Places the table top at the correct simulation height. |
| **6** | **URDF Publisher** | `ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat scene_description.urdf)"` | Essential for Foxglove to render the 3D shapes. |

## Critical Debugging Notes
- **Lazy 0 vs Lazy 1**: The bridge stays at `Lazy 0` until a tool (Foxglove) actively subscribes to the topic.
- **URDF Strictness**: URDF does not support `plane` geometry (use thin boxes) and requires a single root link (`world`) connected via joints.
- **SDF Plugins**: The `PosePublisher` MUST be inside the `<model>` tags and use `<use_pose_vector_msg>true</use_pose_vector_msg>` for multi-model tracking.

## Next Steps: SO-101 Integration
1. Add the SO-101 model to `demo_world.sdf` with a `PosePublisher`.
2. Add SO-101 links to `scene_description.urdf` and joint it to the `table` link.
3. Bridge the `/joint_states` topic to visualize arm movement.
