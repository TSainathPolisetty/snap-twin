# Snap-Twin

Snap Twin is a high-fidelity Digital Twin for the **SO-101 robotic arm**, optimized for the **Qualcomm Rubik Pi**.

## System Stack
- **Compute:** Rubik Pi (Qualcomm QCS6490)
- **Middleware:** ROS 2 Jazzy Jalisco
- **Physics:** Gazebo Harmonic (Headless)
- **Visualization:** Foxglove Studio

## Quick Start
1. SSH into Rubik Pi and start `tmux`.
2. **Launch Sim:** `gz sim -s -r shapes.sdf`
3. **Launch Bridge:** `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
4. **View:** Connect Foxglove Studio to `ws://[PI_IP]:8765`

## Acknowledgements

This project integrates specialized open-source components to create a complete Digital Twin and Teleoperation system for the SO-101 robot arm.

* **Simulation & Kinematics:** The URDF models, Gazebo simulation environments, and MoveIt 2 configurations are derived from the work of [Pavankv92/lerobot_ws](https://github.com/Pavankv92/lerobot_ws).
* **Hardware Interface:** The low-level serial communication and ROS 2 hardware nodes are based on the implementation by [msf4-0/so101_ros2](https://github.com/msf4-0/so101_ros2).
* **Original Hardware Design:** All SO-101 robotic arm designs are part of the [Hugging Face LeRobot](https://github.com/huggingface/lerobot) project.
