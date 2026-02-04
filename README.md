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
