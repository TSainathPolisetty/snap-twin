# SO-101 Digital Twin Baseline

This project establishes a minimal Digital Twin environment using **ROS 2 Jazzy**, **Gazebo Harmonic**, and **Foxglove Studio**. It synchronizes physics-based simulation with 3D web visualization.

## System Architecture
The pipeline relies on three distinct data layers:
1. **Physics Layer (Gazebo)**: Calculates collisions and model positions in the `demo_world.sdf`.
2. **Middleware Layer (ROS 2)**: Translates physics data into coordinate transforms (TF) and geometry descriptions (URDF).
3. **Visualization Layer (Foxglove)**: Renders the 3D scene in a browser via a WebSocket bridge.

## Prerequisites
- **World File**: `demo_world.sdf` (Contains `<model>` definitions with `PosePublisher` plugins).
- **Description File**: `scene_description.urdf` (Defines the visual geometry for ROS 2).

## The "Visual" Requirement Checklist
To see an object in 3D, Foxglove requires:
- **A Transform (TF)**: A "GPS" coordinate for the object.
- **A Robot Description**: A "Skin" for the coordinate (defined in the URDF).
- **A Connected Bridge**: The `foxglove_bridge` must be active and set to the `/robot_description` topic.
