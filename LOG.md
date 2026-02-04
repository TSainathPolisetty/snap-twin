# SnapTwin Technical Log

## Entry 01: Environment & ROS 2 Stack Setup
**Date:** 2026-02-04
**Hardware:** Rubik Pi (QCS6490) | **OS:** Ubuntu 24.04 (Noble)

### Fixes & Workarounds
- **Dependency Blocker:** Resolved "Unable to locate package" for ROS 2 Jazzy by enabling `noble-updates` and `noble-backports` in `/etc/apt/sources.list.d/ubuntu.sources`.
- **Command Not Found:** Fixed missing `gz` command by installing `ros-jazzy-gz-tools-vendor`.
- **Environment:** Automated sourcing by adding `source /opt/ros/jazzy/setup.bash` to `~/.bashrc`.

### Current Status
- [x] Headless Gazebo Harmonic running `shapes.sdf`.
- [x] Foxglove WebSocket bridge active on port 8765.
- [x] Successful 3D visualization on remote laptop.
