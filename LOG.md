# SnapTwin Technical Log

## Entry 01: Environment & ROS 2 Stack Setup
**Date:** 2026-06-16
**Hardware:** Jetson Orin NX (Advantech AFE-R750) | **OS:** Ubuntu 22.04 (Jammy)

### Stack
- ROS 2 Humble via snap (`ros-humble-ros-base`)
- Foxglove bridge via snap (`foxglove-bridge`)
- Workspace: `~/demo/ros2_ws`

### Fixes & Workarounds
- **ROS snap env:** Humble not on system PATH by default — start scripts dynamically resolve snap revision and source `setup.bash` at runtime.
- **RMW:** `rmw_fastrtps_cpp` shared libs are inside the snap; `LD_LIBRARY_PATH` must include snap lib paths or nodes fail to init.
- **Python 3.10 path:** PYTHONPATH must include both `dist-packages` and `site-packages` inside the snap to pick up `rclpy`.

### Current Status
- [x] ROS 2 Humble sourced dynamically from snap revision.
- [x] Foxglove bridge snap active on port 8765.
- [x] Leader → follower teleoperation working over `/joint_states`.
- [x] Digital twin FK bridge streaming to Foxglove via custom WebSocket server.
- [x] Gesture idle animation node integrated.
- [x] Depth Anything V2 Small TensorRT fp16 engine built (`models/` repo-relative).
