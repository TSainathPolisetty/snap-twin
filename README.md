# Snap-Twin

Real-time digital twin and teleoperation system for the **SO-101 robotic arm**, running on the **NVIDIA Jetson Orin NX**. Streams live joint state transforms to Foxglove Studio and supports a gesture idle-animation mode when the leader arm is not being moved.

## System Stack

| Layer | Component |
|---|---|
| **Compute** | Jetson Orin NX (Advantech AFE-R750, Ubuntu 22.04) |
| **Middleware** | ROS 2 Humble (snap: `ros-humble-ros-base`) |
| **Visualization** | Foxglove Studio (WebSocket on port 8765) |
| **Depth Model** | Depth Anything V2 Small — TensorRT fp16 engine |
| **Motor Driver** | Feetech STS3215 via USB serial (`/dev/ttyACM0/1`) |

## Repository Layout

```
snap-twin/
├── assets/                    # SO-101 STL/PART mesh files (served to Foxglove)
├── scripts/
│   ├── download_model.sh      # Download Depth Anything V2 ONNX from HuggingFace
│   ├── convert_model.sh       # Convert ONNX → TensorRT fp16 engine (trtexec)
│   ├── so101_digital_twin.py  # Standalone Foxglove WebSocket bridge (FK + /tf)
│   ├── so101_control.py       # CLI tool: calibrate / record / replay arm episodes
│   └── simple_cors_server.py  # HTTP server for STL meshes and URDF (port 8080)
├── so101_ros2/                # ROS 2 ament_python package
│   ├── launch/
│   │   ├── teleop_launch.py          # Leader + follower arms only
│   │   └── teleop_gesture_launch.py  # Leader + follower + gesture node
│   └── so101_ros2/            # Python module
│       ├── so101_ros2_pub.py  # Leader arm → publishes /joint_states
│       ├── so101_ros2_sub.py  # Follower arm → subscribes to /joint_states
│       ├── gesture_node.py    # Idle animation after N seconds of no teleop
│       ├── config/            # Arm calibration files (JSON, machine-specific)
│       └── lerobot/           # Feetech motor driver library
├── final_twin.urdf            # Robot model served to Foxglove for 3D view
├── start_twin.sh              # Entry point: teleop only
└── start_gesture_demo.sh      # Entry point: teleop + idle gesture mode
```

## Prerequisites

**ROS 2 Humble (snap)**
```bash
sudo snap install ros-humble-ros-base
sudo snap install foxglove-bridge
```

**Python dependencies**
```bash
pip3 install yourdfpy scipy foxglove-sdk huggingface_hub
```

**Build the ROS 2 workspace**
```bash
mkdir -p ~/automate-demo/ros2_ws/src
cd ~/automate-demo/ros2_ws/src
git clone <this-repo> snap-twin
cd ~/automate-demo/ros2_ws
colcon build --packages-select so101_ros2 --symlink-install
```

## Setup

**1. Calibrate the arms** (first-time only, one arm at a time):
```bash
# Leader arm
python3 scripts/so101_control.py --port /dev/ttyACM1 --name leader --recalibrate

# Follower arm
python3 scripts/so101_control.py --port /dev/ttyACM0 --name follower --recalibrate
```
Calibration is saved to `so101_ros2/so101_ros2/config/`.

**2. Download and convert the depth model** (first-time only):
```bash
bash scripts/download_model.sh   # downloads ONNX to ~/models/
bash scripts/convert_model.sh    # builds TensorRT engine (~8 min)
```
Engine is saved to `~/models/depth_anything_v2_small.engine`.

## Running

### Teleoperation only
```bash
bash start_twin.sh
```
Starts the asset server (port 8080), restarts the Foxglove bridge snap (port 8765), then launches leader + follower arm nodes. Connect Foxglove Studio to `ws://<device-ip>:8765`.

### Gesture demo (teleop + idle animation)
```bash
bash start_gesture_demo.sh
bash start_gesture_demo.sh --idle-timeout 10  # seconds before gesture activates
```
Starts the asset server, the custom digital twin bridge, and the full ROS 2 launch with gesture node. The arm performs an idle animation after `idle_timeout` seconds with no leader input. Moving the leader arm resumes teleop immediately.

## ROS 2 Topics

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | leader node | follower node, digital twin |
| `/gesture/joint_states` | `sensor_msgs/JointState` | gesture node | follower node, digital twin |
| `/gesture_active` | `std_msgs/Bool` | gesture node | follower node, digital twin |
| `/collision_warning` | `std_msgs/Bool` | *(external)* | follower node |

## Acknowledgements

- **Hardware Interface:** Low-level serial communication and ROS 2 nodes based on [msf4-0/so101_ros2](https://github.com/msf4-0/so101_ros2).
- **Original Hardware Design:** SO-101 arm designs from the [Hugging Face LeRobot](https://github.com/huggingface/lerobot) project.
