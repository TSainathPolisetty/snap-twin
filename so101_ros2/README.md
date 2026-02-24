# SO101 ROS2 Control

This repository shows how the SO101 robot arm can be used with ROS2. 

## Installation
1. Install ROS2: https://docs.ros.org/en/humble/Installation.html.
2. Create a ROS2 Workspace
    ```
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```
3. Clone this repo
    ```
    git clone https://github.com/msf4-0/so101_ros2.git
    ```
4. Install Dependencies
    ```
    pip install deepdiff feetech-servo-sdk
    ```
5. Build package
    ```
    cd ~/ros2_ws
    colcon build --packages-select so101_ros2
    source install/setup.bash
    ```

## Robot Setup
Follow the assembly instructions to build the SO101 arm in https://huggingface.co/docs/lerobot/so101.

## Usage
### Teleoperation (ROS Publisher & Subscriber)
To perform teleoperation, you will need two SO101 arms, one to act as the leader, and the other to act as the follower. Connect both arms to the computer and find their USB port ID.

#### If you are on Windows and working in WSL, you need to attach the USB device to WSL:
```
usbipd list # Find the Bus ID for COM no.
usbipd bind --busid 1-4 # Bind corresponding Bus ID, e.g. 1-4
usbipd attach --wsl --busid 1-4 
```
Then in WSL, check that the USB device is recognized (usually they will appear as /dev/ttyACM0 and /dev/ttyACM1)
```
ls /dev/tty*
```
#### Calibrate both arms
```
# Leader arm. Change the port to the one you identified. 
# The name can be anything, as long as you use back the same name for the leader later.
python3 ~/ros_ws/src/so101_ros2/so101_ros2/so101_control.py --port="/dev/ttyACM0" --name="so101_leader" --recalibrate=True

# Follower arm. Change the port to the one you identified. 
# The name can be anything, as long as you use back the same name for the leader later.
python3 ~/ros_ws/src/so101_ros2/so101_ros2/so101_control.py --port="/dev/ttyACM1" --name="so101_follower" --recalibrate=True
```
#### Start Publisher
The publisher node will read the joint state from the leader and publish it as a sensor_msgs.msg.JointState message to topic \joint_state.
```
ros2 launch so101_ros2 so101_publisher_launch.py
```
#### Start Subscriber
The subscriber node will subscribe to the \joint_state topic and write the joint state data received to the follower arm.
```
ros2 launch so101_ros2 so101_subscriber_launch.py
```
You can now move the leader arm to teleoperate the follower arm!

### Record and Replay Movements
You can record an episode of movement on a robot arm and replay it later.

#### To Record an Episode of Movement
```
# Change the port and name to the robot arm you want to use:
python3 ~/ros_ws/src/so101_ros2/so101_ros2/so101_control.py --port= <robot USB port ID> --name= <robot name> --record <filename.json> --rate <record rate in Hz>
```
#### Replay a Recorded Episode
```
# Change the port and name to the robot arm you want to use:
python3 ~/ros_ws/src/so101_ros2/so101_ros2/so101_control.py --port= <robot USB port ID> --name= <robot name> --replay <filename.json> 
```