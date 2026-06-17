"""
Full demo launch — teleop + gesture + depth + collision avoidance
-----------------------------------------------------------------
Starts all five nodes:
  leader          so101_ros2_pub   /dev/ttyACM1
  follower        so101_ros2_sub   /dev/ttyACM0
  gesture_node    idle animation + /gesture_active mux
  depth_anything  TRT depth inference on /dev/video0
  collision       checks /camera/depth/image_raw → /collision_warning
                  (started 3 s after depth node to allow warm-up)

Usage:
    ros2 launch so101_ros2 full_demo_launch.py
    ros2 launch so101_ros2 full_demo_launch.py idle_timeout:=10.0
    ros2 launch so101_ros2 full_demo_launch.py camera_device:=/dev/video2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ─────────────────────────────────────────────────────
    idle_arg = DeclareLaunchArgument(
        "idle_timeout",
        default_value="5.0",
        description="Seconds of no teleop input before gesture mode starts (float)",
    )

    engine_arg = DeclareLaunchArgument(
        "engine_path",
        default_value="/home/ubuntu/models/depth_anything_v2_small.engine",
        description="Path to TensorRT .engine file for Depth Anything V2",
    )

    camera_arg = DeclareLaunchArgument(
        "camera_device",
        default_value="/dev/video0",
        description="V4L2 camera device path (e.g. /dev/video0)",
    )

    # ── Arm nodes ────────────────────────────────────────────────────────────
    leader_node = Node(
        package="so101_ros2",
        executable="so101_ros2_pub",
        name="leader_node",
        output="screen",
        parameters=[{
            "robot_name": "leader",
            "port":       "/dev/ttyACM1",
            "recalibrate": False,
        }],
    )

    follower_node = Node(
        package="so101_ros2",
        executable="so101_ros2_sub",
        name="follower_node",
        output="screen",
        parameters=[{
            "robot_name": "follower",
            "port":       "/dev/ttyACM0",
            "recalibrate": False,
        }],
    )

    gesture_node = Node(
        package="so101_ros2",
        executable="gesture_node",
        name="gesture_node",
        output="screen",
        parameters=[{
            "idle_timeout":  LaunchConfiguration("idle_timeout"),
            "home_duration": 2.5,
        }],
    )

    # ── Perception nodes ─────────────────────────────────────────────────────
    depth_node = Node(
        package="so101_ros2",
        executable="depth_anything",
        name="depth_anything_node",
        output="screen",
        parameters=[{
            "engine_path":   LaunchConfiguration("engine_path"),
            "camera_device": LaunchConfiguration("camera_device"),
        }],
    )

    # Collision checker starts 3 s after depth node to let the TRT engine
    # deserialise and the camera pipeline warm up before calibration begins.
    collision_node = Node(
        package="so101_ros2",
        executable="collision_checker",
        name="collision_checker",
        output="screen",
    )

    collision_delayed = TimerAction(
        period=3.0,
        actions=[collision_node],
    )

    return LaunchDescription([
        idle_arg,
        engine_arg,
        camera_arg,
        leader_node,
        follower_node,
        gesture_node,
        depth_node,
        collision_delayed,
    ])
