"""
Teleop + Gesture launch
------------------------
Starts leader, follower, gesture node, and digital twin bridge.

The gesture node owns /joint_states. Leader publishes to /leader/joint_states.
After idle_timeout seconds of no teleop, the arm enters gesture mode automatically.
Moving the leader arm resumes teleop after a smooth home return.

Usage:
    ros2 launch so101_ros2 teleop_gesture_launch.py
    ros2 launch so101_ros2 teleop_gesture_launch.py idle_timeout:=10.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    idle_arg = DeclareLaunchArgument(
        "idle_timeout",
        default_value="5.0",
        description="Seconds of no teleop input before gesture mode starts (float e.g. 5.0)",
    )

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

    return LaunchDescription([
        idle_arg,
        leader_node,
        follower_node,
        gesture_node,
    ])
