"""
Full demo launch - teleop + gesture + overhead vision + wrist depth + display
------------------------------------------------------------------------------
Starts all seven nodes:
  leader           so101_ros2_pub   /dev/ttyACM1
  follower         so101_ros2_sub   /dev/ttyACM0 — state machine driven by /overhead/obstacle_present
  gesture_node     idle animation + /gesture_active mux
  overhead_vision  HSV overhead segmentation on /dev/video0 — publishes /overhead/obstacle_present
  depth_anything   TRT depth inference on wrist camera (/dev/video2 by default)
  frame_display    MJPEG HTTP stream server on port 8081 — http://localhost:8081/stream
                   (started 5 s after launch)
  robot_state_pub  publishes /tf + /tf_static for Foxglove via foxglove-bridge snap

Usage:
    ros2 launch so101_ros2 full_demo_launch.py
    ros2 launch so101_ros2 full_demo_launch.py idle_timeout:=10.0
    ros2 launch so101_ros2 full_demo_launch.py camera_device:=/dev/video4
    ros2 launch so101_ros2 full_demo_launch.py table_height_m:=0.74
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Read URDF at launch time (before any nodes start) for robot_state_publisher.
    # Resolved relative to this launch file's location — works regardless of where
    # the repo is cloned. Snap overrides via SNAP_TWIN_DATA_DIR env var.
    _share_dir = (
        os.environ.get('SNAP_TWIN_DATA_DIR')
        or os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'share'))
    )
    _urdf_path = os.path.join(_share_dir, 'final_twin.urdf')
    with open(_urdf_path, 'r') as _f:
        _robot_description = _f.read()

    _default_engine_dir = (
        os.environ.get('SNAP_TWIN_DATA_DIR')
        or os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'models'))
    )
    _default_engine_path = os.path.join(_default_engine_dir, 'depth_anything_v2_small.engine')

    idle_arg = DeclareLaunchArgument(
        'idle_timeout',
        default_value='15.0',
        description='Seconds of no teleop input before gesture mode starts (float)',
    )

    engine_arg = DeclareLaunchArgument(
        'engine_path',
        default_value=_default_engine_path,
        description='Path to TensorRT .engine file for Depth Anything V2',
    )

    camera_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video2',
        description='Wrist depth camera device path (default /dev/video2)',
    )

    table_height_arg = DeclareLaunchArgument(
        'table_height_m',
        default_value='0.72',
        description='Measured camera-to-table distance in metres (measured: 0.72)',
    )

    leader_node = Node(
        package='so101_ros2',
        executable='so101_ros2_pub',
        name='leader_node',
        output='screen',
        parameters=[{
            'robot_name': 'leader',
            'port': '/dev/ttyACM1',
            'recalibrate': False,
        }],
    )

    follower_node = Node(
        package='so101_ros2',
        executable='so101_ros2_sub',
        name='follower_node',
        output='screen',
        parameters=[{
            'robot_name': 'follower',
            'port': '/dev/ttyACM0',
            'recalibrate': False,
        }],
    )

    gesture_node = Node(
        package='so101_ros2',
        executable='gesture_node',
        name='gesture_node',
        output='screen',
        parameters=[{
            'idle_timeout': LaunchConfiguration('idle_timeout'),
            'return_secs':   0.5,
            'speed_scale':   0.55,
        }],
    )

    overhead_node = Node(
        package='so101_ros2',
        executable='overhead_vision',
        name='overhead_vision_node',
        output='screen',
        parameters=[{
            'camera_device': '/dev/video0',
            'table_height_m': LaunchConfiguration('table_height_m'),
        }],
    )

    depth_node = Node(
        package='so101_ros2',
        executable='depth_anything',
        name='depth_anything_node',
        output='screen',
        parameters=[{
            'engine_path': LaunchConfiguration('engine_path'),
            'camera_device': LaunchConfiguration('camera_device'),
        }],
    )

    display_node = Node(
        package='so101_ros2',
        executable='frame_display',
        name='frame_display_node',
        output='screen',
        parameters=[{
            'mjpeg_port': 8081,
        }],
    )

    display_delayed = TimerAction(
        period=5.0,
        actions=[display_node],
    )

    # robot_state_publisher: computes /tf from /follower/joint_states + URDF FK.
    # foxglove-bridge snap (humble/stable channel) forwards all topics to Studio.
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': _robot_description}],
        remappings=[('joint_states', '/follower/joint_states')],
    )

    return LaunchDescription([
        idle_arg,
        engine_arg,
        camera_arg,
        table_height_arg,
        leader_node,
        follower_node,
        gesture_node,
        overhead_node,
        depth_node,
        display_delayed,
        robot_state_pub,
    ])
