import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the share directory of your package
    # This is useful if you want to include other files (like config files) later
    # package_share_directory = get_package_share_directory('lerobot_ros_control')

    return LaunchDescription([
        Node(
            package='so101_ros2',
            executable='so101_ros2_sub',
            name='so101_subscriber_node',
            output='screen', # Show stdout/stderr in the console
            emulate_tty=True, # Required for colored output and some logging
            parameters=[
                {'robot_name': 'so101_follower'},
                {'port': '/dev/ttyACM1'},
                {'recalibrate': False}
            ]
        )
    ])