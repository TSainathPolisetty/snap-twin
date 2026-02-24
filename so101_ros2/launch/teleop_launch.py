from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. THE LEADER (Publisher)
        Node(
            package='so101_ros2',
            executable='so101_ros2_pub',
            name='leader_node',
            output='screen',
            parameters=[
                {'robot_name': 'leader'},
                {'port': '/dev/ttyACM1'},
                {'recalibrate': False}
            ]
        ),
        # 2. THE FOLLOWER (Subscriber)
        Node(
            package='so101_ros2',
            executable='so101_ros2_sub',
            name='follower_node',
            output='screen',
            parameters=[
                {'robot_name': 'follower'},
                {'port': '/dev/ttyACM0'},
                {'recalibrate': False}
            ]
        )
    ])
