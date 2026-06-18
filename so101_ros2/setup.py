from setuptools import find_packages, setup

package_name = 'so101_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/teleop_launch.py',
            'launch/teleop_gesture_launch.py',
            'launch/full_demo_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    package_data={
        'so101_ros2': ['config/*.json'],
    },
    include_package_data=True,
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='SO-101 arm teleop and gesture demo for Jetson Orin NX',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'so101_ros2_pub    = so101_ros2.so101_ros2_pub:main',
            'so101_ros2_sub    = so101_ros2.so101_ros2_sub:main',
            'gesture_node      = so101_ros2.gesture_node:main',
            'depth_anything    = so101_ros2.depth_anything_node:main',
            'collision_checker = so101_ros2.collision_checker_node:main',
            'frame_display     = so101_ros2.frame_display_node:main',
            'overhead_vision   = so101_ros2.overhead_vision_node:main',
        ],
    },
)
