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
            'launch/so101_publisher_launch.py',
            'launch/so101_subscriber_launch.py',
            'launch/teleop_launch.py',
            'launch/teleop_gesture_launch.py',
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
    description='SO-101 ROS2 control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'so101_ros2_pub = so101_ros2.so101_ros2_pub:main',
            'so101_ros2_sub = so101_ros2.so101_ros2_sub:main',
            'gesture_node   = so101_ros2.gesture_node:main',
        ],
    },
)
