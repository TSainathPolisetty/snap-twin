import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from so101_ros2.lerobot.so101 import SO101

_JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_roll", "gripper",
]


class SO101LeaderNode(Node):

    def __init__(self):
        super().__init__('leader_node')

        self.declare_parameter('robot_name', "so101_leader")
        self.declare_parameter('port', "/dev/ttyACM0")
        self.declare_parameter('recalibrate', False)

        self.robot_name = self.get_parameter('robot_name').value
        self.port = self.get_parameter('port').value
        self.recalibrate = self.get_parameter('recalibrate').value

        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1/10, self.publish_joint_states)

        self.get_logger().info('SO101LeaderNode started.')

        self.robot = self.init_lerobot_arm()


    def init_lerobot_arm(self):
        robot = SO101(port=self.port, name=self.robot_name, recalibrate=self.recalibrate)
        try:
            self.get_logger().info("Connecting to lerobot arm...")
            robot.connect()
            self.get_logger().info("LeRobot arm connected.")
            return robot
        except Exception as e:
            self.get_logger().error(f"Failed to connect to lerobot arm: {e}")
            rclpy.shutdown() # Shutdown ROS if robot connection fails
            return None

    def publish_joint_states(self):
        if self.robot is None:
            self.get_logger().warn("LeRobot arm not initialized. Skipping joint state publication.")
            return

        try:
            # Read current joint positions from the lerobot arm
            # The 'Present_Position' typically returns a list of joint angles in degrees
            # The order of the returned list depends on how the motors were defined in the config
            # Ensure this order matches your `motor_key_to_joint_name` mapping.
            joint_positions_dict = self.robot.get_device_state()
            self.get_logger().debug(f"Raw joint positions (deg): {joint_positions_dict}")

            # Transform joint positions from lerobot's internal representation (degrees)
            # to ROS JointState's representation (radians)
            # print(joint_positions_dict)
            joint_positions_rad = [float(pos_deg) / 180.0 * math.pi
                                   for joint_names, pos_deg in joint_positions_dict.items()]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = _JOINT_NAMES
            msg.position = joint_positions_rad
            self.publisher_.publish(msg)
            self.get_logger().debug(f"Published JointState: {msg.position}")

        except Exception as e:
            self.get_logger().error(f"Error reading or publishing joint states: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SO101LeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.robot is not None:
            node.get_logger().info("Disconnecting lerobot arm...")
            node.robot.disconnect()
            node.get_logger().info("LeRobot arm disconnected.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
