import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from so101_ros2.lerobot.so101 import SO101

class LeRobotJointStatePublisher(Node):

    def __init__(self):
        super().__init__('lerobot_joint_state_publisher')

        # Declare ROS Parameters
        self.declare_parameter('robot_name', "so101_leader")
        self.declare_parameter('port', "/dev/ttyACM0")
        self.declare_parameter('recalibrate', False)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.port = self.get_parameter('port').value
        self.recalibrate = self.get_parameter('recalibrate').value

        self.publisher_ = self.create_publisher(JointState, '/joint_states', 1)
        self.timer = self.create_timer(1/10, self.publish_joint_states) # Publish every 100ms

        self.get_logger().info('LeRobotJointStatePublisher node has been started.')

        # Initialize lerobot arm
        self.robot = self.init_lerobot_arm()

        # IMPORTANT: Define the mapping from lerobot motor keys to JointState names.
        # Ensure the order matches the order in which you expect to read joint data.
        # The mapping must also match the mapping used in the SO101 class (see lerobot.so101.py)
        self.motor_key_to_joint_name = {
            0: "shoulder_pan",  # Example: map motor key 0 to 'joint_1' (shoulder_pan)
            1: "shoulder_lift",  # Example: map motor key 1 to 'joint_2' (shoulder_lift)
            2: "elbow_flex",  # Example: map motor key 2 to 'joint_3' (elbow_flex)
            3: "wrist_flex",  # Example: map motor key 3 to 'joint_4' (wrist_flex)
            4: "wrist_roll",  # Example: map motor key 4 to 'joint_5' (wrist_roll)
            5: "gripper",  # Example: map motor key 5 to 'joint_6' (gripper)
        }
        self.joint_names = [self.motor_key_to_joint_name[i] for i in sorted(self.motor_key_to_joint_name.keys())]


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
            msg.name = self.joint_names
            msg.position = joint_positions_rad
            # If you have velocity or effort data, populate them here:
            # msg.velocity = [...]
            # msg.effort = [...]

            self.publisher_.publish(msg)
            self.get_logger().info(f"Published JointState: {msg.position}")

        except Exception as e:
            self.get_logger().error(f"Error reading or publishing joint states: {e}")

def main(args=None):
    rclpy.init(args=args)
    lerobot_publisher = LeRobotJointStatePublisher()
    try:
        rclpy.spin(lerobot_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure lerobot disconnects when ROS node shuts down
        if lerobot_publisher.robot is not None:
            lerobot_publisher.get_logger().info("Disconnecting lerobot arm...")
            lerobot_publisher.robot.disconnect()
            lerobot_publisher.get_logger().info("LeRobot arm disconnected.")

        lerobot_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
