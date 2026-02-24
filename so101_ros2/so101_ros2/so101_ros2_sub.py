import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math
from so101_ros2.lerobot.so101 import SO101

class LeRobotJointStateSubscriber(Node):

    def __init__(self):
        super().__init__('lerobot_subscriber')

        # Declare ROS Parameters
        self.declare_parameter('robot_name', "so101_follower")
        self.declare_parameter('port', "/dev/ttyACM0")
        self.declare_parameter('recalibrate', False)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.port = self.get_parameter('port').value
        self.recalibrate = self.get_parameter('recalibrate').value

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info('LeRobotController node has been started.')

        # Initialize lerobot arm
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

    def joint_states_callback(self, msg: JointState):
        if self.robot is None:
            self.get_logger().warn("LeRobot arm not initialized. Skipping joint state update.")
            return
        
        joint_states = {} # This will be populated to be a dict of joint_name: joint_angle_deg
        for joint_name, joint_value in zip(msg.name, msg.position):
            joint_states[joint_name] = joint_value / (math.pi) * 180
        
        try:
            self.get_logger().info(f"Sent action: {joint_states}") # Too verbose for constant updates
            self.robot._bus.sync_write("Goal_Position", joint_states)
        except Exception as e:
            self.get_logger().error(f"Error sending action to lerobot arm: {e}")

def main(args=None):
    rclpy.init(args=args)
    lerobot_subscriber = LeRobotJointStateSubscriber()
    rclpy.spin(lerobot_subscriber)
    
    # Ensure lerobot disconnects when ROS node shuts down
    if lerobot_subscriber.robot is not None:
        lerobot_subscriber.get_logger().info("Disconnecting lerobot arm...")
        lerobot_subscriber.robot.disconnect() # [cite: 2]
        lerobot_subscriber.get_logger().info("LeRobot arm disconnected.")

    lerobot_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
