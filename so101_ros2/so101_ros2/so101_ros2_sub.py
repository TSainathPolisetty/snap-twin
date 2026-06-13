import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import math
from so101_ros2.lerobot.so101 import SO101

class LeRobotJointStateSubscriber(Node):

    def __init__(self):
        super().__init__('lerobot_subscriber')

        # Declare ROS Parameters
        self.declare_parameter('robot_name', "follower")
        self.declare_parameter('port', "/dev/ttyACM0")
        self.declare_parameter('recalibrate', False)

        # 1. State tracking for interpolation
        self.current_positions = None
        self.goal_positions = None
        self.interpolation_step = 0.1  # How much to move per tick (0.0 to 1.0)

        # 2. Collision gate
        self.collision_detected = False
        self.create_subscription(Bool, '/collision_warning', self._collision_cb, 10)

        # 3. Gesture mux — gesture node publishes here when idle animation runs
        self.gesture_active = False
        self.create_subscription(
            Bool, '/gesture_active',
            lambda msg: setattr(self, 'gesture_active', msg.data), 10)
        self.create_subscription(
            JointState, '/gesture/joint_states',
            self.gesture_states_callback, 10)

        # 4. Create a high-frequency timer (e.g., 50Hz / 0.02s)
        self.timer = self.create_timer(0.02, self.interpolation_callback)        
        
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


    def _collision_cb(self, msg: Bool):
        if msg.data != self.collision_detected:
            self.collision_detected = msg.data
            state = 'STOPPED — collision detected' if msg.data else 'RESUMED'
            self.get_logger().warn(f'Follower {state}')

    def gesture_states_callback(self, msg: JointState):
        """Gesture commands always pass through — no gate checks."""
        self.joint_states_callback_impl(msg)

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

    #def joint_states_callback(self, msg: JointState):
     #   if self.robot is None:
      #      self.get_logger().warn("LeRobot arm not initialized. Skipping joint state update.")
       #     return
       # 
        #joint_states = {} # This will be populated to be a dict of joint_name: joint_angle_deg
        #for joint_name, joint_value in zip(msg.name, msg.position):
            joint_states[joint_name] = joint_value / (math.pi) * 180
        #
        #try:
         #   self.get_logger().info(f"Sent action: {joint_states}") # Too verbose for constant updates
          #  self.robot._bus.sync_write("Goal_Position", joint_states)
        #except Exception as e:
         #   self.get_logger().error(f"Error sending action to lerobot arm: {e}")
    
    def joint_states_callback(self, msg: JointState):
        """Leader callback — gated by gesture_active flag."""
        if self.gesture_active:
            return
        self.joint_states_callback_impl(msg)

    def joint_states_callback_impl(self, msg: JointState):
        if self.robot is None:
            return

        # UPDATE GOALS ONLY (No direct motor writes)
        new_goals = {}
        for joint_name, joint_value in zip(msg.name, msg.position):
            # Convert radians to degrees
            new_goals[joint_name] = joint_value / (math.pi) * 180
        
        self.goal_positions = new_goals
        
        # Initialize current_positions on the very first message
        if self.current_positions is None:
            self.current_positions = self.goal_positions.copy()
         
    def interpolation_callback(self):
        if self.robot is None or self.goal_positions is None:
            return

        # COLLISION GATE — freeze arm immediately, do not write to motors
        if self.collision_detected:
            return

        # Linear interpolation (LERP) logic
        changed = False
        for joint in self.current_positions:
            target = self.goal_positions[joint]
            current = self.current_positions[joint]
            
            # Move current position a small step toward target
            diff = target - current
            if abs(diff) > 0.1:  # Threshold to stop micro-vibrations
                self.current_positions[joint] += diff * self.interpolation_step
                changed = True

        if changed:
            try:
                # Send the "smoothed" positions to the motors
                self.robot._bus.sync_write("Goal_Position", self.current_positions)
            except Exception as e:
                self.get_logger().error(f"Write error: {e}")  


def main(args=None):
    rclpy.init(args=args)
    lerobot_subscriber = LeRobotJointStateSubscriber()
    rclpy.spin(lerobot_subscriber)
    
    # Ensure disconnection when ROS node shuts down
    if lerobot_subscriber.robot is not None:
        lerobot_subscriber.get_logger().info("Disconnecting lerobot arm...")
        lerobot_subscriber.robot.disconnect() # [cite: 2]
        lerobot_subscriber.get_logger().info("LeRobot arm disconnected.")

    lerobot_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
