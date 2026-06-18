import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import math
from so101_ros2.lerobot.so101 import SO101

# Joint names in the same order as the leader publisher and gesture node
_JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

class LeRobotJointStateSubscriber(Node):

    # Safe retreat pose: arm pulls back and up, away from the workspace
    RETREAT_DEG = {
        'shoulder_pan':  0.0,
        'shoulder_lift': -30.0,
        'elbow_flex':    60.0,
        'wrist_flex':    0.0,
        'wrist_roll':    0.0,
        'gripper':       30.0,
    }

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
        self._prev_gesture_active = False
        self._handoff_countdown = 0
        self._handoff_slow_step = self.interpolation_step / 3.0  # 1/3 of normal rate

        # 2. Collision gate — VOLATILE QoS so stale messages from previous sessions
        #    are not received on startup (prevents spurious retreat at launch)
        _volatile_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10)
        self.collision_detected = False
        self._pre_retreat_positions = None
        self.create_subscription(Bool, '/collision_warning', self._collision_cb, _volatile_qos)

        # 3. Gesture mux — gesture node publishes here when idle animation runs
        self.gesture_active = False
        self.create_subscription(
            Bool, '/gesture_active',
            self._gesture_active_cb, 10)
        self.create_subscription(
            JointState, '/gesture/joint_states',
            self.gesture_states_callback, 10)

        # 4. Create a high-frequency timer (e.g., 50Hz / 0.02s)
        self.timer = self.create_timer(0.02, self.interpolation_callback)

        # 5. Publisher for the follower's actual interpolated position (radians) — used by digital twin
        self._follower_js_pub = self.create_publisher(JointState, '/follower/joint_states', 10)        
        
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
        if msg.data == self.collision_detected:
            return
        self.collision_detected = msg.data
        if msg.data:
            # False→True: save current pose, override goal to retreat position
            if self.current_positions is not None:
                self._pre_retreat_positions = self.current_positions.copy()
            self.goal_positions = self.RETREAT_DEG.copy()
            self.get_logger().warn('Follower RETREATING — collision detected')
        else:
            # True→False: next teleop/gesture callback will restore goal
            self.get_logger().warn('Follower RESUMED — collision cleared')

    def _gesture_active_cb(self, msg: Bool):
        if self._prev_gesture_active and not msg.data:
            self._handoff_countdown = 50
        self.gesture_active = msg.data
        self._prev_gesture_active = msg.data

    def gesture_states_callback(self, msg: JointState):
        """Gesture commands — only accepted while gesture mode is active."""
        if not self.gesture_active:
            return
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

    def joint_states_callback(self, msg: JointState):
        """Leader callback — gated by gesture_active flag."""
        if self.gesture_active:
            return
        self.joint_states_callback_impl(msg)

    def joint_states_callback_impl(self, msg: JointState):
        if self.robot is None:
            return

        # Collision retreat takes priority — ignore teleop/gesture updates
        if self.collision_detected:
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
        if self.robot is None or self.goal_positions is None or self.current_positions is None:
            return

        # No freeze here — when collision_detected=True, goal_positions has
        # already been set to RETREAT_DEG by _collision_cb, so the LERP below
        # smoothly carries the arm to the retreat pose automatically.

        if self._handoff_countdown > 0:
            effective_step = self._handoff_slow_step
            self._handoff_countdown -= 1
        else:
            effective_step = self.interpolation_step

        # Linear interpolation (LERP) logic
        changed = False
        for joint in self.current_positions:
            target = self.goal_positions[joint]
            current = self.current_positions[joint]

            # Move current position a small step toward target
            diff = target - current
            if abs(diff) > 0.1:  # Threshold to stop micro-vibrations
                self.current_positions[joint] += diff * effective_step
                changed = True

        if changed:
            try:
                # Send the "smoothed" positions to the motors
                self.robot._bus.sync_write("Goal_Position", self.current_positions)
            except Exception as e:
                self.get_logger().error(f"Write error: {e}")

        # Always publish follower's actual interpolated position so the digital twin stays in sync
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = _JOINT_NAMES
        js.position = [
            float(self.current_positions.get(n, 0.0)) * math.pi / 180.0
            for n in _JOINT_NAMES
        ]
        self._follower_js_pub.publish(js)  


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
