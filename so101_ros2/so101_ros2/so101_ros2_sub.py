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

# State machine states
_STATE_NORMAL    = 'NORMAL'
_STATE_RETREATING = 'RETREATING'
_STATE_HOLDING   = 'HOLDING'

class LeRobotJointStateSubscriber(Node):

    # Tucked retreat pose: arm folded compact — heavy elbow flexion, wrist curled in,
    # gripper nearly closed, low rather than raised.  Values confirmed by physical test.
    RETREAT_DEG = {
        'shoulder_pan':  0.0,
        'shoulder_lift': -99.0,
        'elbow_flex':    90.0,
        'wrist_flex':    90.0,   # increased from 60 → camera faces up, away from table
        'wrist_roll':    0.0,
        'gripper':       5.0,
    }

    # Max degrees any joint may move in a single 20 ms tick (applies to all motion,
    # including fast retreat entry, to prevent hardware-damaging step commands).
    MAX_STEP_DEG_PER_TICK = 8.0

    def __init__(self):
        super().__init__('lerobot_subscriber')

        # Declare ROS Parameters
        self.declare_parameter('robot_name', "follower")
        self.declare_parameter('port', "/dev/ttyACM0")
        self.declare_parameter('recalibrate', False)

        # Interpolation tracking
        self.current_positions = None
        self.goal_positions = None
        self.interpolation_step = 0.1
        self._prev_gesture_active = False
        self._handoff_countdown = 0
        self._handoff_slow_step = self.interpolation_step / 3.0

        # State machine — starts NORMAL, transitions via collision/convergence/clear-gate
        self._state = _STATE_NORMAL
        self._converge_ticks = 0   # consecutive ticks within 3° of RETREAT_DEG (for RETREATING→HOLDING)
        self._clear_streak  = 0    # consecutive obstacle-free readings in HOLDING
        self._clear_since   = None # time.monotonic() of first consecutive-clear reading

        # /overhead/obstacle_present — consulted ONLY while in HOLDING
        self._obstacle_present = False

        # Collision gate — VOLATILE QoS so stale messages from previous sessions
        # are not received on startup (prevents spurious retreat at launch)
        _volatile_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10)
        self.collision_detected = False
        self.create_subscription(Bool, '/collision_warning', self._collision_cb, _volatile_qos)

        # Overhead general obstacle presence (Phase 5 — ignored until state is HOLDING)
        self.create_subscription(Bool, '/overhead/obstacle_present', self._obstacle_present_cb, 10)

        # Gesture mux — gesture node publishes here when idle animation runs
        self.gesture_active = False
        self.create_subscription(Bool, '/gesture_active', self._gesture_active_cb, 10)
        self.create_subscription(JointState, '/gesture/joint_states', self.gesture_states_callback, 10)

        # 50 Hz interpolation timer
        self.timer = self.create_timer(0.02, self.interpolation_callback)

        # Publisher for the follower's actual interpolated position (radians) —
        # consumed by robot_state_publisher / digital twin for FK visualization
        self._follower_js_pub = self.create_publisher(JointState, '/follower/joint_states', 10)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.port = self.get_parameter('port').value
        self.recalibrate = self.get_parameter('recalibrate').value

        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info('LeRobotController node has been started.')
        self.robot = self.init_lerobot_arm()

    def _collision_cb(self, msg: Bool):
        prev = self.collision_detected
        self.collision_detected = msg.data
        if msg.data:
            if self._state == _STATE_NORMAL:
                # NORMAL → RETREATING: fresh retreat — reset both convergence and clear timer
                self._state = _STATE_RETREATING
                self._converge_ticks = 0
                self._clear_since = None  # fresh hold session
                self.goal_positions = self.RETREAT_DEG.copy()
                self.get_logger().warn('Follower RETREATING — collision detected')
            elif self._state == _STATE_HOLDING:
                # HOLDING → RETREATING: preserve _clear_since so the 3-second timer
                # accumulates across re-entries (wrist cam may re-trigger during hold)
                self._state = _STATE_RETREATING
                self._converge_ticks = 0
                self.goal_positions = self.RETREAT_DEG.copy()
                self.get_logger().warn('Follower RETREATING — collision re-detected from HOLDING')
            # If already RETREATING: do NOT reset converge_ticks — the arm is already moving
            # toward the retreat pose. Resetting would prevent convergence if the camera
            # keeps seeing something (e.g., arm structure) during the retreat motion.
            # goal_positions is already RETREAT_DEG so no update needed.
        else:
            if prev:
                self.get_logger().warn(
                    f'Collision cleared — still {self._state}, waiting for convergence/gate')

    def _obstacle_present_cb(self, msg: Bool):
        self._obstacle_present = msg.data

    def _gesture_active_cb(self, msg: Bool):
        if self._prev_gesture_active and not msg.data:
            # gesture→teleop handoff: trigger slow-step easing.
            # Same mechanism is also fired on HOLDING→NORMAL (see interpolation_callback).
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
            rclpy.shutdown()
            return None

    def joint_states_callback(self, msg: JointState):
        """Leader callback — gated by gesture_active flag."""
        if self.gesture_active:
            return
        self.joint_states_callback_impl(msg)

    def joint_states_callback_impl(self, msg: JointState):
        if self.robot is None:
            return
        # Only accept new goals in NORMAL state — collision retreat takes priority
        # until the full HOLDING→NORMAL gate clears (collision_detected alone
        # is not sufficient since it clears before convergence is reached)
        if self._state != _STATE_NORMAL:
            return
        new_goals = {}
        for joint_name, joint_value in zip(msg.name, msg.position):
            new_goals[joint_name] = joint_value / math.pi * 180
        self.goal_positions = new_goals
        if self.current_positions is None:
            self.current_positions = self.goal_positions.copy()

    def interpolation_callback(self):
        if self.robot is None or self.goal_positions is None or self.current_positions is None:
            return

        # --- State machine transitions ---
        if self._state == _STATE_RETREATING:
            # Check whether the arm has converged to the retreat pose
            max_diff = max(
                abs(self.current_positions.get(j, 0.0) - self.RETREAT_DEG.get(j, 0.0))
                for j in self.RETREAT_DEG
            )
            if max_diff <= 3.0:
                self._converge_ticks += 1
                if self._converge_ticks >= 5:
                    self._state = _STATE_HOLDING
                    self._clear_streak = 0
                    # _clear_since is NOT reset here — it was reset when we first entered
                    # RETREATING from NORMAL, so the 3-second hold timer accumulates
                    # correctly across multiple collision re-triggers during HOLDING.
                    self.get_logger().warn(
                        'Follower HOLDING — retreat pose reached, watching for obstacle clear')
            else:
                self._converge_ticks = 0

        elif self._state == _STATE_HOLDING:
            # Consult /overhead/obstacle_present ONLY while holding.
            # Any obstacle reading resets both the streak and the timer.
            if self._obstacle_present:
                self._clear_streak = 0
                self._clear_since = None
            else:
                if self._clear_since is None:
                    self._clear_since = time.monotonic()
                self._clear_streak += 1
                elapsed = time.monotonic() - self._clear_since
                if self._clear_streak >= 5 and elapsed >= 3.0:
                    self._state = _STATE_NORMAL
                    # Fire the SAME slow-step handoff used for gesture→teleop transitions
                    self._handoff_countdown = 50
                    self.get_logger().warn(
                        'Follower NORMAL — obstacle cleared (5 readings + 3 s), resuming with easing')

        # --- Effective step (handoff easing applies universally) ---
        if self._handoff_countdown > 0:
            effective_step = self._handoff_slow_step
            self._handoff_countdown -= 1
        else:
            effective_step = self.interpolation_step

        # --- LERP with per-tick degree clamp ---
        changed = False
        for joint in self.current_positions:
            target = self.goal_positions.get(joint, self.current_positions[joint])
            current = self.current_positions[joint]
            diff = target - current
            if abs(diff) > 0.1:
                step = diff * effective_step
                # Universal clamp: no joint moves more than MAX_STEP_DEG_PER_TICK per tick
                step = max(-self.MAX_STEP_DEG_PER_TICK, min(self.MAX_STEP_DEG_PER_TICK, step))
                self.current_positions[joint] += step
                changed = True

        if changed:
            try:
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
