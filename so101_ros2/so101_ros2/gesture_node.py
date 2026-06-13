"""
SO-101 Gesture / Idle Animation Node
--------------------------------------
Watches /joint_states for leader activity.
After idle_timeout seconds of no movement, runs gesture loop:

    look around -> wave -> beckoning -> excited wiggle -> repeat

Publishes:
  /gesture/joint_states  (sensor_msgs/JointState)
  /gesture_active        (std_msgs/Bool)

When teleop resumes, arm smoothly returns home then goes silent.

Parameters:
  idle_timeout  float  5.0   seconds before gesture mode activates
  publish_hz    float  20.0  gesture publish rate
  return_secs   float  2.0   seconds to smoothly return home
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from enum import Enum, auto

JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex",   "wrist_roll",    "gripper",
]

HOME_DEG = [0.0, 10.0, 60.0, 0.0, 0.0, 30.0]
MOVEMENT_THRESHOLD = 3.0   # degrees — below this = leader is idle


def build_gesture_sequence():
    H = HOME_DEG

    def h(**offsets):
        names = ["shoulder_pan", "shoulder_lift", "elbow_flex",
                 "wrist_flex",   "wrist_roll",    "gripper"]
        pos = list(H)
        for k, v in offsets.items():
            pos[names.index(k)] = H[names.index(k)] + v
        return pos

    seq = []

    # LOOK AROUND — pan ±35° from home, all other joints at home
    seq += [
        (1.2, h(shoulder_pan=-35)),
        (0.5, h(shoulder_pan=-35)),
        (1.4, h(shoulder_pan=+35)),
        (0.5, h(shoulder_pan=+35)),
        (0.9, h()),
        (0.4, H),
    ]

    # WAVE — raise arm, oscillate wrist
    seq += [
        (1.0,  h(shoulder_pan=+20, shoulder_lift=+40, elbow_flex=-20)),
        (0.25, h(shoulder_pan=+20, shoulder_lift=+40, elbow_flex=-20, wrist_flex=+30)),
        (0.25, h(shoulder_pan=+20, shoulder_lift=+40, elbow_flex=-20, wrist_flex=-25)),
        (0.25, h(shoulder_pan=+20, shoulder_lift=+40, elbow_flex=-20, wrist_flex=+30)),
        (0.25, h(shoulder_pan=+20, shoulder_lift=+40, elbow_flex=-20, wrist_flex=-25)),
        (0.25, h(shoulder_pan=+20, shoulder_lift=+40, elbow_flex=-20, wrist_flex=+30)),
        (0.25, h(shoulder_pan=+20, shoulder_lift=+40, elbow_flex=-20, wrist_flex=-25)),
        (1.0,  H),
        (0.4,  H),
    ]

    # BECKONING — reach out, curl wrist in/out
    seq += [
        (1.0, h(elbow_flex=+15, wrist_flex=-20, gripper=+10)),
        (0.4, h(elbow_flex=+15, wrist_flex=+25, gripper=-15)),
        (0.4, h(elbow_flex=+15, wrist_flex=-20, gripper=+10)),
        (0.4, h(elbow_flex=+15, wrist_flex=+25, gripper=-15)),
        (0.4, h(elbow_flex=+15, wrist_flex=-20, gripper=+10)),
        (0.4, h(elbow_flex=+15, wrist_flex=+25, gripper=-15)),
        (1.0, H),
        (0.4, H),
    ]

    # EXCITED WIGGLE — fast small oscillations on multiple joints
    seq += [
        (0.14, h(shoulder_pan=-12, shoulder_lift=+8,  wrist_flex=+10)),
        (0.14, h(shoulder_pan=+12, shoulder_lift=-8,  wrist_flex=-10)),
        (0.14, h(shoulder_pan=-10, shoulder_lift=+10, wrist_flex=+12)),
        (0.14, h(shoulder_pan=+10, shoulder_lift=-10, wrist_flex=-12)),
        (0.14, h(shoulder_pan=-12, shoulder_lift=+8,  wrist_flex=+10)),
        (0.14, h(shoulder_pan=+12, shoulder_lift=-8,  wrist_flex=-10)),
        (0.14, h(shoulder_pan=-10, shoulder_lift=+10, wrist_flex=+12)),
        (0.14, h(shoulder_pan=+10, shoulder_lift=-10, wrist_flex=-12)),
        (0.8,  H),
        (0.6,  H),
    ]

    return seq


class GestureState(Enum):
    MONITORING = auto()
    GESTURING  = auto()
    RETURNING  = auto()


def _deg_to_rad(deg_list):
    return [d * math.pi / 180.0 for d in deg_list]

def _lerp(a, b, t):
    return [a[i] + (b[i] - a[i]) * t for i in range(len(a))]

def _ease(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


class GestureNode(Node):

    def __init__(self):
        super().__init__('gesture_node')

        self.declare_parameter('idle_timeout', 5.0)
        self.declare_parameter('publish_hz',  20.0)
        self.declare_parameter('return_secs',  2.0)

        self._idle_timeout = float(self.get_parameter('idle_timeout').value)
        self._return_secs  = float(self.get_parameter('return_secs').value)
        hz                 = float(self.get_parameter('publish_hz').value)
        self._dt           = 1.0 / hz

        self._js_pub     = self.create_publisher(JointState, '/gesture/joint_states', 10)
        self._active_pub = self.create_publisher(Bool,       '/gesture_active',       10)
        self._leader_sub = self.create_subscription(
            JointState, '/joint_states', self._leader_cb, 10)

        self._state              = GestureState.MONITORING
        self._last_movement_time = self.get_clock().now()
        self._last_leader_deg    = None

        self._sequence      = build_gesture_sequence()
        self._seg_idx       = 0
        self._seg_t         = 0.0
        self._seg_start_deg = list(HOME_DEG)
        self._current_deg   = list(HOME_DEG)

        self._return_start  = list(HOME_DEG)
        self._return_t      = 0.0

        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'Gesture node ready. Idle timeout: {self._idle_timeout}s')

    def _leader_cb(self, msg: JointState):
        if not msg.position:
            return
        cur = [math.degrees(p) for p in msg.position]
        if self._last_leader_deg is not None:
            delta = max(
                abs(cur[i] - self._last_leader_deg[i])
                for i in range(min(len(cur), len(self._last_leader_deg)))
            )
            if delta > MOVEMENT_THRESHOLD:
                self._last_movement_time = self.get_clock().now()
                if self._state == GestureState.GESTURING:
                    self.get_logger().info('Teleop resumed — returning home.')
                    self._return_start = list(self._current_deg)
                    self._return_t     = 0.0
                    self._state        = GestureState.RETURNING
        self._last_leader_deg = cur

    def _tick(self):
        now     = self.get_clock().now()
        elapsed = (now - self._last_movement_time).nanoseconds / 1e9

        if self._state == GestureState.MONITORING:
            self._publish_active(False)
            if elapsed >= self._idle_timeout:
                self.get_logger().info('Idle — starting gestures.')
                self._state         = GestureState.GESTURING
                self._seg_idx       = 0
                self._seg_t         = 0.0
                self._seg_start_deg = list(self._current_deg)

        elif self._state == GestureState.GESTURING:
            self._publish_active(True)
            self._advance()
            self._publish_joints(self._current_deg)

        elif self._state == GestureState.RETURNING:
            self._publish_active(True)
            self._return_t += self._dt
            t   = _ease(self._return_t / self._return_secs)
            pos = _lerp(self._return_start, HOME_DEG, t)
            self._current_deg = pos
            self._publish_joints(pos)
            if self._return_t >= self._return_secs:
                self.get_logger().info('Home. Monitoring resumed.')
                self._current_deg = list(HOME_DEG)
                self._state       = GestureState.MONITORING
                self._publish_active(False)

    def _advance(self):
        duration, target = self._sequence[self._seg_idx]
        self._seg_t += self._dt
        t = _ease(self._seg_t / duration)
        self._current_deg = _lerp(self._seg_start_deg, target, t)
        if self._seg_t >= duration:
            self._seg_start_deg = list(target)
            self._seg_idx       = (self._seg_idx + 1) % len(self._sequence)
            self._seg_t         = 0.0

    def _publish_joints(self, deg_list):
        msg              = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name         = JOINT_NAMES
        msg.position     = _deg_to_rad(deg_list)
        self._js_pub.publish(msg)

    def _publish_active(self, active: bool):
        self._active_pub.publish(Bool(data=active))


def main(args=None):
    rclpy.init(args=args)
    node = GestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
