"""
Wrist-jaw collision checker.

Subscribes to /camera/depth/image_raw (32FC1, expected 518x518) from the wrist
camera. At 10 Hz it compares the jaw-region median depth against the 95th
percentile of the rest of the frame. If the gap collapses for two consecutive
checks, a collision warning is published.

Publishes:
  /collision_warning  std_msgs/Bool    VOLATILE QoS
  /collision_status   std_msgs/String
  /collision_mask     sensor_msgs/Image mono8 (jaw rectangle highlight)

Parameters:
  jaw_x1            float 0.30
  jaw_y1            float 0.65
  jaw_x2            float 0.70
  jaw_y2            float 1.00
  delta_threshold   float 0.45  collision threshold (live hardware: at-rest delta ~0.575-0.604, hand-in-jaw ~0.35-0.47)
  streak_required   int   2
"""

import array

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class CollisionCheckerNode(Node):

    def __init__(self):
        super().__init__('collision_checker')

        self.declare_parameter('jaw_x1', 0.30)
        self.declare_parameter('jaw_y1', 0.65)
        self.declare_parameter('jaw_x2', 0.70)
        self.declare_parameter('jaw_y2', 1.00)
        self.declare_parameter('delta_threshold', 0.45)  # live hardware: at-rest delta ~0.575-0.604, hand-in-jaw ~0.35-0.47
        self.declare_parameter('streak_required', 2)

        self._jaw_x1 = float(self.get_parameter('jaw_x1').value)
        self._jaw_y1 = float(self.get_parameter('jaw_y1').value)
        self._jaw_x2 = float(self.get_parameter('jaw_x2').value)
        self._jaw_y2 = float(self.get_parameter('jaw_y2').value)
        self._delta_threshold = float(self.get_parameter('delta_threshold').value)
        self._streak_required = max(1, int(self.get_parameter('streak_required').value))

        self._latest_depth = None
        self._obstacle_streak = 0
        self._collision_now = False

        volatile_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )
        self._warning_pub = self.create_publisher(Bool, '/collision_warning', volatile_qos)
        self._status_pub = self.create_publisher(String, '/collision_status', 10)
        self._mask_pub = self.create_publisher(Image, '/collision_mask', 5)

        self.create_subscription(Image, '/camera/depth/image_raw', self._depth_cb, 5)
        self.create_timer(0.1, self._timer_cb)

        self.get_logger().info('CollisionCheckerNode ready - waiting for wrist depth frames')

    def _depth_cb(self, msg: Image):
        if msg.encoding != '32FC1':
            self.get_logger().warn(
                f'Expected 32FC1 depth image, got {msg.encoding}',
                throttle_duration_sec=10.0,
            )
            return
        self._latest_depth = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)

    def _timer_cb(self):
        if self._latest_depth is None:
            self.get_logger().warn(
                'No wrist depth frame received yet - collision checker waiting',
                throttle_duration_sec=5.0,
            )
            self._publish_warning(False)
            self._publish_status('WAITING - no wrist depth frame received')
            return

        depth = self._latest_depth
        h, w = depth.shape
        x1 = int(self._jaw_x1 * w)
        y1 = int(self._jaw_y1 * h)
        x2 = int(self._jaw_x2 * w)
        y2 = int(self._jaw_y2 * h)
        x2 = max(x1 + 1, min(w, x2))
        y2 = max(y1 + 1, min(h, y2))

        jaw = depth[y1:y2, x1:x2]
        jaw_valid = jaw[np.isfinite(jaw) & (jaw > 0.0)]

        rest_mask = np.ones(depth.shape, dtype=bool)
        rest_mask[y1:y2, x1:x2] = False
        rest_valid = depth[rest_mask]
        rest_valid = rest_valid[np.isfinite(rest_valid) & (rest_valid > 0.0)]

        if jaw_valid.size == 0 or rest_valid.size == 0:
            self.get_logger().warn(
                'Depth frame missing valid jaw/rest samples - skipping collision check',
                throttle_duration_sec=5.0,
            )
            self._obstacle_streak = 0
            self._collision_now = False
            self._publish_warning(False)
            self._publish_status('WAITING - insufficient valid wrist depth samples')
            self._publish_mask(h, w, x1, y1, x2, y2, False, active=False)
            return

        jaw_median = float(np.median(jaw_valid))
        rest_p95 = float(np.percentile(rest_valid, 95))
        delta = rest_p95 - jaw_median
        raw_trigger = delta < self._delta_threshold

        if raw_trigger:
            self._obstacle_streak += 1
        else:
            self._obstacle_streak = 0

        self._collision_now = self._obstacle_streak >= self._streak_required

        if self._collision_now:
            status = (
                f'COLLISION delta={delta:.3f} '
                f'jaw={jaw_median:.3f} rest={rest_p95:.3f}'
            )
        else:
            status = (
                f'CLEAR delta={delta:.3f} '
                f'jaw={jaw_median:.3f} rest={rest_p95:.3f}'
            )

        self._publish_warning(self._collision_now)
        self._publish_status(status)
        self._publish_mask(h, w, x1, y1, x2, y2, self._collision_now, active=True)

    def _publish_warning(self, collision_now: bool):
        msg = Bool()
        msg.data = collision_now
        self._warning_pub.publish(msg)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _publish_mask(self, h: int, w: int, x1: int, y1: int, x2: int, y2: int,
                      collision_now: bool, active: bool):
        mask = np.zeros((h, w), dtype=np.uint8)
        if active:
            mask[y1:y2, x1:x2] = 255 if collision_now else 128

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.height = h
        msg.width = w
        msg.encoding = 'mono8'
        msg.is_bigendian = False
        msg.step = w
        msg.data = array.array('B', mask.tobytes())
        self._mask_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
