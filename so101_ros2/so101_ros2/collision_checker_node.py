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
  delta_threshold   float 0.30  collision threshold (live hardware: at-rest delta ~0.575-0.604, hand-in-jaw ~0.35-0.47)
  streak_required   int   2
"""

import array

import cv2
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
        self.declare_parameter('delta_threshold', 0.30)  # live hardware: at-rest delta ~0.575-0.604, hand-in-jaw ~0.35-0.47
        self.declare_parameter('streak_required', 2)
        self.declare_parameter('mask_margin', 0.05)  # per-pixel: flag jaw pixels anomalously closer than jaw_bg_ref
        self.declare_parameter('jaw_bg_alpha', 0.02)  # EMA alpha for jaw background reference adaptation

        self._jaw_x1 = float(self.get_parameter('jaw_x1').value)
        self._jaw_y1 = float(self.get_parameter('jaw_y1').value)
        self._jaw_x2 = float(self.get_parameter('jaw_x2').value)
        self._jaw_y2 = float(self.get_parameter('jaw_y2').value)
        self._delta_threshold = float(self.get_parameter('delta_threshold').value)
        self._streak_required = max(1, int(self.get_parameter('streak_required').value))
        self._mask_margin = float(self.get_parameter('mask_margin').value)
        self._jaw_bg_alpha = float(self.get_parameter('jaw_bg_alpha').value)

        self._latest_depth = None
        self._obstacle_streak = 0
        self._collision_now = False
        # Jaw-internal adaptive background reference. Learns what the gripper's own
        # resting depth profile looks like at each pixel position, so the fingers
        # themselves never register as anomalous in the per-pixel visualization mask.
        # Only adapts when the aggregate collision test is CLEAR (not _collision_now),
        # so a real obstacle inside the jaw cannot be absorbed into "normal."
        self._jaw_bg_ref = None  # float32, same shape as jaw crop

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
            self._publish_mask(h, w, x1, y1, x2, y2, np.empty(0), 0.0, active=False)
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

        # Update jaw background reference — adapts only when CLEAR so a real obstacle
        # inside the jaw is never absorbed into the reference.
        jaw_float = jaw.astype(np.float32)
        if self._jaw_bg_ref is None or self._jaw_bg_ref.shape != jaw.shape:
            self._jaw_bg_ref = jaw_float.copy()
        elif not self._collision_now:
            cv2.accumulateWeighted(jaw_float, self._jaw_bg_ref, self._jaw_bg_alpha)

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
        self._publish_mask(h, w, x1, y1, x2, y2, jaw, rest_p95, active=True)

    def _publish_warning(self, collision_now: bool):
        msg = Bool()
        msg.data = collision_now
        self._warning_pub.publish(msg)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)

    def _publish_mask(self, h: int, w: int, x1: int, y1: int, x2: int, y2: int,
                      jaw: np.ndarray, rest_p95: float, active: bool):
        """Publish per-pixel collision mask using jaw-adaptive background reference.

        Each jaw pixel is flagged 255 when it is significantly CLOSER to the camera
        than the jaw's own slowly-adapting background reference (self._jaw_bg_ref).
        This means the gripper's own resting finger depth is exactly what the reference
        adapts to, so the fingers never register as anomalous — only something NEW and
        closer entering the jaw region does.

        The old rest_p95-based threshold is no longer used for the mask (rest_p95 is
        still passed for call-site compatibility but is ignored here). The aggregate
        streak-based /collision_warning logic is unchanged — this mask is purely for
        visualization.
        """
        mask = np.zeros((h, w), dtype=np.uint8)
        if (active and jaw.size > 0
                and self._jaw_bg_ref is not None
                and self._jaw_bg_ref.shape == jaw.shape):
            # pixel_delta > 0 means closer to camera than the reference (anomalously close)
            pixel_delta = jaw.astype(np.float32) - self._jaw_bg_ref
            close_pixels = (pixel_delta > self._mask_margin) & np.isfinite(pixel_delta)
            jaw_mask_region = np.zeros_like(jaw, dtype=np.uint8)
            jaw_mask_region[close_pixels] = 255
            mask[y1:y2, x1:x2] = jaw_mask_region

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'wrist_link'
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
