"""
Collision checker node
=======================
Subscribes to /camera/depth/image_raw (32FC1, values in [0,1], higher = closer).

Calibration phase:
  Collects `calib_frames` depth frames from the workspace ROI and builds a
  per-pixel median background map. Runs once on startup.

Runtime phase (10 Hz timer):
  Compares each new depth ROI to an adaptive background. A pixel is an obstacle when:
    depth_pixel > background_pixel + depth_change_threshold
  Before computing the obstacle fraction, /arm_self_mask pixels are excluded so the
  arm's own body is never counted as an obstacle. Run arm_self_mask_node to supply
  the mask; if unavailable, an all-zero mask is assumed (no arm exclusion).
  If the fraction of obstacle pixels (after arm exclusion) exceeds obstacle_area_fraction
  for 2 consecutive checks -> collision. When no collision is active, the background
  slowly adapts toward the current depth (EMA, alpha=0.97).

Publishes:
  /collision_warning   std_msgs/Bool         (True = stop/retreat)
  /collision_status    std_msgs/String       (human-readable status)
  /obstacle_markers    visualization_msgs/MarkerArray  (red sphere in Foxglove)
  /collision_mask      sensor_msgs/Image     (mono8, obstacle pixels white)

Parameters (all overridable via --ros-args -p name:=value):
  roi_x1                 float  0.15   ROI left   (normalised 0-1)
  roi_y1                 float  0.15   ROI top    (normalised 0-1)
  roi_x2                 float  0.85   ROI right  (normalised 0-1)
  roi_y2                 float  0.85   ROI bottom (normalised 0-1)
  depth_change_threshold float  0.30   min depth delta to classify as obstacle
  obstacle_area_fraction float  0.15   min fraction of ROI pixels to trigger (15% = large obstacle)
  calib_frames           int    30     frames to collect for initial background median
  background_alpha       float  0.97   EMA alpha for adaptive background (0-1, higher=slower adapt)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

import numpy as np
import cv2

try:
    from visualization_msgs.msg import MarkerArray, Marker
    _HAS_VIS = True
except ImportError:
    _HAS_VIS = False


class CollisionCheckerNode(Node):

    def __init__(self):
        super().__init__('collision_checker')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('roi_x1',                 0.15)
        self.declare_parameter('roi_y1',                 0.15)
        self.declare_parameter('roi_x2',                 0.85)
        self.declare_parameter('roi_y2',                 0.85)
        self.declare_parameter('depth_change_threshold', 0.30)
        self.declare_parameter('obstacle_area_fraction', 0.15)
        self.declare_parameter('calib_frames',           30)
        # Background adapts slowly toward current depth when no collision is active.
        # At 10Hz: alpha=0.97 -> half-life ~23s
        self.declare_parameter('background_alpha',       0.97)

        self._roi_x1    = self.get_parameter('roi_x1').value
        self._roi_y1    = self.get_parameter('roi_y1').value
        self._roi_x2    = self.get_parameter('roi_x2').value
        self._roi_y2    = self.get_parameter('roi_y2').value
        self._threshold = self.get_parameter('depth_change_threshold').value
        self._area_frac = self.get_parameter('obstacle_area_fraction').value
        self._n_calib   = int(self.get_parameter('calib_frames').value)
        self._bg_alpha  = float(self.get_parameter('background_alpha').value)

        # ── Internal state ───────────────────────────────────────────────────
        self._calib_stack      = []      # list of ROI arrays during calibration
        self._background       = None   # (H_roi, W_roi) float32 median map
        self._latest_depth     = None   # most recent full-frame depth array
        self._latest_self_mask = None   # most recent /arm_self_mask (518x518 uint8)
        self._collision        = False  # current published state
        self._obstacle_streak  = 0      # consecutive checks above threshold (debounce)
        self._warmup_remaining = 50     # post-calibration adaptation cycles before detection

        # ── Publishers - VOLATILE so no stale True persists after restart ────
        _volatile_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10)
        self._warn_pub   = self.create_publisher(Bool,   '/collision_warning', _volatile_qos)
        self._status_pub = self.create_publisher(String, '/collision_status',  _volatile_qos)
        self._mask_pub   = self.create_publisher(Image,  '/collision_mask',    5)

        if _HAS_VIS:
            self._marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        else:
            self._marker_pub = None
            self.get_logger().warn('visualization_msgs not available - /obstacle_markers disabled')

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self._depth_cb, 5)
        # Arm self-mask: arm pixels to EXCLUDE from obstacle detection.
        # Supplied by arm_self_mask_node. If not available, an all-zero mask is assumed.
        self.create_subscription(Image, '/arm_self_mask',
                                 self._self_mask_cb, 5)

        # ── 10 Hz check timer ────────────────────────────────────────────────
        self.create_timer(0.1, self._check_cb)

        self.get_logger().info(
            f'CollisionChecker started - collecting {self._n_calib} background frames …'
        )

    # ────────────────────────────────────────────────────────────────────────
    # Depth subscription
    # ────────────────────────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image):
        if msg.encoding != '32FC1':
            self.get_logger().warn(
                f'Expected 32FC1 depth image, got {msg.encoding}', throttle_duration_sec=10.0)
            return

        depth = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
        self._latest_depth = depth

        # Accumulate calibration frames until we have enough
        if self._background is None and len(self._calib_stack) < self._n_calib:
            roi = self._extract_roi(depth)
            self._calib_stack.append(roi.copy())
            remaining = self._n_calib - len(self._calib_stack)
            if remaining % 10 == 0 and remaining > 0:
                self.get_logger().info(
                    f'Background calibration: {len(self._calib_stack)}/{self._n_calib} frames …')
            if len(self._calib_stack) == self._n_calib:
                self._build_background()

    # ────────────────────────────────────────────────────────────────────────
    # Arm self-mask subscription
    # ────────────────────────────────────────────────────────────────────────

    def _self_mask_cb(self, msg: Image):
        if msg.encoding != 'mono8':
            self.get_logger().warn(
                f'Expected mono8 self-mask, got {msg.encoding}', throttle_duration_sec=10.0)
            return
        self._latest_self_mask = np.frombuffer(
            bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width)

    # ────────────────────────────────────────────────────────────────────────
    # Background model
    # ────────────────────────────────────────────────────────────────────────

    def _extract_roi(self, depth: np.ndarray) -> np.ndarray:
        h, w = depth.shape
        y1 = int(self._roi_y1 * h)
        y2 = int(self._roi_y2 * h)
        x1 = int(self._roi_x1 * w)
        x2 = int(self._roi_x2 * w)
        return depth[y1:y2, x1:x2]

    def _build_background(self):
        stack = np.stack(self._calib_stack, axis=0)   # (N, H_roi, W_roi)
        self._background = np.median(stack, axis=0).astype(np.float32)
        self._calib_stack.clear()
        self.get_logger().info(
            f'Background calibrated - ROI shape {self._background.shape}, '
            f'mean depth {self._background.mean():.3f}'
        )

    # ────────────────────────────────────────────────────────────────────────
    # 10 Hz check timer
    # ────────────────────────────────────────────────────────────────────────

    def _check_cb(self):
        if self._background is None:
            n = len(self._calib_stack)
            self._publish_status(False, f'CALIBRATING ({n}/{self._n_calib} frames)')
            return

        if self._latest_depth is None:
            self._publish_status(False, 'WAITING - no depth frame received')
            return

        roi = self._extract_roi(self._latest_depth)

        if roi.shape != self._background.shape:
            self.get_logger().warn(
                f'ROI shape mismatch {roi.shape} vs bg {self._background.shape} - skipping',
                throttle_duration_sec=5.0)
            return

        # Warmup: keep adapting background, no collision reporting yet
        if self._warmup_remaining > 0:
            self._warmup_remaining -= 1
            self._background = (self._bg_alpha * self._background
                                + (1.0 - self._bg_alpha) * roi)
            self._obstacle_streak = 0
            self._publish_status(False, f'WARMING ({self._warmup_remaining} cycles left)')
            return

        # Raw obstacle mask: pixels CLOSER than background by more than threshold
        obstacle_mask = roi > (self._background + self._threshold)

        # ── Arm self-mask exclusion ──────────────────────────────────────────
        # Crop /arm_self_mask to the same ROI region, then exclude arm pixels.
        # This prevents the arm's own body from ever being counted as an obstacle.
        if self._latest_self_mask is not None:
            sm = self._latest_self_mask
            h_sm, w_sm = sm.shape
            sy1 = int(self._roi_y1 * h_sm)
            sy2 = int(self._roi_y2 * h_sm)
            sx1 = int(self._roi_x1 * w_sm)
            sx2 = int(self._roi_x2 * w_sm)
            self_mask_roi = sm[sy1:sy2, sx1:sx2]
            # Resize to match depth ROI dimensions if needed
            if self_mask_roi.shape != obstacle_mask.shape:
                self_mask_roi = cv2.resize(
                    self_mask_roi,
                    (obstacle_mask.shape[1], obstacle_mask.shape[0]),
                    interpolation=cv2.INTER_NEAREST)
            # Exclude arm pixels from obstacle detection
            obstacle_mask = obstacle_mask & (self_mask_roi == 0)

        obstacle_frac = float(obstacle_mask.mean())
        raw_collision = obstacle_frac > self._area_frac

        # 2-check debounce to reject single-frame noise
        if raw_collision:
            self._obstacle_streak += 1
        else:
            self._obstacle_streak = 0
        collision_now = self._obstacle_streak >= 2

        # Adapt background toward current depth only when no active collision
        if not collision_now:
            self._background = (self._bg_alpha * self._background
                                + (1.0 - self._bg_alpha) * roi)

        if collision_now:
            status = f'COLLISION - {obstacle_frac*100:.1f}% of ROI obstructed'
        else:
            status = f'CLEAR - {obstacle_frac*100:.2f}% obstacle fraction'

        self._publish_status(collision_now, status)
        self._publish_markers(collision_now)
        self._publish_mask(obstacle_mask, roi.shape)

    # ────────────────────────────────────────────────────────────────────────
    # Publishing helpers
    # ────────────────────────────────────────────────────────────────────────

    def _publish_status(self, collision: bool, status_text: str):
        if collision != self._collision:
            self._collision = collision
            self.get_logger().warn(f'Collision state changed: {status_text}')

        warn_msg      = Bool()
        warn_msg.data = collision
        self._warn_pub.publish(warn_msg)

        status_msg      = String()
        status_msg.data = status_text
        self._status_pub.publish(status_msg)

    def _publish_markers(self, collision: bool):
        if self._marker_pub is None:
            return

        array = MarkerArray()

        if collision:
            m               = Marker()
            m.header.stamp  = self.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            m.ns            = 'collision'
            m.id            = 0
            m.type          = Marker.SPHERE
            m.action        = Marker.ADD
            # Fixed position ~30 cm in front of the arm base, ~20 cm height
            m.pose.position.x = 0.30
            m.pose.position.y = 0.0
            m.pose.position.z = 0.20
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12   # 12 cm sphere
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 0.8
            # Marker auto-expires after 1 second (prevents stale markers)
            m.lifetime.sec     = 1
            m.lifetime.nanosec = 0
            array.markers.append(m)
        else:
            # Send a DELETE marker to immediately clear when collision clears
            m         = Marker()
            m.ns      = 'collision'
            m.id      = 0
            m.action  = Marker.DELETE
            array.markers.append(m)

        self._marker_pub.publish(array)

    def _publish_mask(self, obstacle_mask: np.ndarray, roi_shape: tuple):
        """Publish mono8 obstacle mask (white=obstacle) padded to 518×518."""
        import array as arr
        # Build a full-frame mask the same size as the depth image (518×518)
        h_full, w_full = 518, 518
        full_mask = np.zeros((h_full, w_full), dtype=np.uint8)
        # Place the ROI mask back into the full frame
        h_roi, w_roi = roi_shape
        y1 = int(self._roi_y1 * h_full)
        x1 = int(self._roi_x1 * w_full)
        full_mask[y1:y1+h_roi, x1:x1+w_roi] = obstacle_mask.astype(np.uint8) * 255

        now = self.get_clock().now().to_msg()
        mask_msg = Image()
        mask_msg.header.stamp    = now
        mask_msg.header.frame_id = 'camera_frame'
        mask_msg.height          = h_full
        mask_msg.width           = w_full
        mask_msg.encoding        = 'mono8'
        mask_msg.is_bigendian    = False
        mask_msg.step            = w_full
        mask_msg.data            = arr.array('B', full_mask.tobytes())
        self._mask_pub.publish(mask_msg)


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
