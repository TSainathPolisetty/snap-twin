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
  If the fraction of such pixels exceeds obstacle_area_fraction for 2 consecutive
  checks → collision. When no collision is active, the background slowly adapts toward
  the current depth (EMA, alpha=0.97) so gradual arm movement is absorbed without
  losing sensitivity to sudden new objects.

Publishes:
  /collision_warning   std_msgs/Bool         (True = stop/retreat)
  /collision_status    std_msgs/String       (human-readable status)
  /obstacle_markers    visualization_msgs/MarkerArray  (red sphere in Foxglove)

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

try:
    from visualization_msgs.msg import MarkerArray, Marker
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA
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
        # At 10Hz: alpha=0.97 → half-life ~23s (absorbs gradual arm movement)
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
        self._calib_stack   = []          # list of ROI np arrays during calibration
        self._background    = None        # (H_roi, W_roi) float32 median map, adapts over time
        self._latest_depth  = None        # most recent full-frame depth array
        self._collision     = False       # current published state
        self._obstacle_streak = 0         # consecutive checks above threshold (for debounce)
        self._warmup_remaining = 50       # post-calibration adaptation cycles before detection
        self._arm_moving    = False       # suppress detection while arm is moving
        self._last_js_positions = None    # previous /joint_states positions for velocity check

        # ── Publishers — VOLATILE so no stale True message persists after restart ──
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
            self.get_logger().warn('visualization_msgs not available — /obstacle_markers disabled')

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self._depth_cb, 5)
        # Subscribe to follower joint states to detect arm movement.
        # When the arm is moving, depth changes are expected — suppress collision detection.
        from sensor_msgs.msg import JointState
        self.create_subscription(JointState, '/joint_states',
                                 self._joint_states_cb, 10)
        # Suppress detection when gesture animation is running (arm actively animating)
        from std_msgs.msg import Bool as BoolMsg
        self.create_subscription(BoolMsg, '/gesture_active',
                                 lambda msg: setattr(self, '_gesture_active', msg.data), 10)
        self._gesture_active = False

        # ── 10 Hz check timer ────────────────────────────────────────────────
        self.create_timer(0.1, self._check_cb)

        self.get_logger().info(
            f'CollisionChecker started — collecting {self._n_calib} background frames …'
        )

    # ────────────────────────────────────────────────────────────────────────
    # Depth subscription — just decode and cache the latest frame
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
    # Joint states subscription — detect arm movement to suppress false alarms
    # ────────────────────────────────────────────────────────────────────────

    def _joint_states_cb(self, msg):
        """Track whether the arm is actively moving. Suppress collision detection
        when the arm moves to avoid false alarms from the arm itself changing depth."""
        positions = list(msg.position)
        if self._last_js_positions is None:
            self._last_js_positions = positions
            return
        # Check max absolute change across all joints (in radians)
        max_delta = max(abs(a - b) for a, b in zip(positions, self._last_js_positions))
        self._arm_moving = max_delta > 0.01   # ~0.6 degrees change = arm is moving
        self._last_js_positions = positions

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
        self._calib_stack.clear()   # free memory
        self.get_logger().info(
            f'Background calibrated — ROI shape {self._background.shape}, '
            f'mean depth {self._background.mean():.3f}'
        )

    # ────────────────────────────────────────────────────────────────────────
    # 10 Hz check timer
    # ────────────────────────────────────────────────────────────────────────

    def _check_cb(self):
        # Still calibrating
        if self._background is None:
            n = len(self._calib_stack)
            status = f'CALIBRATING ({n}/{self._n_calib} frames)'
            self._publish_status(False, status)
            return

        if self._latest_depth is None:
            status = 'WAITING — no depth frame received'
            self._publish_status(False, status)
            return

        roi       = self._extract_roi(self._latest_depth)

        # Safety: re-align shapes if resolution changed after calibration
        if roi.shape != self._background.shape:
            self.get_logger().warn(
                f'ROI shape mismatch {roi.shape} vs bg {self._background.shape} — skipping',
                throttle_duration_sec=5.0)
            return

        # Always adapt background toward current depth when not in confirmed collision.
        # During the warmup period, always adapt and never report collision.
        if self._warmup_remaining > 0:
            self._warmup_remaining -= 1
            self._background = (self._bg_alpha * self._background
                                + (1.0 - self._bg_alpha) * roi)
            self._obstacle_streak = 0
            status = f'WARMING ({self._warmup_remaining} cycles left)'
            self._publish_status(False, status)
            return

        # When the arm is moving (leader OR gesture active), depth changes are expected from the arm.
        # Suppress collision detection and update background to track arm movement.
        arm_active = self._arm_moving or self._gesture_active
        if arm_active:
            self._background = (self._bg_alpha * self._background
                                + (1.0 - self._bg_alpha) * roi)
            self._obstacle_streak = 0
            status = 'CLEAR — arm moving (suppressed)'
            self._publish_status(False, status)
            return

        # Obstacle mask: pixels CLOSER than background by more than threshold
        obstacle_mask   = roi > (self._background + self._threshold)
        obstacle_frac   = float(obstacle_mask.mean())
        raw_collision   = obstacle_frac > self._area_frac

        # Require 2 consecutive detections to confirm collision (debounce noise)
        if raw_collision:
            self._obstacle_streak += 1
        else:
            self._obstacle_streak = 0
        collision_now = self._obstacle_streak >= 2

        # Adapt background toward current depth ONLY when no collision is active.
        # This absorbs gradual arm movement while staying sensitive to sudden objects.
        if not collision_now:
            self._background = (self._bg_alpha * self._background
                                + (1.0 - self._bg_alpha) * roi)

        if collision_now:
            status = f'COLLISION — {obstacle_frac*100:.1f}% of ROI obstructed'
        else:
            status = f'CLEAR — {obstacle_frac*100:.2f}% obstacle fraction'

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
