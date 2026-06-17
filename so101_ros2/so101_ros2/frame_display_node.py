"""
Frame display node
==================
Shows a live OpenCV window on the attached monitor with:
  - Raw camera image (RGB) OR colorized depth visualization, flipping every 7 seconds
  - Orange contour of arm self-mask pixels (from /arm_self_mask)
  - Red bounding boxes around detected obstacle regions (from /collision_mask)
  - Yellow ROI boundary rectangle
  - "! COLLISION WARNING !" banner when active (ASCII-safe, no Unicode)
  - Status text overlay (obstacle fraction, depth info)

Subscribes:
  /camera/image_raw           sensor_msgs/Image  rgb8   -- raw camera frame (518x518)
  /camera/depth/visualization sensor_msgs/Image  rgb8   -- colorized depth map
  /collision_mask             sensor_msgs/Image  mono8  -- obstacle pixel mask
  /arm_self_mask              sensor_msgs/Image  mono8  -- arm pixel mask
  /collision_warning          std_msgs/Bool              -- collision state
  /collision_status           std_msgs/String            -- human-readable status

Parameters:
  window_width   int  960   display window width  (image is scaled to fit)
  window_height  int  540   display window height
  roi_x1         float 0.15  must match collision_checker ROI params
  roi_y1         float 0.15
  roi_x2         float 0.85
  roi_y2         float 0.85
  panel_flip_secs float 7.0  seconds between raw/depth panel switches
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

import numpy as np
import cv2
import os


class FrameDisplayNode(Node):

    def __init__(self):
        super().__init__('frame_display_node')

        self.declare_parameter('window_width',    960)
        self.declare_parameter('window_height',   540)
        self.declare_parameter('roi_x1',          0.15)
        self.declare_parameter('roi_y1',          0.15)
        self.declare_parameter('roi_x2',          0.85)
        self.declare_parameter('roi_y2',          0.85)
        self.declare_parameter('panel_flip_secs', 7.0)

        self._win_w          = int(self.get_parameter('window_width').value)
        self._win_h          = int(self.get_parameter('window_height').value)
        self._roi_x1         = self.get_parameter('roi_x1').value
        self._roi_y1         = self.get_parameter('roi_y1').value
        self._roi_x2         = self.get_parameter('roi_x2').value
        self._roi_y2         = self.get_parameter('roi_y2').value
        flip_secs            = float(self.get_parameter('panel_flip_secs').value)

        # ── Cached latest messages ────────────────────────────────────────────
        self._raw_frame   = None   # (H, W, 3) uint8 BGR -- raw camera frame
        self._depth_frame = None   # (H, W, 3) uint8 BGR -- colorized depth
        self._mask        = None   # (H, W)    uint8 0/255 -- obstacle pixels
        self._self_mask   = None   # (H, W)    uint8 0/255 -- arm pixels
        self._collision   = False
        self._status_txt  = 'INITIALISING...'

        # ── Panel mode: 'raw' or 'depth', flips every panel_flip_secs ────────
        self._panel_mode    = 'raw'
        self._display_ticks = 0
        self._flip_every    = max(1, int(round(flip_secs * 15.0)))  # 15Hz timer ticks

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Image,  '/camera/image_raw',            self._raw_cb,      5)
        self.create_subscription(Image,  '/camera/depth/visualization',   self._depth_cb,    5)
        self.create_subscription(Image,  '/collision_mask',               self._mask_cb,     5)
        self.create_subscription(Image,  '/arm_self_mask',                self._self_mask_cb,5)
        self.create_subscription(Bool,   '/collision_warning',            self._warn_cb,     10)
        self.create_subscription(String, '/collision_status',             self._status_cb,   10)

        # ── 15 Hz display timer ───────────────────────────────────────────────
        self.create_timer(1.0 / 15.0, self._display_cb)

        # ── OpenCV window ─────────────────────────────────────────────────────
        cv2.namedWindow('Snap-Twin | Depth Collision Monitor', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Snap-Twin | Depth Collision Monitor', self._win_w, self._win_h)

        self.get_logger().info(
            f'FrameDisplayNode ready - window {self._win_w}x{self._win_h}, '
            f'panel flip every {flip_secs:.1f}s'
        )

    # ── Message callbacks ─────────────────────────────────────────────────────

    def _raw_cb(self, msg: Image):
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self._raw_frame = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)

    def _depth_cb(self, msg: Image):
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self._depth_frame = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)

    def _mask_cb(self, msg: Image):
        self._mask = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width)

    def _self_mask_cb(self, msg: Image):
        self._self_mask = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width)

    def _warn_cb(self, msg: Bool):
        self._collision = msg.data

    def _status_cb(self, msg: String):
        self._status_txt = msg.data

    # ── Display timer ─────────────────────────────────────────────────────────

    def _display_cb(self):
        # ── Panel mode flip every _flip_every ticks ───────────────────────────
        self._display_ticks += 1
        if self._display_ticks % self._flip_every == 0:
            self._panel_mode = 'depth' if self._panel_mode == 'raw' else 'raw'

        # ── Select background panel ───────────────────────────────────────────
        if self._panel_mode == 'depth' and self._depth_frame is not None:
            base_img = self._depth_frame
        elif self._raw_frame is not None:
            base_img = self._raw_frame
        else:
            # Nothing received yet
            canvas = np.zeros((self._win_h, self._win_w, 3), dtype=np.uint8)
            label = 'Waiting for camera...'
            cv2.putText(canvas, label, (30, self._win_h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)
            cv2.imshow('Snap-Twin | Depth Collision Monitor', canvas)
            cv2.waitKey(1)
            return

        canvas = base_img.copy()
        h, w   = canvas.shape[:2]

        # ── Panel label (top-right, small) ────────────────────────────────────
        panel_label = 'DEPTH' if self._panel_mode == 'depth' else 'RAW'
        cv2.putText(canvas, panel_label, (w - 80, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 0), 1)

        # ── Yellow ROI rectangle ─────────────────────────────────────────────
        rx1 = int(self._roi_x1 * w);  ry1 = int(self._roi_y1 * h)
        rx2 = int(self._roi_x2 * w);  ry2 = int(self._roi_y2 * h)
        cv2.rectangle(canvas, (rx1, ry1), (rx2, ry2), (0, 220, 220), 1)
        cv2.putText(canvas, 'Detection zone', (rx1 + 4, ry1 + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 220), 1)

        # ── Orange arm self-mask contour ──────────────────────────────────────
        if self._self_mask is not None:
            sm_rs = cv2.resize(self._self_mask, (w, h), interpolation=cv2.INTER_NEAREST)
            sm_contours, _ = cv2.findContours(sm_rs, cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_SIMPLE)
            for cnt in sm_contours:
                if cv2.contourArea(cnt) < 50:
                    continue
                cv2.drawContours(canvas, [cnt], -1, (0, 140, 255), 2)  # orange BGR

        # ── Red obstacle bounding boxes from collision mask ───────────────────
        if self._mask is not None:
            mask_rs = cv2.resize(self._mask, (w, h), interpolation=cv2.INTER_NEAREST)
            contours, _ = cv2.findContours(mask_rs, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) < 100:   # ignore tiny noise blobs
                    continue
                bx, by, bw, bh = cv2.boundingRect(cnt)
                cv2.rectangle(canvas, (bx, by), (bx + bw, by + bh), (0, 0, 255), 2)
                cv2.putText(canvas, 'OBSTACLE', (bx, by - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # ── Collision warning banner ─────────────────────────────────────────
        if self._collision:
            overlay = canvas.copy()
            cv2.rectangle(overlay, (0, 0), (w, 52), (0, 0, 180), -1)
            cv2.addWeighted(overlay, 0.55, canvas, 0.45, 0, canvas)
            cv2.putText(canvas, '!  COLLISION WARNING  !',
                        (12, 36), cv2.FONT_HERSHEY_DUPLEX, 1.05,
                        (255, 255, 255), 2)

        # ── Status text (bottom bar) ─────────────────────────────────────────
        status_color = (0, 60, 255) if self._collision else (50, 200, 50)
        cv2.rectangle(canvas, (0, h - 28), (w, h), (20, 20, 20), -1)
        cv2.putText(canvas, self._status_txt, (8, h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)

        # ── Scale to window and show ─────────────────────────────────────────
        display = cv2.resize(canvas, (self._win_w, self._win_h),
                             interpolation=cv2.INTER_LINEAR)
        cv2.imshow('Snap-Twin | Depth Collision Monitor', display)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    # Ensure DISPLAY is set for a headed session
    if 'DISPLAY' not in os.environ:
        os.environ['DISPLAY'] = ':0'

    rclpy.init(args=args)
    node = FrameDisplayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
