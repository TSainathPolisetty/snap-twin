"""
Frame display node
==================
Shows a live OpenCV window on the attached monitor with:
  - Raw camera image (RGB)
  - Red bounding boxes around detected obstacle regions
  - ROI boundary rectangle (yellow)
  - "⚠ COLLISION WARNING" banner when active
  - Status text overlay (obstacle fraction, depth info)

Subscribes:
  /camera/image_raw      sensor_msgs/Image  rgb8   — raw camera frame
  /collision_mask        sensor_msgs/Image  mono8  — obstacle pixel mask
  /collision_warning     std_msgs/Bool              — collision state
  /collision_status      std_msgs/String            — human-readable status

Parameters:
  window_width   int  960   display window width  (image is scaled to fit)
  window_height  int  540   display window height
  roi_x1         float 0.15  must match collision_checker ROI params
  roi_y1         float 0.15
  roi_x2         float 0.85
  roi_y2         float 0.85
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

import numpy as np
import cv2
import array as arr
import os


class FrameDisplayNode(Node):

    def __init__(self):
        super().__init__('frame_display_node')

        self.declare_parameter('window_width',  960)
        self.declare_parameter('window_height', 540)
        self.declare_parameter('roi_x1', 0.15)
        self.declare_parameter('roi_y1', 0.15)
        self.declare_parameter('roi_x2', 0.85)
        self.declare_parameter('roi_y2', 0.85)

        self._win_w    = int(self.get_parameter('window_width').value)
        self._win_h    = int(self.get_parameter('window_height').value)
        self._roi_x1   = self.get_parameter('roi_x1').value
        self._roi_y1   = self.get_parameter('roi_y1').value
        self._roi_x2   = self.get_parameter('roi_x2').value
        self._roi_y2   = self.get_parameter('roi_y2').value

        # ── Cached latest messages ────────────────────────────────────────────
        self._frame      = None   # (H, W, 3) uint8 BGR
        self._mask       = None   # (H, W)    uint8 0/255
        self._collision  = False
        self._status_txt = 'INITIALISING...'

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Image,  '/camera/image_raw',    self._raw_cb,      5)
        self.create_subscription(Image,  '/collision_mask',      self._mask_cb,     5)
        self.create_subscription(Bool,   '/collision_warning',   self._warn_cb,     10)
        self.create_subscription(String, '/collision_status',    self._status_cb,   10)

        # ── 15 Hz display timer ───────────────────────────────────────────────
        self.create_timer(1.0 / 15.0, self._display_cb)

        # ── OpenCV window ─────────────────────────────────────────────────────
        cv2.namedWindow('Snap-Twin | Depth Collision Monitor', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Snap-Twin | Depth Collision Monitor', self._win_w, self._win_h)

        self.get_logger().info(
            f'FrameDisplayNode ready — window {self._win_w}×{self._win_h}'
        )

    # ── Message callbacks ─────────────────────────────────────────────────────

    def _raw_cb(self, msg: Image):
        # rgb8 → numpy → BGR for cv2
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self._frame = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)

    def _mask_cb(self, msg: Image):
        self._mask = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width)

    def _warn_cb(self, msg: Bool):
        self._collision = msg.data

    def _status_cb(self, msg: String):
        self._status_txt = msg.data

    # ── Display timer ─────────────────────────────────────────────────────────

    def _display_cb(self):
        if self._frame is None:
            # Show a waiting screen until first frame arrives
            canvas = np.zeros((self._win_h, self._win_w, 3), dtype=np.uint8)
            cv2.putText(canvas, 'Waiting for camera...', (30, self._win_h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 200, 200), 2)
            cv2.imshow('Snap-Twin | Depth Collision Monitor', canvas)
            cv2.waitKey(1)
            return

        canvas = self._frame.copy()
        h, w   = canvas.shape[:2]

        # ── Yellow ROI rectangle ─────────────────────────────────────────────
        rx1 = int(self._roi_x1 * w);  ry1 = int(self._roi_y1 * h)
        rx2 = int(self._roi_x2 * w);  ry2 = int(self._roi_y2 * h)
        cv2.rectangle(canvas, (rx1, ry1), (rx2, ry2), (0, 220, 220), 1)
        cv2.putText(canvas, 'Detection zone', (rx1 + 4, ry1 + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 220), 1)

        # ── Obstacle bounding boxes from mask ────────────────────────────────
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
            # Semi-transparent red banner across the top
            overlay = canvas.copy()
            cv2.rectangle(overlay, (0, 0), (w, 52), (0, 0, 180), -1)
            cv2.addWeighted(overlay, 0.55, canvas, 0.45, 0, canvas)
            cv2.putText(canvas, '\u26a0  COLLISION WARNING  \u26a0',
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
