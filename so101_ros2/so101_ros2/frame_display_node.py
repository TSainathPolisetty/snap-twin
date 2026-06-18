"""
Split-view frame display node.

Shows overhead annotated RGB on the left and wrist depth RGB on the right at the
same time, with a shared bottom collision-status banner.
"""

import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

WINDOW_NAME = 'Snap-Twin | Overhead + Wrist Monitor'
BANNER_HEIGHT = 48


class FrameDisplayNode(Node):

    def __init__(self):
        super().__init__('frame_display_node')

        self.declare_parameter('window_width', 1280)
        self.declare_parameter('window_height', 480)
        self.declare_parameter('jaw_x1', 0.30)
        self.declare_parameter('jaw_y1', 0.65)
        self.declare_parameter('jaw_x2', 0.70)
        self.declare_parameter('jaw_y2', 1.00)

        self._win_w = int(self.get_parameter('window_width').value)
        self._win_h = int(self.get_parameter('window_height').value)
        self._jaw_x1 = float(self.get_parameter('jaw_x1').value)
        self._jaw_y1 = float(self.get_parameter('jaw_y1').value)
        self._jaw_x2 = float(self.get_parameter('jaw_x2').value)
        self._jaw_y2 = float(self.get_parameter('jaw_y2').value)

        self._overhead = None
        self._depth = None
        self._collision_mask = None
        self._collision_now = False
        self._status_text = 'INITIALISING...'

        self.create_subscription(Image, '/overhead/image_annotated', self._overhead_cb, 5)
        self.create_subscription(Image, '/camera/depth/visualization', self._depth_cb, 5)
        self.create_subscription(Image, '/collision_mask', self._mask_cb, 5)
        self.create_subscription(Bool, '/collision_warning', self._warning_cb, 10)
        self.create_subscription(String, '/collision_status', self._status_cb, 10)

        self.create_timer(1.0 / 15.0, self._display_cb)

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, self._win_w, self._win_h)
        self.get_logger().info(f'FrameDisplayNode ready - window {self._win_w}x{self._win_h}')

    def _overhead_cb(self, msg: Image):
        rgb = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self._overhead = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def _depth_cb(self, msg: Image):
        rgb = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self._depth = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def _mask_cb(self, msg: Image):
        self._collision_mask = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width)

    def _warning_cb(self, msg: Bool):
        self._collision_now = msg.data

    def _status_cb(self, msg: String):
        self._status_text = msg.data

    def _display_cb(self):
        panel_w = self._win_w // 2
        panel_h = max(1, self._win_h - BANNER_HEIGHT)

        left_panel = self._render_panel(self._overhead, panel_w, panel_h, 'OVERHEAD')
        right_panel = self._render_right_panel(panel_w, panel_h)
        banner = self._render_banner(self._win_w, BANNER_HEIGHT)

        canvas = np.vstack([np.hstack([left_panel, right_panel]), banner])
        cv2.imshow(WINDOW_NAME, canvas)
        cv2.waitKey(1)

    def _render_panel(self, frame, width: int, height: int, label: str):
        if frame is None:
            panel = np.full((height, width, 3), 90, dtype=np.uint8)
            cv2.putText(
                panel,
                f'Waiting for {label}...',
                (30, height // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (220, 220, 220),
                2,
                cv2.LINE_AA,
            )
        else:
            panel = cv2.resize(frame, (width, height), interpolation=cv2.INTER_LINEAR)
        cv2.putText(panel, label, (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        return panel

    def _render_right_panel(self, width: int, height: int):
        panel = self._render_panel(self._depth, width, height, 'WRIST DEPTH')
        jaw_x1 = int(self._jaw_x1 * width)
        jaw_y1 = int(self._jaw_y1 * height)
        jaw_x2 = int(self._jaw_x2 * width)
        jaw_y2 = int(self._jaw_y2 * height)

        if self._collision_mask is not None:
            mask = cv2.resize(self._collision_mask, (width, height), interpolation=cv2.INTER_NEAREST)
            overlay = panel.copy()
            selected = mask >= 128
            if selected.any():
                overlay[selected] = cv2.addWeighted(
                    panel[selected], 0.45, np.full_like(panel[selected], 255), 0.55, 0.0
                )
            panel = overlay

            # Draw bounding boxes around obstacle contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) < 200:
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(panel, (x, y), (x + w, y + h), (0, 60, 255), 2)
                cv2.putText(panel, 'OBSTACLE', (x, max(y - 6, 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 60, 255), 1, cv2.LINE_AA)

        if self._collision_now:
            overlay = panel.copy()
            cv2.rectangle(overlay, (jaw_x1, jaw_y1), (jaw_x2, jaw_y2), (0, 0, 255), -1)
            cv2.addWeighted(overlay, 0.28, panel, 0.72, 0.0, panel)

        cv2.rectangle(panel, (jaw_x1, jaw_y1), (jaw_x2, jaw_y2), (255, 255, 255), 2)
        return panel

    def _render_banner(self, width: int, height: int):
        banner = np.zeros((height, width, 3), dtype=np.uint8)
        if self._collision_now:
            banner[:] = (0, 0, 180)
            title = '! COLLISION WARNING !'
            color = (255, 255, 255)
        else:
            banner[:] = (30, 80, 30)
            title = 'CLEAR'
            color = (230, 255, 230)

        cv2.putText(banner, title, (14, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
        cv2.putText(banner, self._status_text, (14, height - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
        return banner

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    if 'DISPLAY' not in os.environ:
        os.environ['DISPLAY'] = ':0'

    rclpy.init(args=args)
    node = FrameDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
