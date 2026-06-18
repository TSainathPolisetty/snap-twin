"""
Split-view frame display node.

Shows overhead annotated RGB on the left and wrist depth colormap on the right,
with a three-state bottom banner driven by /arm_state (NORMAL / RETREATING / HOLDING).

Runs fullscreen. Works on both X11 and Wayland (ubuntu-frame).
"""

import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

WINDOW_NAME = 'Snap-Twin | Overhead + Wrist Monitor'
BANNER_HEIGHT = 60

# Banner colours (BGR)
_CYAN        = (196, 180, 24)   # NORMAL
_DARK_PURPLE = (89,  18,  61)   # RETREATING
_GREEN       = (87,  139, 46)   # HOLDING

# Gengar / icon colours (BGR)
_GENGAR_FILL    = (122, 58,  91)   # body fill #5B3A7A
_GENGAR_OUTLINE = (208, 136, 176)  # body outline #B088D0
_GENGAR_EYE     = (64,  64,  232)  # red eyes #E84040
_GOLD           = (0,   215, 255)  # spooked robot gold #FFD700
_WHITE          = (255, 255, 255)


class FrameDisplayNode(Node):

    def __init__(self):
        super().__init__('frame_display_node')

        self.declare_parameter('screen_width',  1920)
        self.declare_parameter('screen_height', 1080)

        self._win_w = int(self.get_parameter('screen_width').value)
        self._win_h = int(self.get_parameter('screen_height').value)

        self._overhead  = None
        self._depth     = None
        self._arm_state = 'NORMAL'

        self.create_subscription(Image,  '/overhead/image_annotated',      self._overhead_cb,  5)
        self.create_subscription(Image,  '/camera/depth/visualization',    self._depth_cb,     5)
        self.create_subscription(String, '/arm_state',                     self._arm_state_cb, 10)

        self.create_timer(1.0 / 15.0, self._display_cb)

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        self.get_logger().info(f'FrameDisplayNode ready — fullscreen {self._win_w}x{self._win_h}')

    # ── subscriptions ──────────────────────────────────────────────────────────

    def _overhead_cb(self, msg: Image):
        rgb = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self._overhead = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def _depth_cb(self, msg: Image):
        rgb = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self._depth = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def _arm_state_cb(self, msg: String):
        self._arm_state = msg.data

    # ── render loop ────────────────────────────────────────────────────────────

    def _display_cb(self):
        # Re-read window size each frame so we adapt if the compositor resizes us
        rect = cv2.getWindowImageRect(WINDOW_NAME)
        if rect[2] > 0 and rect[3] > 0:
            self._win_w, self._win_h = rect[2], rect[3]

        panel_w = self._win_w // 2
        panel_h = max(1, self._win_h - BANNER_HEIGHT)

        left  = self._render_panel(self._overhead, panel_w, panel_h, 'OVERHEAD')
        right = self._render_panel(self._depth,    panel_w, panel_h, 'WRIST DEPTH')
        banner = self._render_banner(self._win_w, BANNER_HEIGHT)

        canvas = np.vstack([np.hstack([left, right]), banner])
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
        cv2.putText(panel, label, (14, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, _WHITE, 2, cv2.LINE_AA)
        return panel

    # ── banner ─────────────────────────────────────────────────────────────────

    def _render_banner(self, width: int, height: int):
        banner = np.zeros((height, width, 3), dtype=np.uint8)
        state  = self._arm_state

        if state == 'RETREATING':
            banner[:] = _DARK_PURPLE
            text      = 'Gengar detected. Spooked! Retreating'
            icon_x    = width - 130
        elif state == 'HOLDING':
            banner[:] = _GREEN
            text      = 'Feeling safe now :)'
            icon_x    = width - 70
        else:  # NORMAL (and INITIALISING)
            banner[:] = _CYAN
            text      = 'I am free!'
            icon_x    = width - 70

        cy = height // 2
        cv2.putText(banner, text, (16, cy + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.8, _WHITE, 2, cv2.LINE_AA)

        if state == 'RETREATING':
            self._draw_gengar_and_spooked_robot(banner, icon_x, cy)
        elif state == 'HOLDING':
            self._draw_sleeping_robot(banner, icon_x, cy)
        else:
            self._draw_happy_robot(banner, icon_x, cy)

        return banner

    # ── icon helpers ───────────────────────────────────────────────────────────

    def _draw_happy_robot(self, img, x: int, cy: int):
        """White happy robot: square head, antenna, dot eyes, smile, waving arm."""
        hw, hh = 20, 18
        hx, hy = x - hw // 2, cy - hh // 2
        cv2.rectangle(img, (hx, hy), (hx + hw, hy + hh), _WHITE, 1)
        # antenna
        cv2.line(img, (hx + hw // 2, hy), (hx + hw // 2, hy - 6), _WHITE, 1)
        cv2.circle(img, (hx + hw // 2, hy - 7), 2, _WHITE, -1)
        # eyes
        cv2.circle(img, (hx + 5,  hy + 6), 2, _WHITE, -1)
        cv2.circle(img, (hx + 15, hy + 6), 2, _WHITE, -1)
        # smile
        cv2.ellipse(img, (hx + hw // 2, hy + 13), (6, 4), 0, 0, 180, _WHITE, 1)
        # waving arm (right side, angled lines)
        ax, ay = hx + hw, hy + 5
        cv2.line(img, (ax, ay),     (ax + 10, ay - 6),  _WHITE, 2)
        cv2.line(img, (ax + 10, ay - 6), (ax + 14, ay - 2), _WHITE, 2)

    def _draw_sleeping_robot(self, img, x: int, cy: int):
        """White sleeping robot: closed eyes (arcs), smile, zzz floating up."""
        hw, hh = 20, 18
        hx, hy = x - hw // 2, cy - hh // 2
        cv2.rectangle(img, (hx, hy), (hx + hw, hy + hh), _WHITE, 1)
        # antenna
        cv2.line(img, (hx + hw // 2, hy), (hx + hw // 2, hy - 6), _WHITE, 1)
        cv2.circle(img, (hx + hw // 2, hy - 7), 2, _WHITE, -1)
        # closed eyes (downward arcs = sleeping)
        cv2.ellipse(img, (hx + 5,  hy + 7), (3, 2), 0, 180, 360, _WHITE, 1)
        cv2.ellipse(img, (hx + 15, hy + 7), (3, 2), 0, 180, 360, _WHITE, 1)
        # gentle smile
        cv2.ellipse(img, (hx + hw // 2, hy + 13), (5, 3), 0, 0, 180, _WHITE, 1)
        # zzz rising up-right
        zx, zy = hx + hw + 4, hy - 2
        cv2.putText(img, 'z', (zx,      zy),      cv2.FONT_HERSHEY_SIMPLEX, 0.50, _WHITE, 1, cv2.LINE_AA)
        cv2.putText(img, 'z', (zx + 6,  zy - 7),  cv2.FONT_HERSHEY_SIMPLEX, 0.40, _WHITE, 1, cv2.LINE_AA)
        cv2.putText(img, 'z', (zx + 11, zy - 13), cv2.FONT_HERSHEY_SIMPLEX, 0.35, _WHITE, 1, cv2.LINE_AA)

    def _draw_gengar_and_spooked_robot(self, img, x: int, cy: int):
        """Draw Gengar (left) and spooked robot (right) side by side."""
        self._draw_gengar(img, x, cy)
        self._draw_spooked_robot(img, x + 60, cy)

    def _draw_gengar(self, img, x: int, cy: int):
        """Medium-purple Gengar with crown spikes, stub limbs, red eyes, zigzag grin."""
        # body ellipse
        cv2.ellipse(img, (x, cy), (18, 15), 0, 0, 360, _GENGAR_FILL, -1)
        cv2.ellipse(img, (x, cy), (18, 15), 0, 0, 360, _GENGAR_OUTLINE, 1)

        # crown spikes — 4 triangles across top arc
        spikes = [(-12, -12), (-4, -16), (4, -16), (12, -12)]
        for sx, sy in spikes:
            tip = np.array([[x + sx, cy + sy - 6]], dtype=np.int32)
            bl  = np.array([[x + sx - 4, cy + sy + 2]], dtype=np.int32)
            br  = np.array([[x + sx + 4, cy + sy + 2]], dtype=np.int32)
            tri = np.array([tip, bl, br]).reshape((-1, 1, 2))
            cv2.fillPoly(img, [tri], _GENGAR_FILL)
            cv2.polylines(img, [tri], True, _GENGAR_OUTLINE, 1)

        # stub arms
        for sign in (-1, 1):
            arm = np.array([
                [x + sign * 16, cy - 3],
                [x + sign * 24, cy - 8],
                [x + sign * 24, cy + 2],
            ], dtype=np.int32).reshape((-1, 1, 2))
            cv2.fillPoly(img, [arm], _GENGAR_FILL)
            cv2.polylines(img, [arm], True, _GENGAR_OUTLINE, 1)

        # stub feet
        for fx_off in (-7, 7):
            foot = np.array([
                [x + fx_off,     cy + 14],
                [x + fx_off - 5, cy + 20],
                [x + fx_off + 5, cy + 20],
            ], dtype=np.int32).reshape((-1, 1, 2))
            cv2.fillPoly(img, [foot], _GENGAR_FILL)
            cv2.polylines(img, [foot], True, _GENGAR_OUTLINE, 1)

        # eyes (red diamond polygons)
        for ex_off in (-7, 7):
            eye = np.array([
                [x + ex_off,     cy - 8],
                [x + ex_off - 4, cy - 4],
                [x + ex_off,     cy],
                [x + ex_off + 4, cy - 4],
            ], dtype=np.int32).reshape((-1, 1, 2))
            cv2.fillPoly(img, [eye], _GENGAR_EYE)
            cv2.circle(img, (x + ex_off, cy - 5), 1, _WHITE, -1)

        # zigzag grin
        grin_y = cy + 7
        pts = [(x - 12, grin_y), (x - 7, grin_y + 5), (x - 2, grin_y),
               (x + 3, grin_y + 5), (x + 8, grin_y), (x + 12, grin_y + 4)]
        for i in range(len(pts) - 1):
            cv2.line(img, pts[i], pts[i + 1], _WHITE, 1)
        # tooth blocks
        for tx in (x - 9, x, x + 7):
            cv2.rectangle(img, (tx - 2, grin_y - 4), (tx + 2, grin_y), _WHITE, -1)

    def _draw_spooked_robot(self, img, x: int, cy: int):
        """Gold spooked robot: shocked O-eyes, frown, fright lines, !! text."""
        hw, hh = 20, 18
        hx, hy = x - hw // 2, cy - hh // 2
        cv2.rectangle(img, (hx, hy), (hx + hw, hy + hh), _GOLD, 1)
        # antenna
        cv2.line(img, (hx + hw // 2, hy), (hx + hw // 2, hy - 6), _GOLD, 1)
        cv2.circle(img, (hx + hw // 2, hy - 7), 2, _GOLD, -1)
        # shocked O eyes
        cv2.circle(img, (hx + 5,  hy + 6), 3, _GOLD, 1)
        cv2.circle(img, (hx + 15, hy + 6), 3, _GOLD, 1)
        # frown arc (inverted smile)
        cv2.ellipse(img, (hx + hw // 2, hy + 15), (5, 3), 0, 180, 360, _GOLD, 1)
        # fright lines (left side)
        for i, (dx, dy) in enumerate([(-8, -4), (-10, 2), (-8, 8)]):
            cv2.line(img, (hx - 2, hy + 5 + i * 4), (hx + dx, hy + dy + i * 4), _GOLD, 1)
        # !! to the right
        cv2.putText(img, '!!', (hx + hw + 3, hy + hh - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, _GOLD, 1, cv2.LINE_AA)

    # ── cleanup ────────────────────────────────────────────────────────────────

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    # Wayland (ubuntu-frame): set backend hints before any GUI init.
    # If WAYLAND_DISPLAY is set, use Wayland — do NOT force DISPLAY=:0.
    if os.environ.get('WAYLAND_DISPLAY'):
        os.environ.setdefault('GDK_BACKEND', 'wayland')
        os.environ.setdefault('QT_QPA_PLATFORM', 'wayland')
        os.environ.pop('DISPLAY', None)
    elif not os.environ.get('DISPLAY'):
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
