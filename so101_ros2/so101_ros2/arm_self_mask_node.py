"""
arm_self_mask_node.py — Runtime arm self-masking node.
=======================================================
Publishes /arm_self_mask (sensor_msgs/Image, mono8, 518×518) marking pixels
that belong to the robot arm, so collision_checker_node can exclude them.

Startup mode detection (checked in order):
  1. calibration/camera_extrinsics.yaml exists → mode = "fk"
     Uses yourdfpy FK + /joint_states + solvePnP extrinsics to project arm
     links into the live camera frame in real-time.
  2. calibration/arm_envelope_mask.png exists  → mode = "static"
     Republishes the pre-built static envelope mask on every timer tick.
  3. Neither found                              → mode = "none"
     Publishes an all-zero mask — no arm pixels are excluded. A clear
     warning is logged so the operator knows collision detection is
     running without arm awareness.

FK mode detail:
  - Reads /joint_states (leader) for current joint positions.
  - Runs yourdfpy FK for every link in ARM_LINKS chain.
  - Projects each 3D link origin (base_link frame) through the calibrated
    rvec/tvec + brio.yaml intrinsics into 518×518 pixel coordinates.
  - Draws a thick connected polyline shoulder→wrist→gripper on a blank
    mono8 canvas, then dilates by DILATION_PX pixels for safety margin.

Publishes:
  /arm_self_mask   sensor_msgs/Image  mono8  (518×518, white=arm pixels)

Parameters:
  urdf_path        str   path to final_twin.urdf
  calib_dir        str   directory containing calibration outputs
  dilation_px      int   15   dilation kernel radius (safety margin)
"""

import os
import math
import array as arr

import numpy as np
import cv2
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

# ─── Constants ───────────────────────────────────────────────────────────────
DEPTH_W, DEPTH_H = 518, 518

# Arm link chain in base→tip order (used for FK polyline projection)
ARM_LINKS = [
    'base_link',
    'shoulder_link',
    'upper_arm_link',
    'lower_arm_link',
    'wrist_link',
    'gripper_link',
    'gripper_frame_link',
]

# Default paths — the node is built/installed into ros2_ws, but calibration
# and URDF live in the snap-twin repo.  Override via --ros-args -p if moved.
_REPO_ROOT       = os.path.expanduser('~/automate-demo/snap-twin')
_DEFAULT_URDF    = os.path.join(_REPO_ROOT, 'final_twin.urdf')
_DEFAULT_CALIB   = os.path.join(_REPO_ROOT, 'calibration')


def _load_intrinsics(path):
    """Load brio.yaml and return (K, dist) scaled to 518×518."""
    with open(path) as f:
        data = yaml.safe_load(f)
    orig_w = data['image_width']
    orig_h = data['image_height']
    d = data['camera_matrix']['data']
    fx, fy, cx, cy = d[0], d[4], d[2], d[5]
    dist = data['distortion_coefficients']['data']
    sx, sy = DEPTH_W / orig_w, DEPTH_H / orig_h
    K = np.array([[fx * sx, 0.0, cx * sx],
                  [0.0, fy * sy, cy * sy],
                  [0.0, 0.0,    1.0     ]], dtype=np.float64)
    dist = np.array(dist, dtype=np.float64).reshape(1, -1)
    return K, dist


class ArmSelfMaskNode(Node):

    def __init__(self):
        super().__init__('arm_self_mask_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('urdf_path',  _DEFAULT_URDF)
        self.declare_parameter('calib_dir',  _DEFAULT_CALIB)
        self.declare_parameter('dilation_px', 15)

        urdf_path  = self.get_parameter('urdf_path').value
        calib_dir  = self.get_parameter('calib_dir').value
        dilation   = int(self.get_parameter('dilation_px').value)

        self._dilation_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (dilation * 2 + 1, dilation * 2 + 1))

        # ── Internal state ────────────────────────────────────────────────────
        self._mode          = 'none'
        self._static_mask   = None    # (518,518) uint8 for static mode
        self._robot         = None    # yourdfpy URDF for fk mode
        self._K             = None
        self._dist          = None
        self._rvec          = None
        self._tvec          = None
        self._joint_positions = {}    # {name: rad}

        # ── Determine mode ────────────────────────────────────────────────────
        extrin_path = os.path.join(calib_dir, 'camera_extrinsics.yaml')
        mask_path   = os.path.join(calib_dir, 'arm_envelope_mask.png')

        if os.path.exists(extrin_path):
            self._init_fk_mode(urdf_path, extrin_path)
        elif os.path.exists(mask_path):
            self._init_static_mode(mask_path)
        else:
            self._mode = 'none'
            self.get_logger().warn(
                'ArmSelfMask: No calibration data found '
                f'(checked {extrin_path} and {mask_path}). '
                'Mode = "none" — /arm_self_mask will be all-zero. '
                'Collision detection will see the arm as potential obstacles. '
                'Run scripts/calibrate_camera_arm.py to fix this.'
            )

        self.get_logger().info(f'ArmSelfMask mode: {self._mode}')

        # ── Publisher ─────────────────────────────────────────────────────────
        self._mask_pub = self.create_publisher(Image, '/arm_self_mask', 5)

        # ── Subscriptions ─────────────────────────────────────────────────────
        if self._mode == 'fk':
            self.create_subscription(JointState, '/joint_states',
                                     self._js_cb, 10)

        # ── 10 Hz timer ───────────────────────────────────────────────────────
        self.create_timer(0.1, self._timer_cb)

    # ── Initialisation ────────────────────────────────────────────────────────

    def _init_fk_mode(self, urdf_path, extrin_path):
        try:
            from yourdfpy import URDF
            import sys, io
            # Suppress yourdfpy mesh-load warnings (STL files may reference a
            # local HTTP server that isn't running; FK computation doesn't need them)
            _old_stderr = sys.stderr
            sys.stderr = io.StringIO()
            try:
                self._robot = URDF.load(urdf_path)
            finally:
                sys.stderr = _old_stderr
        except Exception as e:
            self.get_logger().error(f'Failed to load URDF {urdf_path}: {e} — falling back to none mode')
            self._mode = 'none'
            return

        # Load extrinsics
        try:
            with open(extrin_path) as f:
                ext = yaml.safe_load(f)
            self._rvec = np.array(ext['rotation_vector'],    dtype=np.float64).reshape(3, 1)
            self._tvec = np.array(ext['translation_vector'], dtype=np.float64).reshape(3, 1)
        except Exception as e:
            self.get_logger().error(f'Failed to load extrinsics {extrin_path}: {e} — falling back to none mode')
            self._mode = 'none'
            return

        # Load intrinsics
        intrinsics_path = os.path.expanduser('~/.ros/camera_info/brio.yaml')
        try:
            self._K, self._dist = _load_intrinsics(intrinsics_path)
        except Exception as e:
            self.get_logger().error(f'Failed to load intrinsics {intrinsics_path}: {e} — falling back to none mode')
            self._mode = 'none'
            return

        self._mode = 'fk'
        self.get_logger().info(
            f'FK mode: URDF={urdf_path}, extrinsics={extrin_path}, '
            f'reprojection_error={ext.get("reprojection_error_px","?")}px'
        )

    def _init_static_mode(self, mask_path):
        try:
            mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
            if mask is None:
                raise FileNotFoundError(f'cv2.imread returned None for {mask_path}')
            if mask.shape != (DEPTH_H, DEPTH_W):
                mask = cv2.resize(mask, (DEPTH_W, DEPTH_H), interpolation=cv2.INTER_NEAREST)
            self._static_mask = mask
            self._mode = 'static'
            self.get_logger().info(f'Static mode: mask={mask_path} ({mask.shape})')
        except Exception as e:
            self.get_logger().error(f'Failed to load envelope mask {mask_path}: {e} — falling back to none mode')
            self._mode = 'none'

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = pos

    # ── Timer: compute and publish mask ───────────────────────────────────────

    def _timer_cb(self):
        if self._mode == 'fk':
            mask = self._compute_fk_mask()
        elif self._mode == 'static':
            mask = self._static_mask.copy()
        else:
            mask = np.zeros((DEPTH_H, DEPTH_W), dtype=np.uint8)

        self._publish_mask(mask)

    def _compute_fk_mask(self):
        """Project arm link origins through extrinsics to 2D, draw polyline, dilate."""
        canvas = np.zeros((DEPTH_H, DEPTH_W), dtype=np.uint8)

        if not self._joint_positions:
            # No joint state received yet — publish empty mask
            return canvas

        # Update URDF FK with current joint positions (radians)
        try:
            self._robot.update_cfg(self._joint_positions)
        except Exception as e:
            self.get_logger().warn(f'FK update_cfg failed: {e}', throttle_duration_sec=5.0)
            return canvas

        # Compute 3D link origins in base_link frame
        pts_3d = []
        for link_name in ARM_LINKS:
            try:
                T = self._robot.get_transform(frame_to=link_name, frame_from='base_link')
                pts_3d.append(T[:3, 3])
            except Exception as e:
                self.get_logger().warn(
                    f'FK get_transform failed for {link_name}: {e}',
                    throttle_duration_sec=5.0)
                pts_3d.append(np.zeros(3))

        pts_3d = np.array(pts_3d, dtype=np.float64).reshape(-1, 1, 3)

        # Project to 2D image coordinates
        try:
            pts_2d, _ = cv2.projectPoints(pts_3d, self._rvec, self._tvec,
                                          self._K, self._dist)
        except Exception as e:
            self.get_logger().warn(f'projectPoints failed: {e}', throttle_duration_sec=5.0)
            return canvas

        pts_2d = pts_2d.reshape(-1, 2).astype(np.int32)

        # Clip to image bounds
        pts_2d[:, 0] = np.clip(pts_2d[:, 0], 0, DEPTH_W - 1)
        pts_2d[:, 1] = np.clip(pts_2d[:, 1], 0, DEPTH_H - 1)

        # Draw thick polyline connecting the arm link chain
        cv2.polylines(canvas, [pts_2d.reshape(-1, 1, 2)],
                      isClosed=False, color=255, thickness=12)
        # Also draw filled circles at each link origin for extra coverage
        for pt in pts_2d:
            cv2.circle(canvas, tuple(pt), radius=10, color=255, thickness=-1)

        # Dilate for safety margin
        canvas = cv2.dilate(canvas, self._dilation_kernel, iterations=1)
        return canvas

    # ── Publisher helper ──────────────────────────────────────────────────────

    def _publish_mask(self, mask: np.ndarray):
        msg = Image()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.height          = DEPTH_H
        msg.width           = DEPTH_W
        msg.encoding        = 'mono8'
        msg.is_bigendian    = False
        msg.step            = DEPTH_W
        msg.data            = arr.array('B', mask.tobytes())
        self._mask_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmSelfMaskNode()
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
