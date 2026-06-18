#!/usr/bin/env python3
"""
calibrate_camera_arm.py - Standalone arm-camera calibration script.

Calibrates the OVERHEAD (Brio, /dev/video0) camera pose relative to the arm
base frame. Blob detection uses direct Brio capture with HSV orange segmentation
(NOT depth inference — depth_anything_node now runs on the wrist camera and
its data is not usable for overhead pose calibration).

Produces (in ~/automate-demo/snap-twin/calibration/):
  camera_extrinsics.yaml - rvec/tvec from solvePnP (only if >=8 pairs and error <=15px)

Prerequisites:
  - so101_ros2_sub (follower arm) must be running
  - so101_ros2_pub (leader arm) must NOT be running during Step 1
    (would conflict with this script's /gesture/joint_states commands)
  - gesture_node must NOT be running during Step 1
  - For Step 2 (manual phase): start so101_ros2_pub so leader arm is live

Run:
    source ~/automate-demo/ros2_ws/install/setup.bash
    cd ~/automate-demo/snap-twin
    python3 scripts/calibrate_camera_arm.py

NOTE: Camera intrinsics (brio.yaml) were originally calibrated at 640x480,
then scaled non-uniformly to 1920x1080 (fx and fy use different scale factors),
and are re-scaled here to the actual captured frame size. This known imprecision
reduces solvePnP accuracy. Recalibrating at native 1920x1080 resolution would
significantly improve results - the current intrinsics are used as-is.
"""

import math
import os
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool as BoolMsg

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
URDF_FILE = os.path.join(BASE_DIR, 'final_twin.urdf')
CALIB_DIR = os.path.join(BASE_DIR, 'calibration')
INTRINSICS_FILE = os.path.expanduser('~/.ros/camera_info/brio.yaml')
EXTRIN_OUT = os.path.join(CALIB_DIR, 'camera_extrinsics.yaml')

# Overhead camera device (Brio 101)
OVERHEAD_DEVICE = '/dev/video0'

ROI_X1, ROI_Y1, ROI_X2, ROI_Y2 = 0.15, 0.15, 0.85, 0.85
MIN_BLOB_AREA = 300
SETTLE_SECS = 2.5

# HSV orange range for arm detection (tune if arm color varies)
ARM_HUE_LOW, ARM_HUE_HIGH   = 5,   25
ARM_SAT_LOW, ARM_SAT_HIGH   = 120, 255
ARM_VAL_LOW, ARM_VAL_HIGH   = 80,  255

CORRESPONDENCE_POSES = [
    {'shoulder_pan': 0, 'shoulder_lift': 20, 'elbow_flex': 30, 'wrist_flex': -15, 'wrist_roll': 0, 'gripper': 0},
    {'shoulder_pan': 25, 'shoulder_lift': 20, 'elbow_flex': 30, 'wrist_flex': -15, 'wrist_roll': 0, 'gripper': 0},
    {'shoulder_pan': -25, 'shoulder_lift': 20, 'elbow_flex': 30, 'wrist_flex': -15, 'wrist_roll': 0, 'gripper': 0},
    {'shoulder_pan': 0, 'shoulder_lift': 35, 'elbow_flex': 20, 'wrist_flex': -10, 'wrist_roll': 0, 'gripper': 0},
    {'shoulder_pan': 15, 'shoulder_lift': 10, 'elbow_flex': 50, 'wrist_flex': -30, 'wrist_roll': 0, 'gripper': 0},
    {'shoulder_pan': -15, 'shoulder_lift': 10, 'elbow_flex': 50, 'wrist_flex': -30, 'wrist_roll': 0, 'gripper': 0},
    {'shoulder_pan': 0, 'shoulder_lift': 45, 'elbow_flex': 10, 'wrist_flex': 0, 'wrist_roll': 0, 'gripper': 0},
    {'shoulder_pan': 20, 'shoulder_lift': 30, 'elbow_flex': 40, 'wrist_flex': -25, 'wrist_roll': 10, 'gripper': 0},
    {'shoulder_pan': -20, 'shoulder_lift': 30, 'elbow_flex': 40, 'wrist_flex': -25, 'wrist_roll': -10, 'gripper': 0},
    {'shoulder_pan': 10, 'shoulder_lift': 25, 'elbow_flex': 35, 'wrist_flex': -20, 'wrist_roll': 0, 'gripper': 0},
    {'shoulder_pan': -10, 'shoulder_lift': 25, 'elbow_flex': 35, 'wrist_flex': -20, 'wrist_roll': 0, 'gripper': 0},
    {'shoulder_pan': 0, 'shoulder_lift': 15, 'elbow_flex': 55, 'wrist_flex': -35, 'wrist_roll': 0, 'gripper': 0},
]


def load_intrinsics(path, frame_w, frame_h):
    """Load brio.yaml intrinsics, scaled to the actual captured frame size."""
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    orig_w = data['image_width']
    orig_h = data['image_height']
    d = data['camera_matrix']['data']
    fx, fy, cx, cy = d[0], d[4], d[2], d[5]
    dist = data['distortion_coefficients']['data']

    sx = frame_w / orig_w
    sy = frame_h / orig_h
    K = np.array([
        [fx * sx, 0.0, cx * sx],
        [0.0, fy * sy, cy * sy],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    return K, np.array(dist, dtype=np.float64).reshape(1, -1)


def extract_roi(frame_h, frame_w, roi=(ROI_X1, ROI_Y1, ROI_X2, ROI_Y2)):
    y1 = int(roi[1] * frame_h)
    y2 = int(roi[3] * frame_h)
    x1 = int(roi[0] * frame_w)
    x2 = int(roi[2] * frame_w)
    return y1, y2, x1, x2


def detect_arm_blob_hsv(frame_bgr, frame_w, frame_h):
    """Detect orange arm blob via HSV segmentation. Returns (mask, largest_contour or None)."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lower = np.array([ARM_HUE_LOW,  ARM_SAT_LOW,  ARM_VAL_LOW],  dtype=np.uint8)
    upper = np.array([ARM_HUE_HIGH, ARM_SAT_HIGH, ARM_VAL_HIGH], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return mask, None
    return mask, max(contours, key=cv2.contourArea)


def most_distal_pixel(contour, frame_w, frame_h):
    """Return the contour point most distal from the arm base (bottom-centre of frame)."""
    ref_x = frame_w // 2
    ref_y = frame_h
    pts = contour.reshape(-1, 2)
    dists = np.sqrt((pts[:, 0] - ref_x) ** 2 + (pts[:, 1] - ref_y) ** 2)
    idx = int(np.argmax(dists))
    return int(pts[idx, 0]), int(pts[idx, 1])


def validate_blob(contour, frame_w, frame_h):
    area = cv2.contourArea(contour)
    if area < MIN_BLOB_AREA:
        return False, f'area {area:.0f} < {MIN_BLOB_AREA}'
    bx, by, bw, bh = cv2.boundingRect(contour)
    margin = 5
    # Allow bottom-edge contact (arm base is always near frame bottom in overhead view).
    # Reject only if left, right, or top edges are clipped (would clip the gripper tip).
    if bx < margin or by < margin or bx + bw > frame_w - margin:
        return False, f'touches L/R/T image edge ({bx},{by})-({bx+bw},{by+bh})'
    return True, 'OK'


def fk_gripper_tip(robot, pose_deg):
    cfg = {name: deg * math.pi / 180.0 for name, deg in pose_deg.items()}
    robot.update_cfg(cfg)
    transform = robot.get_transform(frame_to='gripper_frame_link', frame_from='base_link')
    return np.array(transform[:3, 3], dtype=np.float64)


class CalibrationNode(Node):

    def __init__(self):
        super().__init__('calibrate_camera_arm')
        self._latest_js  = None
        self._cap        = None
        self._frame_w    = 0
        self._frame_h    = 0

        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self._gesture_active_pub = self.create_publisher(BoolMsg, '/gesture_active', 10)
        self._gesture_pub = self.create_publisher(JointState, '/gesture/joint_states', 10)

        self.get_logger().info('CalibrationNode ready')

    def _js_cb(self, msg: JointState):
        self._latest_js = {name: pos for name, pos in zip(msg.name, msg.position)}

    def open_camera(self, device=OVERHEAD_DEVICE):
        """Open the overhead Brio camera directly (no ROS, no TRT)."""
        dev_idx = int(device.replace('/dev/video', '')) if '/dev/video' in device else 0
        cap = cv2.VideoCapture(dev_idx)
        if not cap.isOpened():
            print(f'ERROR: Cannot open camera {device}')
            return False
        ret, frame = cap.read()
        if not ret or frame is None:
            print(f'ERROR: Camera {device} opened but returned no frame')
            cap.release()
            return False
        self._cap = cap
        self._frame_h, self._frame_w = frame.shape[:2]
        print(f'Overhead camera opened: {self._frame_w}x{self._frame_h}')
        return True

    def grab_frame(self):
        """Read one frame from the Brio. Returns BGR ndarray or None."""
        if self._cap is None:
            return None
        # Flush stale buffered frames by reading twice quickly
        self._cap.read()
        ret, frame = self._cap.read()
        return frame if ret and frame is not None else None

    def command_arm(self, pose_deg, settle_secs=SETTLE_SECS):
        names = list(pose_deg.keys())
        rads  = [deg * math.pi / 180.0 for deg in pose_deg.values()]
        end   = time.time() + settle_secs
        while time.time() < end:
            active      = BoolMsg()
            active.data = True
            self._gesture_active_pub.publish(active)
            js                 = JointState()
            js.header.stamp    = self.get_clock().now().to_msg()
            js.name            = names
            js.position        = rads
            self._gesture_pub.publish(js)
            rclpy.spin_once(self, timeout_sec=0.1)

    def release_gesture_control(self):
        for _ in range(5):
            active      = BoolMsg()
            active.data = False
            self._gesture_active_pub.publish(active)
            rclpy.spin_once(self, timeout_sec=0.05)

    def step1_correspondence(self, robot):
        print('\n══ STEP 1 - Scripted correspondence sweep ══')
        pairs_3d = []
        pairs_2d = []
        fw, fh = self._frame_w, self._frame_h

        for i, pose in enumerate(CORRESPONDENCE_POSES):
            print(f'  Pose {i+1}/{len(CORRESPONDENCE_POSES)}: pan={pose["shoulder_pan"]} '
                  f'lift={pose["shoulder_lift"]} elbow={pose["elbow_flex"]}')
            self.command_arm(pose, settle_secs=SETTLE_SECS)

            frame = self.grab_frame()
            if frame is None:
                print('    SKIP: no camera frame')
                continue

            _, contour = detect_arm_blob_hsv(frame, fw, fh)
            if contour is None:
                print('    SKIP: no arm blob detected')
                continue

            ok, reason = validate_blob(contour, fw, fh)
            if not ok:
                print(f'    SKIP: validation failed - {reason}')
                continue

            u, v   = most_distal_pixel(contour, fw, fh)
            pt3d   = fk_gripper_tip(robot, pose)
            pairs_3d.append(pt3d)
            pairs_2d.append([float(u), float(v)])
            area = cv2.contourArea(contour)
            print(f'    ACCEPTED - area={area:.0f}px  tip=({u},{v})  '
                  f'3D=({pt3d[0]:.3f},{pt3d[1]:.3f},{pt3d[2]:.3f})m')

        print(f'Step 1 complete: {len(pairs_3d)} valid correspondences')
        return pairs_3d, pairs_2d

    def step2_manual(self, robot, pairs_3d, pairs_2d):
        print('\n══ STEP 2 - Manual calibration phase ══')
        fw, fh = self._frame_w, self._frame_h
        pause_threshold  = 0.005
        pause_min_secs   = 0.5
        duration_secs    = 18.0

        self.release_gesture_control()
        print('Go - move the leader arm now')
        print('Moving slowly and pausing at interesting poses gives best results.')

        prev_js         = None
        pause_start     = None
        captured_at_pause = False
        end_time        = time.time() + duration_secs

        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)
            js = self._latest_js
            if js is None or prev_js is None:
                prev_js = js
                continue

            common_joints = set(js.keys()) & set(prev_js.keys())
            if not common_joints:
                prev_js = js
                continue

            max_delta = max(abs(js[joint] - prev_js[joint]) for joint in common_joints)
            prev_js   = js

            if max_delta < pause_threshold:
                if pause_start is None:
                    pause_start = time.time()
                    captured_at_pause = False
                elif not captured_at_pause and time.time() - pause_start >= pause_min_secs:
                    frame = self.grab_frame()
                    if frame is None:
                        continue
                    _, contour = detect_arm_blob_hsv(frame, fw, fh)
                    if contour is not None:
                        ok, reason = validate_blob(contour, fw, fh)
                        if ok:
                            u, v     = most_distal_pixel(contour, fw, fh)
                            pose_deg = {joint: value * 180.0 / math.pi for joint, value in js.items()}
                            pt3d     = fk_gripper_tip(robot, pose_deg)
                            pairs_3d.append(pt3d)
                            pairs_2d.append([float(u), float(v)])
                            print(f'  Manual ACCEPTED - tip=({u},{v})  3D=({pt3d[0]:.3f},{pt3d[1]:.3f},{pt3d[2]:.3f})m')
                        else:
                            print(f'  Manual SKIP - {reason}')
                    captured_at_pause = True
            else:
                pause_start = None
                captured_at_pause = False

        print(f'Manual phase complete. Total pairs so far: {len(pairs_3d)}')
        return pairs_3d, pairs_2d

    def step3_solve(self, pairs_3d, pairs_2d):
        print(f'\n══ STEP 3 - Solve PnP ({len(pairs_3d)} total correspondences) ══')
        min_pairs = 8
        if len(pairs_3d) < min_pairs:
            print(f'Only {len(pairs_3d)} valid correspondences collected ({min_pairs} required) - calibration aborted')
            return False

        K, dist = load_intrinsics(INTRINSICS_FILE, self._frame_w, self._frame_h)
        print(f'Intrinsics loaded from {INTRINSICS_FILE} (scaled to {self._frame_w}x{self._frame_h})')
        print(f'  K:\n{K}')
        print(f'  dist: {dist}')

        obj_pts = np.array(pairs_3d, dtype=np.float64).reshape(-1, 1, 3)
        img_pts = np.array(pairs_2d, dtype=np.float64).reshape(-1, 1, 2)

        ret, rvec, tvec, inliers = cv2.solvePnPRansac(
            obj_pts,
            img_pts,
            K,
            dist,
            iterationsCount=1000,
            reprojectionError=8.0,
            confidence=0.99,
            flags=cv2.SOLVEPNP_EPNP,
        )
        if not ret:
            print('solvePnPRansac failed - calibration rejected')
            return False

        n_inliers = int(inliers.shape[0]) if inliers is not None else 0
        print(f'solvePnPRansac: {n_inliers}/{len(pairs_3d)} inliers')

        if inliers is not None:
            inlier_idx = inliers.flatten()
            obj_in = obj_pts[inlier_idx]
            img_in = img_pts[inlier_idx]
        else:
            obj_in = obj_pts
            img_in = img_pts

        proj, _ = cv2.projectPoints(obj_in, rvec, tvec, K, dist)
        errors = np.linalg.norm(proj.reshape(-1, 2) - img_in.reshape(-1, 2), axis=1)
        mean_err = float(errors.mean())
        max_err = float(errors.max())
        print(f'Reprojection error: mean={mean_err:.2f}px  max={max_err:.2f}px')

        max_allowed = 15.0
        if mean_err > max_allowed:
            print(f'Reprojection error too high ({mean_err:.2f}px > {max_allowed}px) - calibration rejected')
            return False

        from datetime import datetime
        data = {
            'calibration_timestamp': datetime.now().isoformat(),
            'intrinsics_file': INTRINSICS_FILE,
            'image_size': [self._frame_w, self._frame_h],
            'num_correspondences_total': int(len(pairs_3d)),
            'num_inliers': n_inliers,
            'reprojection_error_px': round(mean_err, 3),
            'rvec': rvec.flatten().tolist(),
            'tvec': tvec.flatten().tolist(),
            'rotation_vector': rvec.flatten().tolist(),
            'translation_vector': tvec.flatten().tolist(),
        }
        with open(EXTRIN_OUT, 'w', encoding='utf-8') as handle:
            yaml.dump(data, handle, default_flow_style=False, sort_keys=False)
        print(f'\nCalibration SUCCESS - saved to {EXTRIN_OUT}')
        print(f'  rvec: {rvec.flatten().tolist()}')
        print(f'  tvec: {tvec.flatten().tolist()}')
        print(f'  reprojection error: {mean_err:.2f}px')
        return True


def main():
    import argparse
    from yourdfpy import URDF

    parser = argparse.ArgumentParser(description='Arm-camera calibration script')
    parser.add_argument('--camera-device', type=str, default=OVERHEAD_DEVICE,
                        help='Overhead camera device (default: /dev/video0)')
    parser.add_argument(
        '--auto',
        action='store_true',
        help='Skip interactive input() at Step 2 - Step 2 still runs its 18s timer collecting leader arm data',
    )
    args = parser.parse_args()

    os.makedirs(CALIB_DIR, exist_ok=True)

    if not os.path.exists(URDF_FILE):
        print(f'ERROR: URDF not found at {URDF_FILE}')
        sys.exit(1)
    if not os.path.exists(INTRINSICS_FILE):
        print(f'ERROR: Camera intrinsics not found at {INTRINSICS_FILE}')
        sys.exit(1)

    print('Loading URDF for FK ...')
    import io, sys as _sys
    _stderr_save = _sys.stderr; _sys.stderr = io.StringIO()
    robot = URDF.load(URDF_FILE)
    _sys.stderr = _stderr_save
    movable = [joint.name for joint in robot.robot.joints if joint.type == 'revolute']
    print(f'  Movable joints: {movable}')

    rclpy.init()
    node = CalibrationNode()

    print(f'\nOpening overhead camera: {args.camera_device}')
    if not node.open_camera(args.camera_device):
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    try:
        pairs_3d, pairs_2d = node.step1_correspondence(robot)

        print()
        print('━' * 60)
        if args.auto:
            print('AUTO MODE: proceeding to Step 2 immediately.')
            print('Move the leader arm slowly for 18 seconds starting NOW.')
        else:
            input(
                'Ready to start the manual calibration phase? Move the leader arm\n'
                'slowly for 15-20 seconds when I say go.\n'
                'Press ENTER when ready (or Ctrl+C to skip Step 2 and go to Step 3): '
            )

        try:
            pairs_3d, pairs_2d = node.step2_manual(robot, pairs_3d, pairs_2d)
        except KeyboardInterrupt:
            print('\nManual phase skipped.')

        success = node.step3_solve(pairs_3d, pairs_2d)
        if success:
            print('\nCalibration complete.')
        else:
            print('\nCalibration complete, but solvePnP did not produce a usable result.')
    except KeyboardInterrupt:
        print('\nAborted by user.')
    finally:
        if node._cap is not None:
            node._cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
