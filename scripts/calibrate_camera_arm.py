#!/usr/bin/env python3
"""
calibrate_camera_arm.py — Standalone arm-camera calibration script.

Produces (in ~/automate-demo/snap-twin/calibration/):
  arm_envelope_mask.png      — 518×518 mono8 static fallback mask (full frame)
  camera_extrinsics.yaml     — rvec/tvec from solvePnP (only if ≥8 pairs & error ≤15px)

Prerequisites:
  • depth_anything_node must be running (publishes /camera/depth/image_raw 32FC1)
  • so101_ros2_sub (follower arm) must be running
  • so101_ros2_pub (leader arm) must NOT be running during Steps 0-2
    (would conflict with this script's /joint_states commands)
  • gesture_node must NOT be running (would animate the arm and corrupt background)
  • For Step 3 (manual phase): start so101_ros2_pub so leader arm is live

Run:
    source ~/automate-demo/ros2_ws/install/setup.bash
    cd ~/automate-demo/snap-twin
    python3 scripts/calibrate_camera_arm.py

NOTE: Camera intrinsics (brio.yaml) were originally calibrated at 640×480,
then scaled non-uniformly to 1920×1080 (fx and fy use different scale factors),
and are re-scaled here to the 518×518 depth frame. This known imprecision
reduces solvePnP accuracy. Recalibrating at native 1920×1080 resolution would
significantly improve results — the current intrinsics are used as-is.
"""

import os
import sys
import math
import time
import yaml
import argparse
import array as arr

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

# ─── Paths ──────────────────────────────────────────────────────────────────
BASE_DIR       = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
URDF_FILE      = os.path.join(BASE_DIR, "final_twin.urdf")
CALIB_DIR      = os.path.join(BASE_DIR, "calibration")
INTRINSICS_FILE = os.path.expanduser("~/.ros/camera_info/brio.yaml")
MASK_OUT       = os.path.join(CALIB_DIR, "arm_envelope_mask.png")
EXTRIN_OUT     = os.path.join(CALIB_DIR, "camera_extrinsics.yaml")

# ─── ROI (must match collision_checker_node.py defaults) ────────────────────
ROI_X1, ROI_Y1, ROI_X2, ROI_Y2 = 0.15, 0.15, 0.85, 0.85

# ─── Detection parameters ───────────────────────────────────────────────────
# Smaller threshold than collision_checker (0.30) — the arm is always present
ARM_DEPTH_THRESHOLD = 0.10     # depth delta to classify a pixel as "arm"
MIN_BLOB_AREA       = 300      # px² — reject tiny noise blobs (Step 2 gate)
SETTLE_SECS         = 2.5      # seconds to wait after commanding a pose
BACKGROUND_FRAMES   = 30       # Step 0: frames to build median background

# ─── Step 1: Envelope sweep poses (10 poses, wide range) ────────────────────
# Format: {joint_name: degrees}. Over-inclusion is fine for the fallback mask.
ENVELOPE_POSES = [
    {'shoulder_pan':  0,  'shoulder_lift': 0,   'elbow_flex':  0,  'wrist_flex':  0, 'wrist_roll':  0, 'gripper': 0},
    {'shoulder_pan': 30,  'shoulder_lift': 10,  'elbow_flex': 20,  'wrist_flex': 10, 'wrist_roll':  0, 'gripper': 0},
    {'shoulder_pan':-30,  'shoulder_lift': 10,  'elbow_flex': 20,  'wrist_flex': 10, 'wrist_roll':  0, 'gripper': 0},
    {'shoulder_pan':  0,  'shoulder_lift': 30,  'elbow_flex': 40,  'wrist_flex':-20, 'wrist_roll':  0, 'gripper': 0},
    {'shoulder_pan': 20,  'shoulder_lift': 20,  'elbow_flex': 30,  'wrist_flex':-10, 'wrist_roll': 15, 'gripper': 0},
    {'shoulder_pan':-20,  'shoulder_lift': 20,  'elbow_flex': 30,  'wrist_flex':-10, 'wrist_roll':-15, 'gripper': 0},
    {'shoulder_pan':  0,  'shoulder_lift': 40,  'elbow_flex':-20,  'wrist_flex': 30, 'wrist_roll':  0, 'gripper': 0},
    {'shoulder_pan': 15,  'shoulder_lift': 15,  'elbow_flex': 45,  'wrist_flex':-30, 'wrist_roll': 10, 'gripper': 0},
    {'shoulder_pan':-15,  'shoulder_lift': 15,  'elbow_flex': 45,  'wrist_flex':-30, 'wrist_roll':-10, 'gripper': 0},
    {'shoulder_pan':  0,  'shoulder_lift':  0,  'elbow_flex': 60,  'wrist_flex':-40, 'wrist_roll':  0, 'gripper': 0},
]

# ─── Step 2: Correspondence sweep poses (12 poses, workspace-biased) ────────
# Chosen to spread the gripper tip across the camera's visible workspace
# while keeping the arm well within the ROI and avoiding extreme joint limits.
CORRESPONDENCE_POSES = [
    {'shoulder_pan':  0,  'shoulder_lift': 20,  'elbow_flex': 30,  'wrist_flex':-15, 'wrist_roll':  0, 'gripper': 0},
    {'shoulder_pan': 25,  'shoulder_lift': 20,  'elbow_flex': 30,  'wrist_flex':-15, 'wrist_roll':  0, 'gripper': 0},
    {'shoulder_pan':-25,  'shoulder_lift': 20,  'elbow_flex': 30,  'wrist_flex':-15, 'wrist_roll':  0, 'gripper': 0},
    {'shoulder_pan':  0,  'shoulder_lift': 35,  'elbow_flex': 20,  'wrist_flex':-10, 'wrist_roll':  0, 'gripper': 0},
    {'shoulder_pan': 15,  'shoulder_lift': 10,  'elbow_flex': 50,  'wrist_flex':-30, 'wrist_roll':  0, 'gripper': 0},
    {'shoulder_pan':-15,  'shoulder_lift': 10,  'elbow_flex': 50,  'wrist_flex':-30, 'wrist_roll':  0, 'gripper': 0},
    {'shoulder_pan':  0,  'shoulder_lift': 45,  'elbow_flex': 10,  'wrist_flex':  0, 'wrist_roll':  0, 'gripper': 0},
    {'shoulder_pan': 20,  'shoulder_lift': 30,  'elbow_flex': 40,  'wrist_flex':-25, 'wrist_roll': 10, 'gripper': 0},
    {'shoulder_pan':-20,  'shoulder_lift': 30,  'elbow_flex': 40,  'wrist_flex':-25, 'wrist_roll':-10, 'gripper': 0},
    {'shoulder_pan': 10,  'shoulder_lift': 25,  'elbow_flex': 35,  'wrist_flex':-20, 'wrist_roll':  0, 'gripper': 0},
    {'shoulder_pan':-10,  'shoulder_lift': 25,  'elbow_flex': 35,  'wrist_flex':-20, 'wrist_roll':  0, 'gripper': 0},
    {'shoulder_pan':  0,  'shoulder_lift': 15,  'elbow_flex': 55,  'wrist_flex':-35, 'wrist_roll':  0, 'gripper': 0},
]

# Ordered arm link chain from base to gripper (for self-mask polyline projection)
ARM_LINKS = [
    'base_link', 'shoulder_link', 'upper_arm_link',
    'lower_arm_link', 'wrist_link', 'gripper_link', 'gripper_frame_link',
]

# Depth frame resolution (from TRT model output)
DEPTH_W, DEPTH_H = 518, 518


# ─── Helpers ────────────────────────────────────────────────────────────────

def load_intrinsics(path):
    """Load camera intrinsics from ROS camera_info YAML.
    Returns (K_518, dist_518) scaled to the 518×518 depth frame."""
    with open(path) as f:
        data = yaml.safe_load(f)
    orig_w = data['image_width']   # 1920
    orig_h = data['image_height']  # 1080
    d = data['camera_matrix']['data']
    fx, fy, cx, cy = d[0], d[4], d[2], d[5]
    dist = data['distortion_coefficients']['data']  # [k1,k2,p1,p2,k3]

    sx = DEPTH_W / orig_w
    sy = DEPTH_H / orig_h
    K = np.array([[fx * sx, 0.0,      cx * sx],
                  [0.0,      fy * sy,  cy * sy],
                  [0.0,      0.0,      1.0     ]], dtype=np.float64)
    dist = np.array(dist, dtype=np.float64).reshape(1, -1)
    return K, dist


def extract_roi(depth, roi=(ROI_X1, ROI_Y1, ROI_X2, ROI_Y2)):
    """Return (roi_array, y1, y2, x1, x2) in depth-frame pixel coordinates."""
    h, w = depth.shape
    y1 = int(roi[1] * h); y2 = int(roi[3] * h)
    x1 = int(roi[0] * w); x2 = int(roi[2] * w)
    return depth[y1:y2, x1:x2], y1, y2, x1, x2


def detect_arm_blob(depth_roi, background_roi, threshold=ARM_DEPTH_THRESHOLD):
    """Detect arm pixels as 'closer than background' region.
    Returns (mask_uint8, largest_contour_or_None)."""
    diff = depth_roi.astype(np.float32) - background_roi.astype(np.float32)
    arm_mask = (diff > threshold).astype(np.uint8) * 255
    # Morphological cleanup to remove small noise
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    arm_mask = cv2.morphologyEx(arm_mask, cv2.MORPH_OPEN,  kernel, iterations=1)
    arm_mask = cv2.morphologyEx(arm_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(arm_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return arm_mask, None
    largest = max(contours, key=cv2.contourArea)
    return arm_mask, largest


def most_distal_pixel(contour, roi_x1, roi_y1, full_w=DEPTH_W, full_h=DEPTH_H):
    """Find the contour pixel farthest from the arm-base (image bottom-centre).
    Contour points are in ROI-local coordinates; returned (u,v) is in full-frame coords."""
    ref_x = full_w // 2
    ref_y = full_h          # arm base exits from near the image bottom
    pts = contour.reshape(-1, 2)   # (N, 2) in ROI coords
    # Convert to full-frame coords
    pts_full = pts + np.array([roi_x1, roi_y1], dtype=np.float32)
    dists = np.sqrt((pts_full[:, 0] - ref_x) ** 2 + (pts_full[:, 1] - ref_y) ** 2)
    idx = int(np.argmax(dists))
    u = int(pts_full[idx, 0])
    v = int(pts_full[idx, 1])
    return u, v


def validate_blob(contour, roi_x1, roi_y1, roi_x2, roi_y2):
    """Step 2 validation gate.
    Returns True if blob is large enough and doesn't touch the image boundary."""
    area = cv2.contourArea(contour)
    if area < MIN_BLOB_AREA:
        return False, f"area {area:.0f} < {MIN_BLOB_AREA}"
    # Bounding box in full-frame coordinates
    bx, by, bw, bh = cv2.boundingRect(contour)
    ux1, uy1 = bx + roi_x1, by + roi_y1
    ux2, uy2 = ux1 + bw,    uy1 + bh
    margin = 3  # pixels
    if ux1 < margin or uy1 < margin or ux2 > DEPTH_W - margin or uy2 > DEPTH_H - margin:
        return False, f"touches image edge ({ux1},{uy1})-({ux2},{uy2})"
    return True, "OK"


def fk_gripper_tip(robot, pose_deg):
    """Compute gripper_frame_link position in base_link frame via FK.
    pose_deg: {joint_name: degrees}
    Returns np.ndarray shape (3,) in metres."""
    cfg = {name: deg * math.pi / 180.0 for name, deg in pose_deg.items()}
    robot.update_cfg(cfg)
    T = robot.get_transform(frame_to='gripper_frame_link', frame_from='base_link')
    return np.array(T[:3, 3], dtype=np.float64)


# ─── Main calibration node ──────────────────────────────────────────────────

class CalibrationNode(Node):

    def __init__(self):
        super().__init__('calibrate_camera_arm')

        self._latest_depth = None
        self._depth_count  = 0
        self._latest_js    = None      # {joint_name: rad} — leader positions (Step 3)
        self._is_commanding = False    # gate to ignore our own /joint_states echo

        # Depth subscriber
        self.create_subscription(Image, '/camera/depth/image_raw', self._depth_cb, 5)
        # Joint state subscriber — for manual phase (Step 3)
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        # Arm commander
        self._arm_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.get_logger().info('CalibrationNode ready')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image):
        if msg.encoding != '32FC1':
            return
        depth = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
        self._latest_depth = depth
        self._depth_count += 1

    def _js_cb(self, msg: JointState):
        if self._is_commanding:
            return   # ignore echo of our own commands
        self._latest_js = {name: pos for name, pos in zip(msg.name, msg.position)}

    # ── Utilities ─────────────────────────────────────────────────────────

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_depth(self, timeout=10.0):
        self.get_logger().info('Waiting for /camera/depth/image_raw …')
        start = time.time()
        while self._latest_depth is None:
            if time.time() - start > timeout:
                self.get_logger().error('Timed out waiting for depth frame!')
                return False
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info('Depth frame received.')
        return True

    def collect_depth_frames(self, n, timeout=30.0):
        """Collect n consecutive depth frames. Returns list of np arrays."""
        frames = []
        last_count = self._depth_count
        start = time.time()
        while len(frames) < n:
            if time.time() - start > timeout:
                self.get_logger().warn(f'Timeout collecting frames: got {len(frames)}/{n}')
                break
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._depth_count != last_count:
                frames.append(self._latest_depth.copy())
                last_count = self._depth_count
        return frames

    def command_arm(self, pose_deg, settle_secs=SETTLE_SECS):
        """Publish JointState pose (degrees→radians) at 10Hz for settle_secs."""
        names = list(pose_deg.keys())
        rads  = [deg * math.pi / 180.0 for deg in pose_deg.values()]
        self._is_commanding = True
        end = time.time() + settle_secs
        while time.time() < end:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name     = names
            msg.position = rads
            self._arm_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        self._is_commanding = False

    def get_latest_depth(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        return self._latest_depth.copy() if self._latest_depth is not None else None

    # ── Step 0 ────────────────────────────────────────────────────────────

    def step0_background(self):
        print('\n══ STEP 0 — Background capture ══')
        print(f'Arm must be held still at resting pose for ~{BACKGROUND_FRAMES/14:.0f}s …')
        if not self.wait_for_depth(timeout=15.0):
            sys.exit(1)
        frames = self.collect_depth_frames(BACKGROUND_FRAMES, timeout=30.0)
        if len(frames) < BACKGROUND_FRAMES:
            print(f'WARNING: Only {len(frames)} background frames collected (wanted {BACKGROUND_FRAMES})')
        stack = np.stack(frames, axis=0)
        background = np.median(stack, axis=0).astype(np.float32)
        print(f'Background built — shape {background.shape}, mean depth {background.mean():.4f}')
        return background

    # ── Step 1 ────────────────────────────────────────────────────────────

    def step1_envelope(self, background):
        """Sweep arm through ENVELOPE_POSES, OR-accumulate depth-diff blobs.
        Returns full-frame 518×518 uint8 union mask."""
        print('\n══ STEP 1 — Static envelope sweep ══')
        full_mask = np.zeros((DEPTH_H, DEPTH_W), dtype=np.uint8)
        roi_h = int((ROI_Y2 - ROI_Y1) * DEPTH_H)
        roi_w = int((ROI_X2 - ROI_X1) * DEPTH_W)
        fy1   = int(ROI_Y1 * DEPTH_H)
        fx1   = int(ROI_X1 * DEPTH_W)

        for i, pose in enumerate(ENVELOPE_POSES):
            print(f'  Envelope pose {i+1}/{len(ENVELOPE_POSES)}: {pose}')
            self.command_arm(pose, settle_secs=SETTLE_SECS)
            depth = self.get_latest_depth()
            if depth is None:
                print('  WARNING: no depth frame, skipping')
                continue
            depth_roi = depth[fy1:fy1+roi_h, fx1:fx1+roi_w]
            if depth_roi.shape != background.shape:
                bg_resized = cv2.resize(background, (depth_roi.shape[1], depth_roi.shape[0]),
                                        interpolation=cv2.INTER_LINEAR)
            else:
                bg_resized = background
            arm_mask, contour = detect_arm_blob(depth_roi, bg_resized)
            # OR into union mask (placed in full-frame coords)
            full_mask[fy1:fy1+roi_h, fx1:fx1+roi_w] = (
                full_mask[fy1:fy1+roi_h, fx1:fx1+roi_w] | arm_mask
            )
            n_arm = int(arm_mask.sum() // 255)
            print(f'    → {n_arm} arm pixels detected')

        # Dilate union mask for safety margin
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        full_mask = cv2.dilate(full_mask, kernel, iterations=2)
        cv2.imwrite(MASK_OUT, full_mask)
        print(f'Envelope mask saved → {MASK_OUT}  (non-zero px: {int(full_mask.sum()//255)})')
        return full_mask

    # ── Step 2 ────────────────────────────────────────────────────────────

    def step2_correspondence(self, background, robot):
        """Sweep through CORRESPONDENCE_POSES, collect validated (3D,2D) pairs."""
        print('\n══ STEP 2 — Scripted correspondence sweep ══')
        fy1 = int(ROI_Y1 * DEPTH_H)
        fy2 = int(ROI_Y2 * DEPTH_H)
        fx1 = int(ROI_X1 * DEPTH_W)
        fx2 = int(ROI_X2 * DEPTH_W)
        bg_roi = background   # already ROI-sized

        pairs_3d = []
        pairs_2d = []

        for i, pose in enumerate(CORRESPONDENCE_POSES):
            print(f'  Correspondence pose {i+1}/{len(CORRESPONDENCE_POSES)}: pan={pose["shoulder_pan"]}'
                  f' lift={pose["shoulder_lift"]} elbow={pose["elbow_flex"]}')
            self.command_arm(pose, settle_secs=SETTLE_SECS)
            depth = self.get_latest_depth()
            if depth is None:
                print('    SKIP: no depth frame')
                continue
            depth_roi = depth[fy1:fy2, fx1:fx2]
            if depth_roi.shape != bg_roi.shape:
                bg_roi_rs = cv2.resize(bg_roi, (depth_roi.shape[1], depth_roi.shape[0]),
                                       interpolation=cv2.INTER_LINEAR)
            else:
                bg_roi_rs = bg_roi
            arm_mask, contour = detect_arm_blob(depth_roi, bg_roi_rs)

            if contour is None:
                print('    SKIP: no blob detected')
                continue

            # Validation gate
            ok, reason = validate_blob(contour, fx1, fy1, fx2, fy2)
            if not ok:
                print(f'    SKIP: validation failed — {reason}')
                continue

            # 2D: most distal pixel in full-frame coords
            u, v = most_distal_pixel(contour, fx1, fy1)
            # 3D: FK gripper tip in base_link frame
            pt3d = fk_gripper_tip(robot, pose)

            pairs_3d.append(pt3d)
            pairs_2d.append([float(u), float(v)])
            print(f'    ACCEPTED — blob area {cv2.contourArea(contour):.0f}px'
                  f'  tip=({u},{v})  3D=({pt3d[0]:.3f},{pt3d[1]:.3f},{pt3d[2]:.3f})m')

        print(f'Step 2 complete: {len(pairs_3d)} valid correspondences')
        return pairs_3d, pairs_2d

    # ── Step 3 ────────────────────────────────────────────────────────────

    def step3_manual(self, background, robot, pairs_3d, pairs_2d):
        """Manual phase: operator moves leader arm for ~18s.
        Detect natural pauses (velocity near zero) and attempt correspondence."""
        fy1 = int(ROI_Y1 * DEPTH_H)
        fy2 = int(ROI_Y2 * DEPTH_H)
        fx1 = int(ROI_X1 * DEPTH_W)
        fx2 = int(ROI_X2 * DEPTH_W)
        bg_roi = background
        PAUSE_THRESHOLD = 0.005   # rad — max joint delta to count as "paused"
        PAUSE_MIN_SECS  = 0.5     # minimum seconds of near-zero velocity to trigger
        DURATION_SECS   = 18.0

        print('Go — move the leader arm now')
        print('Moving slowly and pausing at interesting poses gives best results.')

        self._is_commanding = False   # accept /joint_states from leader
        prev_js = None
        pause_start = None
        captured_at_pause = False
        end_time = time.time() + DURATION_SECS

        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)
            js = self._latest_js
            remaining = int(end_time - time.time())
            if remaining % 5 == 0 and remaining > 0:
                # Print countdown every 5 seconds (throttled)
                pass

            if js is None or prev_js is None:
                prev_js = js
                continue

            # Compute max joint velocity
            common_joints = set(js.keys()) & set(prev_js.keys())
            if not common_joints:
                prev_js = js
                continue
            max_delta = max(abs(js[j] - prev_js[j]) for j in common_joints)
            prev_js = js

            if max_delta < PAUSE_THRESHOLD:
                if pause_start is None:
                    pause_start = time.time()
                    captured_at_pause = False
                elif (not captured_at_pause and
                      time.time() - pause_start >= PAUSE_MIN_SECS):
                    # Arm has been paused long enough — try to capture
                    depth = self.get_latest_depth()
                    if depth is None:
                        continue
                    depth_roi = depth[fy1:fy2, fx1:fx2]
                    if depth_roi.shape != bg_roi.shape:
                        bg_rs = cv2.resize(bg_roi, (depth_roi.shape[1], depth_roi.shape[0]),
                                           interpolation=cv2.INTER_LINEAR)
                    else:
                        bg_rs = bg_roi
                    arm_mask, contour = detect_arm_blob(depth_roi, bg_rs)
                    if contour is not None:
                        ok, reason = validate_blob(contour, fx1, fy1, fx2, fy2)
                        if ok:
                            u, v = most_distal_pixel(contour, fx1, fy1)
                            # FK using current leader joint positions
                            pose_rad = {k: v2 for k, v2 in js.items()}
                            pose_deg = {k: v2 * 180.0 / math.pi for k, v2 in pose_rad.items()}
                            pt3d = fk_gripper_tip(robot, pose_deg)
                            pairs_3d.append(pt3d)
                            pairs_2d.append([float(u), float(v)])
                            print(f'  Manual ACCEPTED — tip=({u},{v})'
                                  f'  3D=({pt3d[0]:.3f},{pt3d[1]:.3f},{pt3d[2]:.3f})m')
                        else:
                            print(f'  Manual SKIP — {reason}')
                    captured_at_pause = True
            else:
                pause_start = None
                captured_at_pause = False

        print(f'Manual phase complete. Total pairs so far: {len(pairs_3d)}')
        return pairs_3d, pairs_2d

    # ── Step 4 ────────────────────────────────────────────────────────────

    def step4_solve(self, pairs_3d, pairs_2d):
        """Solve PnP from collected correspondences. Save extrinsics if successful."""
        print(f'\n══ STEP 4 — Solve PnP ({len(pairs_3d)} total correspondences) ══')
        MIN_PAIRS = 8

        if len(pairs_3d) < MIN_PAIRS:
            print(f'Only {len(pairs_3d)} valid correspondences collected '
                  f'({MIN_PAIRS} required) — calibration aborted, '
                  f'envelope mask fallback will be used')
            return False

        K, dist = load_intrinsics(INTRINSICS_FILE)
        print(f'Intrinsics loaded from {INTRINSICS_FILE}')
        print(f'  K (518×518):\n{K}')
        print(f'  dist: {dist}')

        obj_pts = np.array(pairs_3d, dtype=np.float64).reshape(-1, 1, 3)
        img_pts = np.array(pairs_2d, dtype=np.float64).reshape(-1, 1, 2)

        ret, rvec, tvec, inliers = cv2.solvePnPRansac(
            obj_pts, img_pts, K, dist,
            iterationsCount=1000,
            reprojectionError=8.0,
            confidence=0.99,
            flags=cv2.SOLVEPNP_EPNP,
        )
        if not ret:
            print('solvePnPRansac failed — calibration rejected, envelope mask fallback will be used')
            return False

        n_inliers = int(inliers.shape[0]) if inliers is not None else 0
        print(f'solvePnPRansac: {n_inliers}/{len(pairs_3d)} inliers')

        # Compute mean reprojection error on inliers
        if inliers is not None:
            inlier_idx = inliers.flatten()
            obj_in = obj_pts[inlier_idx]
            img_in = img_pts[inlier_idx]
        else:
            obj_in, img_in = obj_pts, img_pts

        proj, _ = cv2.projectPoints(obj_in, rvec, tvec, K, dist)
        errors = np.linalg.norm(proj.reshape(-1, 2) - img_in.reshape(-1, 2), axis=1)
        mean_err = float(errors.mean())
        max_err  = float(errors.max())
        print(f'Reprojection error: mean={mean_err:.2f}px  max={max_err:.2f}px')

        MAX_ERR = 15.0
        if mean_err > MAX_ERR:
            print(f'Reprojection error too high ({mean_err:.2f}px > {MAX_ERR}px) — '
                  f'calibration rejected, envelope mask fallback will be used')
            return False

        # Save extrinsics
        from datetime import datetime
        data = {
            'calibration_timestamp': datetime.now().isoformat(),
            'intrinsics_file': INTRINSICS_FILE,
            'image_size': [DEPTH_W, DEPTH_H],
            'num_correspondences_total': int(len(pairs_3d)),
            'num_inliers': n_inliers,
            'reprojection_error_px': round(mean_err, 3),
            # Rotation as Rodrigues vector [rx, ry, rz] (radians)
            'rotation_vector': rvec.flatten().tolist(),
            # Translation [tx, ty, tz] in metres (arm base origin in camera frame)
            'translation_vector': tvec.flatten().tolist(),
        }
        with open(EXTRIN_OUT, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        print(f'\nCalibration SUCCESS — saved to {EXTRIN_OUT}')
        print(f'  rvec: {rvec.flatten().tolist()}')
        print(f'  tvec: {tvec.flatten().tolist()}')
        print(f'  reprojection error: {mean_err:.2f}px')
        return True


# ─── Entry point ────────────────────────────────────────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Arm-camera calibration script')
    parser.add_argument('--roi-x1', type=float, default=ROI_X1)
    parser.add_argument('--roi-y1', type=float, default=ROI_Y1)
    parser.add_argument('--roi-x2', type=float, default=ROI_X2)
    parser.add_argument('--roi-y2', type=float, default=ROI_Y2)
    args = parser.parse_args()

    # Check prerequisites
    os.makedirs(CALIB_DIR, exist_ok=True)

    if not os.path.exists(URDF_FILE):
        print(f'ERROR: URDF not found at {URDF_FILE}')
        sys.exit(1)
    if not os.path.exists(INTRINSICS_FILE):
        print(f'ERROR: Camera intrinsics not found at {INTRINSICS_FILE}')
        sys.exit(1)

    print('Loading URDF for FK …')
    from yourdfpy import URDF
    robot = URDF.load(URDF_FILE)
    movable = [j.name for j in robot.robot.joints if j.type == 'revolute']
    print(f'  Movable joints: {movable}')

    rclpy.init()
    node = CalibrationNode()

    try:
        # ── Step 0: Background ─────────────────────────────────────────────
        background_full = node.step0_background()
        background_roi, fy1, fy2, fx1, fx2 = extract_roi(background_full)

        # ── Step 1: Envelope sweep ─────────────────────────────────────────
        node.step1_envelope(background_roi)

        # ── Step 2: Correspondence sweep ───────────────────────────────────
        pairs_3d, pairs_2d = node.step2_correspondence(background_roi, robot)

        # ── Step 3: Manual phase — PAUSE HERE ─────────────────────────────
        print('\n══ STEP 3 — Manual calibration phase ══')
        print('  (Handled by caller — pausing here for confirmation)')
        # This is the pause point — the outer driver asks the user
        # The actual call to step3_manual happens after confirmation

    except KeyboardInterrupt:
        print('\nAborted by user.')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    # Pause at Step 3 — ask for confirmation
    print()
    print('━' * 60)
    answer = input(
        'Ready to start the manual calibration phase? Move the leader arm\n'
        'slowly for 15-20 seconds when I say go.\n'
        'Press ENTER when ready (or Ctrl+C to skip Step 3 and go to Step 4): '
    )

    try:
        pairs_3d, pairs_2d = node.step3_manual(background_roi, robot, pairs_3d, pairs_2d)
    except KeyboardInterrupt:
        print('\nManual phase skipped.')

    # ── Step 4: Solve and save ─────────────────────────────────────────────
    try:
        success = node.step4_solve(pairs_3d, pairs_2d)
        if success:
            print('\nCalibration complete. Both mask and extrinsics saved.')
        else:
            print('\nCalibration complete (fallback mode — only envelope mask saved).')
    except KeyboardInterrupt:
        print('\nAborted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
