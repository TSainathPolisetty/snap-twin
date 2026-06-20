"""
Overhead HSV segmentation node.

Owns the Brio 101 overhead camera on /dev/video0, segments the orange arm and a
green prop (for backprojection/simulation), and detects a lavender Gengar plushie
as the workspace obstacle trigger.
"""

import array
import contextlib
import io
import math
import os

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

BASE_DIR = (
    os.environ.get('SNAP_TWIN_DATA_DIR')
    or os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'share'))
)
EXTRINSICS_PATH = os.path.join(BASE_DIR, 'calibration', 'camera_extrinsics.yaml')
INTRINSICS_PATH = os.path.expanduser('~/.ros/camera_info/brio.yaml')
URDF_PATH = os.path.join(BASE_DIR, 'final_twin.urdf')
PLACEHOLDER_TABLE_HEIGHT = -999.0
CONSISTENCY_THRESHOLD_PX = 80.0


class OverheadVisionNode(Node):

    def __init__(self):
        super().__init__('overhead_vision_node')

        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('arm_hue_low', 5)
        self.declare_parameter('arm_hue_high', 25)
        self.declare_parameter('arm_sat_low', 120)
        self.declare_parameter('arm_sat_high', 255)
        self.declare_parameter('arm_val_low', 80)
        self.declare_parameter('arm_val_high', 255)
        self.declare_parameter('prop_hue_low', 40)   # green: hue 40-80 in OpenCV HSV (0-180)
        self.declare_parameter('prop_hue_high', 80)
        self.declare_parameter('prop_sat_low', 60)
        self.declare_parameter('prop_sat_high', 255)
        self.declare_parameter('prop_val_low', 40)
        self.declare_parameter('prop_val_high', 255)
        self.declare_parameter('roi_x1', 0.15)
        self.declare_parameter('roi_y1', 0.15)
        self.declare_parameter('roi_x2', 0.85)
        self.declare_parameter('roi_y2', 0.85)
        self.declare_parameter('min_blob_area', 500)
        self.declare_parameter('table_height_m', PLACEHOLDER_TABLE_HEIGHT)
        # Gengar plushie detector — lavender/purple obstacle trigger.
        # These are starting estimates for a lavender Gengar plush toy under show lighting;
        # tune gengar_hue_low/high on-site if detection is poor or noisy.
        # OpenCV HSV: hue 0-180 (purple ≈ 120-150), sat/val 0-255.
        # All gengar_* params support live tuning via `ros2 param set`.
        self.declare_parameter('gengar_hue_low',  120)
        self.declare_parameter('gengar_hue_high', 155)
        self.declare_parameter('gengar_sat_low',   50)
        self.declare_parameter('gengar_sat_high', 255)
        self.declare_parameter('gengar_val_low',   40)
        self.declare_parameter('gengar_val_high', 255)
        # Separate minimum blob area for Gengar (higher than arm/prop to suppress noise)
        self.declare_parameter('gengar_min_area', 3000)

        self._camera_device = str(self.get_parameter('camera_device').value)
        self._arm_lower = np.array([
            int(self.get_parameter('arm_hue_low').value),
            int(self.get_parameter('arm_sat_low').value),
            int(self.get_parameter('arm_val_low').value),
        ], dtype=np.uint8)
        self._arm_upper = np.array([
            int(self.get_parameter('arm_hue_high').value),
            int(self.get_parameter('arm_sat_high').value),
            int(self.get_parameter('arm_val_high').value),
        ], dtype=np.uint8)
        self._prop_lower = np.array([
            int(self.get_parameter('prop_hue_low').value),
            int(self.get_parameter('prop_sat_low').value),
            int(self.get_parameter('prop_val_low').value),
        ], dtype=np.uint8)
        self._prop_upper = np.array([
            int(self.get_parameter('prop_hue_high').value),
            int(self.get_parameter('prop_sat_high').value),
            int(self.get_parameter('prop_val_high').value),
        ], dtype=np.uint8)
        self._roi_x1 = float(self.get_parameter('roi_x1').value)
        self._roi_y1 = float(self.get_parameter('roi_y1').value)
        self._roi_x2 = float(self.get_parameter('roi_x2').value)
        self._roi_y2 = float(self.get_parameter('roi_y2').value)
        self._min_blob_area = float(self.get_parameter('min_blob_area').value)
        self._table_height_m = float(self.get_parameter('table_height_m').value)
        # Gengar HSV bounds (live-tunable via ros2 param set)
        self._gengar_lower = np.array([
            int(self.get_parameter('gengar_hue_low').value),
            int(self.get_parameter('gengar_sat_low').value),
            int(self.get_parameter('gengar_val_low').value),
        ], dtype=np.uint8)
        self._gengar_upper = np.array([
            int(self.get_parameter('gengar_hue_high').value),
            int(self.get_parameter('gengar_sat_high').value),
            int(self.get_parameter('gengar_val_high').value),
        ], dtype=np.uint8)
        self._gengar_min_area = float(self.get_parameter('gengar_min_area').value)

        self._current_joint_positions = {}
        self._gesture_active = False
        self._scaled_K = None
        self._dist = None
        self._frame_w = None
        self._frame_h = None
        self._camera_opened = False
        self._no_calibration_warned = False
        self._extrinsics_loaded = False
        self._intrinsics_loaded = False
        self._robot = None
        self._rvec = None
        self._tvec = None
        self._rotation = None
        self._camera_origin_base = None

        self._annotated_pub = self.create_publisher(Image, '/overhead/image_annotated', 5)
        self._consistency_pub = self.create_publisher(String, '/overhead/consistency_status', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        self._obstacle_present_pub = self.create_publisher(Bool, '/overhead/obstacle_present', 10)

        self.create_subscription(JointState, '/joint_states', self._joint_states_cb, 10)
        self.create_subscription(Bool, '/gesture_active', self._gesture_active_cb, 10)
        self.create_subscription(JointState, '/gesture/joint_states', self._gesture_joint_states_cb, 10)

        # Parameter callback — allows live tuning of Gengar HSV bounds via ros2 param set
        self.add_on_set_parameters_callback(self._on_gengar_params_changed)

        self._load_extrinsics()
        self._open_camera(self._camera_device)
        self._load_intrinsics()
        self._load_robot()

        self.create_timer(1.0 / 15.0, self._timer_cb)
        self.get_logger().info(
            'OverheadVisionNode ready — arm HSV orange, prop HSV green, Gengar HSV purple; '
            'live-tune with: ros2 param set /overhead_vision_node gengar_hue_low <val>'
        )

    def _load_extrinsics(self):
        if not os.path.exists(EXTRINSICS_PATH):
            self.get_logger().warn(
                f'camera_extrinsics.yaml missing at {EXTRINSICS_PATH} - sim population and consistency check disabled'
            )
            self._no_calibration_warned = True
            return
        try:
            with open(EXTRINSICS_PATH, 'r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
            rvec_data = data.get('rvec', data.get('rotation_vector'))
            tvec_data = data.get('tvec', data.get('translation_vector'))
            if rvec_data is None or tvec_data is None:
                raise KeyError('expected rvec/tvec or rotation_vector/translation_vector')
            self._rvec = np.array(rvec_data, dtype=np.float64).reshape(3, 1)
            self._tvec = np.array(tvec_data, dtype=np.float64).reshape(3, 1)
            self._rotation, _ = cv2.Rodrigues(self._rvec)
            self._camera_origin_base = (-self._rotation.T @ self._tvec).reshape(3)
            self._extrinsics_loaded = True
            self.get_logger().info(
                f'Loaded overhead extrinsics from {EXTRINSICS_PATH} '
                f'(reprojection_error_px={data.get("reprojection_error_px", "?")})'
            )
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to load camera_extrinsics.yaml: {exc} - sim population and consistency check disabled'
            )
            self._no_calibration_warned = True

    def _open_camera(self, device: str):
        gst_pipeline = (
            f'v4l2src device={device} io-mode=mmap ! '
            'image/jpeg,width=1920,height=1080,framerate=30/1 ! '
            'jpegdec ! videoconvert ! '
            'appsink drop=true max-buffers=1'
        )
        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            self.get_logger().warn(
                f'GStreamer pipeline failed for {device} - trying plain VideoCapture'
            )
            dev_index = int(device.replace('/dev/video', '')) if '/dev/video' in device else 0
            cap = cv2.VideoCapture(dev_index)
            # Request 1920×1080 MJPEG — many USB cameras (incl. Brio) support it
            # without GStreamer. Without this the fallback defaults to 640×480 which
            # makes gengar_min_area (tuned for 1920×1080) too large to detect.
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        if not cap.isOpened():
            self.get_logger().error(f'Cannot open overhead camera {device}')
            self._cap = None
            return

        ret, frame = cap.read()
        if ret and frame is not None:
            self._frame_h, self._frame_w = frame.shape[:2]
        else:
            self._frame_h, self._frame_w = 1080, 1920
        self._cap = cap
        self._camera_opened = True
        self.get_logger().info(f'Overhead camera opened: {self._frame_w}x{self._frame_h}')

    def _load_intrinsics(self):
        if not os.path.exists(INTRINSICS_PATH):
            self.get_logger().warn(
                f'brio.yaml missing at {INTRINSICS_PATH} - sim population and consistency check disabled'
            )
            return
        if self._frame_w is None or self._frame_h is None:
            self.get_logger().warn('Camera frame size unknown - intrinsics scaling deferred')
            return
        try:
            with open(INTRINSICS_PATH, 'r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle)
            orig_w = float(data['image_width'])
            orig_h = float(data['image_height'])
            camera_matrix = data['camera_matrix']['data']
            fx, fy, cx, cy = camera_matrix[0], camera_matrix[4], camera_matrix[2], camera_matrix[5]
            sx = self._frame_w / orig_w
            sy = self._frame_h / orig_h
            self._scaled_K = np.array([
                [fx * sx, 0.0, cx * sx],
                [0.0, fy * sy, cy * sy],
                [0.0, 0.0, 1.0],
            ], dtype=np.float64)
            self._dist = np.array(data['distortion_coefficients']['data'], dtype=np.float64).reshape(1, -1)
            self._intrinsics_loaded = True
            self.get_logger().info(f'Loaded scaled overhead intrinsics from {INTRINSICS_PATH}')
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to load brio intrinsics: {exc} - sim population and consistency check disabled'
            )

    def _load_robot(self):
        if not os.path.exists(URDF_PATH):
            self.get_logger().warn(f'URDF not found at {URDF_PATH} - consistency check disabled')
            return
        try:
            from yourdfpy import URDF
            with contextlib.redirect_stderr(io.StringIO()):
                self._robot = URDF.load(URDF_PATH)
        except Exception as exc:
            self.get_logger().warn(f'Failed to load URDF for consistency check: {exc}')
            self._robot = None

    def _joint_states_cb(self, msg: JointState):
        if self._gesture_active:
            return
        for name, position in zip(msg.name, msg.position):
            self._current_joint_positions[name] = position

    def _gesture_active_cb(self, msg: Bool):
        self._gesture_active = msg.data

    def _gesture_joint_states_cb(self, msg: JointState):
        if not self._gesture_active:
            return
        for name, position in zip(msg.name, msg.position):
            self._current_joint_positions[name] = position

    def _on_gengar_params_changed(self, params):
        """Live parameter callback — updates Gengar HSV bounds without restarting the node."""
        from rcl_interfaces.msg import SetParametersResult
        gengar_names = {
            'gengar_hue_low', 'gengar_hue_high',
            'gengar_sat_low', 'gengar_sat_high',
            'gengar_val_low', 'gengar_val_high',
            'gengar_min_area',
        }
        changed = [p for p in params if p.name in gengar_names]
        if changed:
            for p in changed:
                if p.name == 'gengar_hue_low':
                    self._gengar_lower[0] = int(p.value)
                elif p.name == 'gengar_sat_low':
                    self._gengar_lower[1] = int(p.value)
                elif p.name == 'gengar_val_low':
                    self._gengar_lower[2] = int(p.value)
                elif p.name == 'gengar_hue_high':
                    self._gengar_upper[0] = int(p.value)
                elif p.name == 'gengar_sat_high':
                    self._gengar_upper[1] = int(p.value)
                elif p.name == 'gengar_val_high':
                    self._gengar_upper[2] = int(p.value)
                elif p.name == 'gengar_min_area':
                    self._gengar_min_area = float(p.value)
            self.get_logger().info(
                f'Gengar HSV updated: lower={self._gengar_lower.tolist()} '
                f'upper={self._gengar_upper.tolist()} min_area={self._gengar_min_area}'
            )
        return SetParametersResult(successful=True)

    def _timer_cb(self):
        if self._cap is None or not self._cap.isOpened():
            self.get_logger().warn('Overhead camera unavailable - skipping frame', throttle_duration_sec=5.0)
            return

        ret, frame = self._cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Overhead camera read failed - skipping frame', throttle_duration_sec=2.0)
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        arm_blob = self._extract_blob(hsv, self._arm_lower, self._arm_upper)
        prop_blob = self._extract_blob(hsv, self._prop_lower, self._prop_upper)
        gengar_blob = self._extract_blob(hsv, self._gengar_lower, self._gengar_upper,
                                          min_area=self._gengar_min_area)

        # Gengar plushie drives /overhead/obstacle_present (replaces bg-diff)
        gengar_in_roi = self._blob_in_roi(gengar_blob, frame.shape[1], frame.shape[0])
        self._publish_bool(self._obstacle_present_pub, gengar_in_roi)

        self._publish_marker(prop_blob)
        consistency_text = self._compute_consistency_status(arm_blob)
        self._publish_string(self._consistency_pub, consistency_text)

        annotated = self._annotate_frame(frame, arm_blob, prop_blob, gengar_blob, consistency_text)
        self._publish_image(annotated)

    def _extract_blob(self, hsv: np.ndarray, lower: np.ndarray, upper: np.ndarray,
                      min_area: float = None):
        mask = self._threshold_hsv(hsv, lower, upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(largest))
        threshold = min_area if min_area is not None else self._min_blob_area
        if area < threshold:
            return None
        x, y, w, h = cv2.boundingRect(largest)
        # Reject blobs with extreme aspect ratios — Gengar is roughly circular,
        # not a thin strip (catches frame-edge noise and elongated shadows)
        if w > 0 and h > 0:
            ratio = max(w / h, h / w)
            if ratio > 4.0:
                return None
        moments = cv2.moments(largest)
        if abs(moments['m00']) < 1e-6:
            return None
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        return {
            'contour': largest,
            'area': area,
            'centroid': (cx, cy),
            'bbox': (x, y, w, h),
        }

    def _threshold_hsv(self, hsv: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
        if int(lower[0]) <= int(upper[0]):
            return cv2.inRange(hsv, lower, upper)
        low_a = np.array([0, lower[1], lower[2]], dtype=np.uint8)
        high_a = np.array([upper[0], upper[1], upper[2]], dtype=np.uint8)
        low_b = np.array([lower[0], lower[1], lower[2]], dtype=np.uint8)
        high_b = np.array([179, upper[1], upper[2]], dtype=np.uint8)
        return cv2.bitwise_or(cv2.inRange(hsv, low_a, high_a), cv2.inRange(hsv, low_b, high_b))

    def _blob_in_roi(self, blob, frame_w: int, frame_h: int) -> bool:
        if blob is None:
            return False
        cx, cy = blob['centroid']
        x1 = int(self._roi_x1 * frame_w)
        y1 = int(self._roi_y1 * frame_h)
        x2 = int(self._roi_x2 * frame_w)
        y2 = int(self._roi_y2 * frame_h)
        return x1 <= cx <= x2 and y1 <= cy <= y2

    def _delete_prop_marker(self):
        """Publish a DELETE action for the prop obstacle marker."""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.ns = 'overhead_prop'
        marker.id = 0
        marker.action = Marker.DELETE
        msg = MarkerArray()
        msg.markers.append(marker)
        self._marker_pub.publish(msg)

    def _publish_marker(self, prop_blob):
        self.get_logger().debug(
            f'[diag] prop_blob found: {prop_blob is not None}'
            + (f' area={prop_blob["area"]:.0f} centroid={prop_blob["centroid"]}' if prop_blob else ''),
        )

        if (prop_blob is None
                or not (self._extrinsics_loaded and self._intrinsics_loaded)
                or math.isclose(self._table_height_m, PLACEHOLDER_TABLE_HEIGHT, abs_tol=1e-9)):
            if (prop_blob is not None
                    and math.isclose(self._table_height_m, PLACEHOLDER_TABLE_HEIGHT, abs_tol=1e-9)):
                self.get_logger().warn(
                    'table_height_m is still -999.0 - set it during setup before backprojection can run',
                    throttle_duration_sec=5.0,
                )
            self._delete_prop_marker()
            return

        point_base = self._backproject_to_table(prop_blob['centroid'])
        if point_base is None:
            self._delete_prop_marker()
            return

        marker_array = MarkerArray()
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'base_link'
        marker.ns = 'overhead_prop'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(point_base[0])
        marker.pose.position.y = float(point_base[1])
        # Marker z = table_height_m by backprojection construction.
        # Add a small upward offset so it renders visibly above the base_link in Foxglove
        # rather than at/below the floor plane when calibration places the table near z=0.
        _MARKER_Z_OFFSET = 0.05  # metres — adjustable if marker appears buried
        marker.pose.position.z = float(point_base[2]) + _MARKER_Z_OFFSET
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.06
        marker.scale.y = 0.06
        marker.scale.z = 0.06
        marker.color.r = 0.1
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.85
        marker.lifetime = Duration(seconds=1.0).to_msg()
        marker_array.markers.append(marker)

        self.get_logger().debug(
            f'[diag] marker pos base_link: x={point_base[0]:.4f} y={point_base[1]:.4f} '
            f'z={point_base[2]:.4f} | table_height_m={self._table_height_m}',
        )

        self._marker_pub.publish(marker_array)

    def _backproject_to_table(self, pixel_xy):
        if self._scaled_K is None or self._rotation is None or self._tvec is None:
            return None
        u, v = pixel_xy
        ray_cam = np.linalg.inv(self._scaled_K) @ np.array([u, v, 1.0], dtype=np.float64)
        ray_base = self._rotation.T @ ray_cam.reshape(3, 1)
        ray_base = ray_base.reshape(3)
        denom = float(ray_base[2])
        if abs(denom) < 1e-9:
            return None
        camera_origin = self._camera_origin_base
        scale = (self._table_height_m - float(camera_origin[2])) / denom
        if scale <= 0.0:
            return None
        return camera_origin + scale * ray_base

    def _compute_consistency_status(self, arm_blob) -> str:
        if not self._extrinsics_loaded or not self._intrinsics_loaded or self._robot is None:
            return 'NO CALIBRATION - consistency check disabled'
        if not self._current_joint_positions:
            return 'WAITING FOR JOINTS'
        if arm_blob is None:
            return 'ARM NOT VISIBLE'
        tip_base = self._fk_gripper_tip()
        if tip_base is None:
            return 'WAITING FOR JOINTS'
        projected = self._project_point(tip_base)
        if projected is None:
            return 'NO CALIBRATION - consistency check disabled'
        ax, ay = arm_blob['centroid']
        px, py = projected
        error_px = float(np.hypot(px - ax, py - ay))
        if error_px <= CONSISTENCY_THRESHOLD_PX:
            return f'OK - {error_px:.1f}px'
        return f'MISMATCH - {error_px:.1f}px, check calibration or arm fault'

    def _fk_gripper_tip(self):
        if self._robot is None or not self._current_joint_positions:
            return None
        try:
            self._robot.update_cfg(dict(self._current_joint_positions))
            transform = self._robot.get_transform(frame_to='gripper_frame_link', frame_from='base_link')
            return np.array(transform[:3, 3], dtype=np.float64)
        except Exception as exc:
            self.get_logger().warn(f'Consistency FK failed: {exc}', throttle_duration_sec=5.0)
            return None

    def _project_point(self, point_base):
        if self._scaled_K is None or self._rvec is None or self._tvec is None:
            return None
        projected, _ = cv2.projectPoints(
            np.array(point_base, dtype=np.float64).reshape(1, 1, 3),
            self._rvec,
            self._tvec,
            self._scaled_K,
            self._dist,
        )
        u, v = projected.reshape(2)
        return float(u), float(v)

    def _annotate_frame(self, frame: np.ndarray, arm_blob, prop_blob, gengar_blob,
                        consistency_text: str) -> np.ndarray:
        annotated = frame.copy()
        frame_h, frame_w = annotated.shape[:2]

        if prop_blob is not None:
            pass  # prop detection kept for sim population / obstacle markers; no visual overlay

        # Gengar plushie — magenta bounding box
        if gengar_blob is not None:
            gx, gy, gw, gh = gengar_blob['bbox']
            cv2.rectangle(annotated, (gx, gy), (gx + gw, gy + gh), (255, 0, 255), 2)
            cv2.putText(
                annotated,
                'GENGAR',
                (gx, max(20, gy - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            annotated,
            consistency_text,
            (16, max(28, frame_h - 16)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return annotated

    def _publish_image(self, bgr_image: np.ndarray):
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.height = rgb_image.shape[0]
        msg.width = rgb_image.shape[1]
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = rgb_image.shape[1] * 3
        msg.data = array.array('B', rgb_image.tobytes())
        self._annotated_pub.publish(msg)

    def _publish_bool(self, publisher, value: bool):
        msg = Bool()
        msg.data = value
        publisher.publish(msg)

    def _publish_string(self, publisher, value: str):
        msg = String()
        msg.data = value
        publisher.publish(msg)

    def destroy_node(self):
        if hasattr(self, '_cap') and self._cap is not None and self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OverheadVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
