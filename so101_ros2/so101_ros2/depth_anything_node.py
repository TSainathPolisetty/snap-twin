"""
Depth Anything V2 inference node — Jetson Orin NX
===================================================
Owns camera capture (GStreamer → cv2) and TensorRT fp16 inference.

Publishes:
  /camera/depth/image_raw         sensor_msgs/Image  encoding=32FC1  ~15 Hz
  /camera/depth/visualization     sensor_msgs/Image  encoding=rgb8   ~15 Hz
  /camera/image_raw               sensor_msgs/Image  encoding=rgb8   ~15 Hz  (raw camera frame)

Parameters:
  engine_path     (string) path to .engine file
  camera_device   (string) /dev/videoN device path
  publish_width   (int)    publish resolution width  (default 518 = TRT native)
  publish_height  (int)    publish resolution height (default 518 = TRT native)

NOTE: publish resolution defaults to the TRT native output (518×518) for
performance.  Upscaling to camera native (1920×1080) would produce ~14 MB
messages and reduce frame rate to ~2.5 Hz.  The collision_checker and
Foxglove visualisation both work fine at 518×518.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import array
import numpy as np
import cv2
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit  # noqa: F401 — initialises CUDA context for this thread


# ImageNet normalisation constants (float32 broadcast-ready)
_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class DepthAnythingNode(Node):

    def __init__(self):
        super().__init__('depth_anything_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('engine_path',
                               '/home/ubuntu/models/depth_anything_v2_small.engine')
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('publish_width',  518)
        self.declare_parameter('publish_height', 518)

        engine_path      = self.get_parameter('engine_path').value
        camera_device    = self.get_parameter('camera_device').value
        self._pub_w      = int(self.get_parameter('publish_width').value)
        self._pub_h      = int(self.get_parameter('publish_height').value)

        # ── Publishers ──────────────────────────────────────────────────────
        self._depth_pub = self.create_publisher(Image, '/camera/depth/image_raw',      5)
        self._vis_pub   = self.create_publisher(Image, '/camera/depth/visualization',  5)
        self._raw_pub   = self.create_publisher(Image, '/camera/image_raw',            5)

        # ── TensorRT engine + buffer allocation ─────────────────────────────
        self.get_logger().info(f'Loading TRT engine: {engine_path}')
        self._load_engine(engine_path)

        # ── Camera ──────────────────────────────────────────────────────────
        self.get_logger().info(f'Opening camera: {camera_device}')
        self._open_camera(camera_device)

        # ── 15 Hz inference timer ────────────────────────────────────────────
        self._frame_count = 0
        self.create_timer(1.0 / 15.0, self._timer_cb)
        self.get_logger().info(
            f'DepthAnythingNode ready — publishing at ~15 Hz  '
            f'({self._pub_w}×{self._pub_h})'
        )

    # ────────────────────────────────────────────────────────────────────────
    # Initialisation helpers
    # ────────────────────────────────────────────────────────────────────────

    def _load_engine(self, engine_path: str):
        """Deserialise the TRT engine and allocate page-locked buffers once."""
        trt_logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, 'rb') as f:
            runtime = trt.Runtime(trt_logger)
            self._engine  = runtime.deserialize_cuda_engine(f.read())

        self._trt_ctx = self._engine.create_execution_context()
        self._stream  = cuda.Stream()

        # Input:  pixel_values  (1, 3, 518, 518)  FLOAT32
        in_numel  = 1 * 3 * 518 * 518
        self._h_input  = cuda.pagelocked_empty(in_numel, np.float32)
        self._d_input  = cuda.mem_alloc(in_numel * 4)   # 4 bytes per float32

        # Output: predicted_depth (1, 518, 518)  FLOAT32
        out_numel = 1 * 518 * 518
        self._h_output = cuda.pagelocked_empty(out_numel, np.float32)
        self._d_output = cuda.mem_alloc(out_numel * 4)

        # Set tensor addresses once — TensorRT 10 API (no bindings API)
        self._trt_ctx.set_tensor_address('pixel_values',    int(self._d_input))
        self._trt_ctx.set_tensor_address('predicted_depth', int(self._d_output))

        self.get_logger().info('TRT engine loaded — buffers allocated')

    def _open_camera(self, device: str):
        """Open camera via GStreamer MJPG pipeline; fall back to plain VideoCapture."""
        gst_pipeline = (
            f'v4l2src device={device} io-mode=mmap ! '
            'image/jpeg,width=1920,height=1080,framerate=30/1 ! '
            'jpegdec ! videoconvert ! '
            'appsink drop=true max-buffers=1'
        )
        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            self.get_logger().warn(
                f'GStreamer pipeline failed for {device} — trying plain VideoCapture')
            dev_index = int(device.replace('/dev/video', '')) if '/dev/video' in device else 0
            cap = cv2.VideoCapture(dev_index)

        if not cap.isOpened():
            self.get_logger().error(f'Cannot open camera {device}')
            self._cap = None
            return

        # Warm up: one frame read to confirm pipeline is live
        ret, frame = cap.read()
        cam_h, cam_w = (frame.shape[:2] if ret else (1080, 1920))

        self._cap = cap
        self.get_logger().info(
            f'Camera opened: {cam_w}×{cam_h}  →  publishing at {self._pub_w}×{self._pub_h}'
        )

    # ────────────────────────────────────────────────────────────────────────
    # Preprocessing / inference
    # ────────────────────────────────────────────────────────────────────────

    def _preprocess(self, bgr_frame: np.ndarray) -> np.ndarray:
        """BGR frame → CHW float32 tensor ready for the TRT input buffer."""
        rgb     = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (518, 518), interpolation=cv2.INTER_LINEAR)
        x       = resized.astype(np.float32) / 255.0
        x       = (x - _MEAN) / _STD           # ImageNet normalise (HWC)
        x       = np.ascontiguousarray(x.transpose(2, 0, 1))   # CHW
        return x

    def _infer(self, chw_input: np.ndarray) -> np.ndarray:
        """Run one forward pass; return raw (518, 518) depth output."""
        np.copyto(self._h_input, chw_input.ravel())
        cuda.memcpy_htod_async(self._d_input,  self._h_input,  self._stream)
        self._trt_ctx.execute_async_v3(stream_handle=self._stream.handle)
        cuda.memcpy_dtoh_async(self._h_output, self._d_output, self._stream)
        self._stream.synchronize()
        return self._h_output.reshape(518, 518).copy()

    # ────────────────────────────────────────────────────────────────────────
    # Timer callback — capture + infer + publish
    # ────────────────────────────────────────────────────────────────────────

    def _timer_cb(self):
        if self._cap is None or not self._cap.isOpened():
            self.get_logger().warn('Camera unavailable — skipping frame', throttle_duration_sec=5.0)
            return

        ret, frame = self._cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Camera read failed — skipping frame', throttle_duration_sec=2.0)
            return

        # ── Inference ────────────────────────────────────────────────────────
        chw   = self._preprocess(frame)
        depth = self._infer(chw)           # (518, 518) raw relative depth

        # Normalise to [0, 1]  (higher = closer, disparity-like)
        d_min, d_max = float(depth.min()), float(depth.max())
        if d_max - d_min < 1e-6:
            depth_norm = np.zeros_like(depth)
        else:
            depth_norm = (depth - d_min) / (d_max - d_min)

        # Resize to publish resolution if different from TRT output (518×518)
        if self._pub_w != 518 or self._pub_h != 518:
            depth_out = cv2.resize(depth_norm, (self._pub_w, self._pub_h),
                                   interpolation=cv2.INTER_LINEAR)
        else:
            depth_out = depth_norm

        now = self.get_clock().now().to_msg()

        # ── Publish 32FC1 depth ───────────────────────────────────────────────
        depth_msg                = Image()
        depth_msg.header.stamp   = now
        depth_msg.header.frame_id = 'camera_frame'
        depth_msg.height         = self._pub_h
        depth_msg.width          = self._pub_w
        depth_msg.encoding       = '32FC1'
        depth_msg.is_bigendian   = False
        depth_msg.step           = self._pub_w * 4   # 4 bytes / pixel
        # array.array('B', ...) avoids 175ms byte-by-byte Python iteration
        depth_msg.data           = array.array('B', depth_out.astype(np.float32).tobytes())
        self._depth_pub.publish(depth_msg)

        # ── Publish colorised visualisation (rgb8) ───────────────────────────
        depth_u8   = (depth_out * 255).astype(np.uint8)
        color_bgr  = cv2.applyColorMap(depth_u8, cv2.COLORMAP_INFERNO)
        rgb_color  = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)

        vis_msg                = Image()
        vis_msg.header.stamp   = now
        vis_msg.header.frame_id = 'camera_frame'
        vis_msg.height         = self._pub_h
        vis_msg.width          = self._pub_w
        vis_msg.encoding       = 'rgb8'
        vis_msg.is_bigendian   = False
        vis_msg.step           = self._pub_w * 3
        vis_msg.data           = array.array('B', rgb_color.tobytes())
        self._vis_pub.publish(vis_msg)

        # ── Publish raw RGB camera frame (resized to publish resolution) ──────
        rgb_raw = cv2.cvtColor(
            cv2.resize(frame, (self._pub_w, self._pub_h), interpolation=cv2.INTER_LINEAR),
            cv2.COLOR_BGR2RGB)
        raw_msg                = Image()
        raw_msg.header.stamp   = now
        raw_msg.header.frame_id = 'camera_frame'
        raw_msg.height         = self._pub_h
        raw_msg.width          = self._pub_w
        raw_msg.encoding       = 'rgb8'
        raw_msg.is_bigendian   = False
        raw_msg.step           = self._pub_w * 3
        raw_msg.data           = array.array('B', rgb_raw.tobytes())
        self._raw_pub.publish(raw_msg)

        self._frame_count += 1
        if self._frame_count % 30 == 0:
            self.get_logger().info(f'Depth inference running — frame {self._frame_count}')

    # ────────────────────────────────────────────────────────────────────────
    # Cleanup
    # ────────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        if hasattr(self, '_cap') and self._cap is not None and self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DepthAnythingNode()
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
