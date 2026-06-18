"""
tf_static_relay — republish /tf_static content onto volatile /tf at 2 Hz.

ROOT CAUSE it fixes:
  robot_state_publisher (Humble) publishes fixed-joint transforms on /tf_static
  with TRANSIENT_LOCAL (latching) QoS.  The foxglove-bridge snap (Jazzy build)
  cannot receive that latched content due to a cross-distro FastDDS
  TRANSIENT_LOCAL handshake incompatibility — /tf_static is present and readable
  from any Humble terminal, but the Jazzy bridge silently gets nothing, leaving
  Foxglove Studio's frame dropdown empty.

  This node subscribes to /tf_static from within the same Humble DDS domain
  (so the handshake succeeds), accumulates all received static transforms, and
  republishes them on the VOLATILE /tf topic at 2 Hz.  The foxglove-bridge snap
  subscribes to /tf just fine (confirmed at 16.6 Hz for the revolute joints
  already flowing there).  The result is that all frames — fixed and revolute —
  appear in Foxglove Studio's frame dropdown and the 3D panel correctly shows
  the full kinematic chain.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfStaticRelayNode(Node):

    def __init__(self):
        super().__init__('tf_static_relay')

        # Subscribe to /tf_static with TRANSIENT_LOCAL to receive latched content.
        # This subscription DOES work within Humble DDS (same distro as RSP).
        static_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=100,
        )
        self.create_subscription(TFMessage, '/tf_static', self._static_cb, static_qos)

        # Publish relayed transforms on volatile /tf — foxglove-bridge receives this fine.
        volatile_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=100,
        )
        self._tf_pub = self.create_publisher(TFMessage, '/tf', volatile_qos)

        # Accumulate all static transforms keyed by (parent, child) so duplicates
        # are deduplicated (RSP may re-publish the same transform multiple times).
        self._static_transforms: dict[tuple[str, str], object] = {}

        # Relay timer — 2 Hz is plenty for static frames (they don't change).
        self.create_timer(0.5, self._relay_cb)

        self.get_logger().info(
            'tf_static_relay ready — will relay /tf_static → /tf at 2 Hz '
            'to work around Humble/Jazzy cross-distro TRANSIENT_LOCAL incompatibility'
        )

    def _static_cb(self, msg: TFMessage):
        for transform in msg.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            self._static_transforms[key] = transform
            self.get_logger().info(
                f'tf_static_relay: got static transform '
                f'{transform.header.frame_id} → {transform.child_frame_id}',
                throttle_duration_sec=10.0,
            )

    def _relay_cb(self):
        if not self._static_transforms:
            return
        now = self.get_clock().now().to_msg()
        out = TFMessage()
        for transform in self._static_transforms.values():
            # Stamp with current time so Foxglove doesn't treat them as stale.
            t = type(transform)()
            t.header.stamp = now
            t.header.frame_id = transform.header.frame_id
            t.child_frame_id = transform.child_frame_id
            t.transform = transform.transform
            out.transforms.append(t)
        self._tf_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TfStaticRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
