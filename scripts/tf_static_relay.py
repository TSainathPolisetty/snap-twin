#!/usr/bin/env python3
"""
tf_static_relay.py — republish /tf_static on volatile /tf at 2 Hz.

robot_state_publisher publishes fixed-joint transforms to /tf_static with
TRANSIENT_LOCAL durability.  The foxglove-bridge snap subscribes to /tf_static
with RELIABLE/VOLATILE durability and therefore misses all messages that were
published before the bridge started (late-join problem).

This script subscribes to /tf_static with TRANSIENT_LOCAL (so it receives the
full cached history immediately on startup) and then re-publishes the accumulated
set of transforms onto /tf at 2 Hz with VOLATILE durability.  foxglove-bridge
receives those messages and can populate its frame list.

Usage (background process in start_gesture_demo.sh):
    python3 scripts/tf_static_relay.py &
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf2_msgs.msg import TFMessage


class TFStaticRelay(Node):

    def __init__(self):
        super().__init__('tf_static_relay')

        transient_qos = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile_qos = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._transforms = {}  # child_frame_id → TransformStamped
        self._pub = self.create_publisher(TFMessage, '/tf', volatile_qos)
        self.create_subscription(TFMessage, '/tf_static', self._static_cb, transient_qos)
        self.create_timer(0.5, self._relay_tick)  # 2 Hz
        self.get_logger().info('tf_static_relay ready — republishing /tf_static → /tf at 2 Hz')

    def _static_cb(self, msg: TFMessage):
        for t in msg.transforms:
            self._transforms[t.child_frame_id] = t

    def _relay_tick(self):
        if not self._transforms:
            return
        out = TFMessage()
        now = self.get_clock().now().to_msg()
        for t in self._transforms.values():
            t.header.stamp = now
            out.transforms.append(t)
        self._pub.publish(out)


def main():
    rclpy.init()
    node = TFStaticRelay()
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
