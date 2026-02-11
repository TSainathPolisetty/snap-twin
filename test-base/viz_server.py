import asyncio
import os
import rclpy
from rclpy.node import Node
from foxglove_sdk import Scene, URDF
from sensor_msgs.msg import JointState

class FoxgloveViz(Node):
    def __init__(self):
        super().__init__('foxglove_viz_server')
        # Initialize Foxglove Scene
        self.scene = Scene(name="SO101 Digital Twin")
        
        # Define paths
        urdf_path = "/home/ubuntu/demo/snap-twin/test-base/final_scene.urdf"
        so101_path = "/home/ubuntu/demo/snap-twin/simulation/models/SO101"
        
        # Load URDF and map the 'SO101' package name to its physical directory
        self.robot = URDF.load(urdf_path, package_path={"SO101": so101_path})
        self.scene.add(self.robot)
        
        # Subscribe to Gazebo joint states
        self.subscription = self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)

    def listener_callback(self, msg):
        # Push Gazebo joint angles to the 3D model
        joint_map = {name: pos for name, pos in zip(msg.name, msg.position)}
        self.robot.update_joints(joint_map)

async def main():
    rclpy.init()
    node = FoxgloveViz()
    # Serve to all interfaces (0.0.0.0) so your laptop can connect
    async with node.scene.run_server(host="0.0.0.0", port=8765):
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            await asyncio.sleep(0.01)

if __name__ == "__main__":
    asyncio.run(main())
