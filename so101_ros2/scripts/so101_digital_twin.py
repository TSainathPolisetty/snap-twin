import os
import time
import rclpy
import numpy as np
import foxglove
from rclpy.node import Node
from sensor_msgs.msg import JointState
from yourdfpy import URDF
from scipy.spatial.transform import Rotation as R
from foxglove.schemas import FrameTransforms, FrameTransform, Vector3, Quaternion

# Absolute Paths
BASE_DIR = "/home/ubuntu/demo/src/snap-twin"
URDF_FILE = os.path.join(BASE_DIR, "final_twin.urdf")

class So101DigitalTwin(Node):
    def __init__(self, robot_model):
        super().__init__('so101_digital_twin')
        self.robot = robot_model
        
        # Identify joints that are actually movable (revolute)
        self.movable_joints = [j.name for j in self.robot.robot.joints if j.type == "revolute"]
        self.current_joint_positions = {name: 0.0 for name in self.movable_joints}
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.get_logger().info(f"Digital Twin Bridge active. Monitoring moving joints: {self.movable_joints}")

    def joint_state_callback(self, msg):
        # Map hardware names directly to URDF names
        for name, position in zip(msg.name, msg.position):
            if name in self.current_joint_positions:
                self.current_joint_positions[name] = position

    def get_foxglove_transforms(self):
        # Update the URDF math engine with the latest motor angles
        self.robot.update_cfg(self.current_joint_positions)
        transforms = []

        for joint in self.robot.robot.joints:
            # FIX: We ONLY broadcast transforms for moving joints. 
            # Static links (table, floor, offsets) are handled by Foxglove reading the URDF.
            if joint.type != "revolute":
                continue

            # Calculate the transformation matrix between the parent and child links
            T_local = self.robot.get_transform(frame_to=joint.child, frame_from=joint.parent)
            trans = T_local[:3, 3]
            
            # Convert the rotation matrix to a quaternion for Foxglove
            rotation_matrix = np.array(T_local[:3, :3], dtype=float, copy=True)
            quat = R.from_matrix(rotation_matrix).as_quat()
            
            transforms.append(FrameTransform(
                parent_frame_id=joint.parent,
                child_frame_id=joint.child,
                translation=Vector3(x=float(trans[0]), y=float(trans[1]), z=float(trans[2])),
                rotation=Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
            ))
        return FrameTransforms(transforms=transforms)

def main():
    rclpy.init()
    
    if not os.path.exists(URDF_FILE):
        print(f"Error: URDF not found at {URDF_FILE}")
        return
    
    # Load the robot model for kinematic math
    robot = URDF.load(URDF_FILE)
    
    bridge_node = So101DigitalTwin(robot)
    
    # Start the Foxglove WebSocket server on port 8765
    server = foxglove.start_server(host="0.0.0.0", port=8765)
    
    try:
        while rclpy.ok():
            # Spin ROS to process incoming /joint_states
            rclpy.spin_once(bridge_node, timeout_sec=0.01)
            
            # Broadcast live movement to Foxglove
            tf_data = bridge_node.get_foxglove_transforms()
            foxglove.log("/tf", tf_data)
            
            # ~50Hz refresh rate
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        server.stop()
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
