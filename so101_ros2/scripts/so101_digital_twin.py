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
#URDF_FILE = os.path.join(BASE_DIR, "so101_ros2/config/app_follower.urdf") 
URDF_FILE = os.path.join(BASE_DIR, "so101_ros2/config/resolved_app.urdf")
PACKAGE_ROOT = os.path.join(BASE_DIR, "lerobot_description")

class So101DigitalTwin(Node):
    def __init__(self, robot_model):
        super().__init__('so101_digital_twin')
        self.robot = robot_model
        self.movable_joints = [j.name for j in self.robot.robot.joints if j.type != "fixed"]
        self.current_joint_positions = {name: 0.0 for name in self.movable_joints}
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.get_logger().info(f"Digital Twin Bridge active. Monitoring: {self.movable_joints}")

    #def joint_state_callback(self, msg):
     #   for name, position in zip(msg.name, msg.position):
    #        if name in self.current_joint_positions:
    #            self.current_joint_positions[name] = position
    
    def joint_state_callback(self, msg):
        # Map hardware names to URDF joint IDs
        # Mapping: Hardware Name -> URDF ID
        name_map = {
            "shoulder_pan": "1",
            "shoulder_lift": "2",
            "elbow_flex": "3",
            "wrist_flex": "4",
            "wrist_roll": "5",
            "gripper": "6"
        }

        for name, position in zip(msg.name, msg.position):
            # Translate hardware name to URDF ID (e.g., "shoulder_pan" -> "1")
            urdf_name = name_map.get(name, name) 
            
            if urdf_name in self.current_joint_positions:
                self.current_joint_positions[urdf_name] = position


    def get_foxglove_transforms(self):
        self.robot.update_cfg(self.current_joint_positions)
        transforms = []

        for joint in self.robot.robot.joints:
            T_local = self.robot.get_transform(frame_to=joint.child, frame_from=joint.parent)
            trans = T_local[:3, 3]
            
            # Writable copy for Scipy
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
    
    robot = URDF.load(URDF_FILE)
    
    # Path resolution for internal kinematics engine
    for link in robot.robot.links:
        for visual in link.visuals:
            if visual.geometry.mesh:
                fname = visual.geometry.mesh.filename
                if not fname.startswith('/'):
                    visual.geometry.mesh.filename = os.path.join(PACKAGE_ROOT, fname)
    
    bridge_node = So101DigitalTwin(robot)
    
    #Start the server
    server = foxglove.start_server(host="0.0.0.0", port=8765)
    
    try:
        while rclpy.ok():
            rclpy.spin_once(bridge_node, timeout_sec=0.01)
            tf_data = bridge_node.get_foxglove_transforms()
            foxglove.log("/tf", tf_data)
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        server.stop()
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
