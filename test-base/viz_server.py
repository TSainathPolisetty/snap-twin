import foxglove
import logging
import time
import os
import numpy as np
from yourdfpy import URDF
from scipy.spatial.transform import Rotation as R
from foxglove.schemas import FrameTransforms, FrameTransform, Vector3, Quaternion

# Absolute Paths
BASE_DIR = "/home/ubuntu/demo/snap-twin"
URDF_FILE = os.path.join(BASE_DIR, "test-base/final_scene.urdf")
TEMP_URDF = os.path.join(BASE_DIR, "test-base/final_scene_resolved.urdf")
PACKAGE_ROOT = os.path.join(BASE_DIR, "simulation/models/SO101")

def main():
    foxglove.set_log_level(logging.INFO)
    
    if not os.path.exists(URDF_FILE):
        print(f"Error: URDF not found at {URDF_FILE}")
        return

    # 1. Resolve Paths
    with open(URDF_FILE, 'r') as f:
        urdf_content = f.read()
    fixed_content = urdf_content.replace("package://SO101", PACKAGE_ROOT)
    with open(TEMP_URDF, 'w') as f:
        f.write(fixed_content)
    
    print(f"Loading Resolved URDF...")
    robot = URDF.load(TEMP_URDF)
    
    # 2. Start Foxglove Server
    server = foxglove.start_server(host="0.0.0.0", port=8765)

    movable_joints = [j.name for j in robot.robot.joints if j.type != "fixed"]
    joint_positions = {name: 0.0 for name in movable_joints}
    
    print(f"Server Live at ws://0.0.0.0:8765")
    print(f"Movable joints found: {movable_joints}")

    try:
        while True:
            elapsed = time.time()
            if movable_joints:
                # Sine wave for visual confirmation
                joint_positions['shoulder_pan'] = 0.5 * np.sin(elapsed * 2)
            
            robot.update_cfg(joint_positions)
            
            transforms = []
            for joint in robot.robot.joints:
                T_local = robot.get_transform(frame_to=joint.child, frame_from=joint.parent)
                trans = T_local[:3, 3]
                quat = R.from_matrix(T_local[:3, :3]).as_quat()
                
                # Removed the failing get_timestamp() call
                transforms.append(FrameTransform(
                    parent_frame_id=joint.parent,
                    child_frame_id=joint.child,
                    translation=Vector3(x=float(trans[0]), y=float(trans[1]), z=float(trans[2])),
                    rotation=Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
                ))
            
            # Log to /tf
            foxglove.log("/tf", FrameTransforms(transforms=transforms))
            time.sleep(0.05) 
            
    except KeyboardInterrupt:
        print("\nStopping...")
        server.stop()
        if os.path.exists(TEMP_URDF):
            os.remove(TEMP_URDF)

if __name__ == "__main__":
    main()
