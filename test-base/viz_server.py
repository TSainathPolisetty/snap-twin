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
URDF_FILE = os.path.join(BASE_DIR, "test-base/final_browser.urdf")
TEMP_URDF = os.path.join(BASE_DIR, "test-base/final_scene_resolved.urdf")
PACKAGE_ROOT = os.path.join(BASE_DIR, "simulation/models/SO101")

def main():
    foxglove.set_log_level(logging.INFO)
    
    if not os.path.exists(URDF_FILE):
        print(f"Error: URDF not found at {URDF_FILE}")
        return

    # 1. Resolve Paths for the local URDF loader
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
            if 'shoulder_pan' in joint_positions:
                # Smooth sine wave movement
                joint_positions['shoulder_pan'] = 0.5 * np.sin(elapsed * 1.5)
            
            robot.update_cfg(joint_positions)
            
            transforms = []

            # --- MANUAL STATIC TRANSFORMS (The "World" Environment) ---
            # These are needed because fixed joints aren't always processed 
            # by the joint loop if they sit above the robot's base.
            
            # 1. World to Floor
            transforms.append(FrameTransform(
                parent_frame_id="world",
                child_frame_id="floor",
                translation=Vector3(x=0.0, y=0.0, z=0.0),
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ))

            # 2. World to Table (Table center is at half height: 0.75 / 2 = 0.375)
            transforms.append(FrameTransform(
                parent_frame_id="world",
                child_frame_id="table",
                translation=Vector3(x=0.0, y=0.0, z=0.375),
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ))

            # 3. Table to Base_Link (Robot sits on top of the table)
            transforms.append(FrameTransform(
                parent_frame_id="table",
                child_frame_id="base_link",
                translation=Vector3(x=0.0, y=0.0, z=0.375),
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ))

            # --- ROBOT JOINT TRANSFORMS ---
            for joint in robot.robot.joints:
                T_local = robot.get_transform(frame_to=joint.child, frame_from=joint.parent)
                trans = T_local[:3, 3]
                quat = R.from_matrix(T_local[:3, :3]).as_quat()
                
                transforms.append(FrameTransform(
                    parent_frame_id=joint.parent,
                    child_frame_id=joint.child,
                    translation=Vector3(x=float(trans[0]), y=float(trans[1]), z=float(trans[2])),
                    rotation=Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3]))
                ))
            
            # Broadcast all transforms to Foxglove
            foxglove.log("/tf", FrameTransforms(transforms=transforms))
            time.sleep(0.05) # 20Hz refresh rate
            
    except KeyboardInterrupt:
        print("\nStopping Server...")
        server.stop()
        if os.path.exists(TEMP_URDF):
            os.remove(TEMP_URDF)

if __name__ == "__main__":
    main()
