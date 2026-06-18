import os
import time
import sys

_MISSING = []
try:
    import rclpy
    import numpy as np
    import foxglove
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from yourdfpy import URDF
    from scipy.spatial.transform import Rotation as R
    from foxglove.schemas import FrameTransforms, FrameTransform, Vector3, Quaternion
except ImportError as _e:
    print(
        f"ERROR: Missing dependency — {_e}\n"
        "This script requires:\n"
        "  pip3 install foxglove-sdk yourdfpy scipy --break-system-packages\n"
        "It is a standby fallback for when foxglove_bridge cannot be run natively.\n"
        "Under normal operation start_gesture_demo.sh uses foxglove_bridge instead."
    )
    sys.exit(1)

try:
    from foxglove.messages import (
        SceneUpdate, SceneEntity, SceneEntityDeletion, SceneEntityDeletionType,
        SpherePrimitive, Color as FoxgloveColor, Pose as FoxglovePose,
        Vector3 as FoxgloveVector3,
    )
    from foxglove.schemas import Timestamp as FoxgloveTimestamp
    _HAS_SCENE = True
except ImportError:
    _HAS_SCENE = False

try:
    from visualization_msgs.msg import MarkerArray
    _HAS_MARKERS = True
except ImportError:
    _HAS_MARKERS = False

# Derive paths relative to this script's location
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
URDF_FILE = os.path.join(BASE_DIR, "final_twin.urdf")

class So101DigitalTwin(Node):
    def __init__(self, robot_model):
        super().__init__('so101_digital_twin')
        self.robot = robot_model
        
        # Identify joints that are actually movable (revolute)
        self.movable_joints = [j.name for j in self.robot.robot.joints if j.type == "revolute"]
        self.current_joint_positions = {name: 0.0 for name in self.movable_joints}
        
        self.gesture_active = False
        self.gesture_positions = None
        self._latest_markers = None  # cached /obstacle_markers
        self._collision_active = False  # cached /collision_warning state

        # Primary source of truth: follower's actual interpolated position
        self.create_subscription(
            JointState,
            '/follower/joint_states',
            self._follower_js_cb,
            10)
        # Fallback: leader position used only when follower hasn't published yet
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self._follower_received = False

        self.create_subscription(
            Bool,
            '/gesture_active',
            self._gesture_active_cb,
            10)
        self.create_subscription(
            JointState,
            '/gesture/joint_states',
            self._gesture_states_cb,
            10)
        self.create_subscription(
            Bool,
            '/collision_warning',
            self._collision_cb,
            10)

        # Subscribe to obstacle markers from overhead_vision_node
        if _HAS_MARKERS:
            self.create_subscription(
                MarkerArray,
                '/obstacle_markers',
                self._obstacle_markers_cb,
                10)
            self.get_logger().info("Subscribed to /obstacle_markers for Foxglove scene population")
        else:
            self.get_logger().warn("visualization_msgs not available - /obstacle_markers not forwarded to Foxglove")

        self.get_logger().info(f"Digital Twin Bridge active. Monitoring moving joints: {self.movable_joints}")

    def _follower_js_cb(self, msg):
        """Follower's actual interpolated joint positions — primary sim source of truth."""
        self._follower_received = True
        for name, position in zip(msg.name, msg.position):
            if name in self.current_joint_positions:
                self.current_joint_positions[name] = position

    def joint_state_callback(self, msg):
        # Only used before the follower has started publishing its own states
        if self._follower_received:
            return
        for name, position in zip(msg.name, msg.position):
            if name in self.current_joint_positions:
                self.current_joint_positions[name] = position

    def _gesture_active_cb(self, msg):
        self.gesture_active = msg.data

    def _gesture_states_cb(self, msg):
        self.gesture_positions = msg

    def _collision_cb(self, msg):
        self._collision_active = msg.data

    def _obstacle_markers_cb(self, msg: 'MarkerArray'):
        self._latest_markers = msg

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

    def get_foxglove_scene(self):
        """Build Foxglove SceneUpdate: obstacle markers from overhead + collision sphere at gripper FK."""
        if not _HAS_SCENE:
            return None

        entities = []
        deletions = []

        # --- Overhead prop markers (from overhead_vision_node) ---
        markers = self._latest_markers
        if markers is not None:
            for m in markers.markers:
                from visualization_msgs.msg import Marker
                if m.action == Marker.DELETE or m.action == Marker.DELETEALL:
                    deletions.append(
                        SceneEntityDeletion(
                            type=SceneEntityDeletionType.MatchingId,
                            id=f"{m.ns}_{m.id}",
                        )
                    )
                elif m.action in (Marker.ADD, Marker.MODIFY):
                    if m.type == Marker.SPHERE:
                        try:
                            sphere = SpherePrimitive(
                                pose=FoxglovePose(
                                    position=FoxgloveVector3(
                                        x=float(m.pose.position.x),
                                        y=float(m.pose.position.y),
                                        z=float(m.pose.position.z),
                                    ),
                                ),
                                size=FoxgloveVector3(
                                    x=float(m.scale.x),
                                    y=float(m.scale.y),
                                    z=float(m.scale.z),
                                ),
                                color=FoxgloveColor(
                                    r=float(m.color.r),
                                    g=float(m.color.g),
                                    b=float(m.color.b),
                                    a=float(m.color.a),
                                ),
                            )
                            entities.append(
                                SceneEntity(
                                    frame_id=m.header.frame_id or 'base_link',
                                    id=f"{m.ns}_{m.id}",
                                    spheres=[sphere],
                                )
                            )
                        except Exception:
                            pass

        # --- Collision sphere: placed at gripper FK position when collision is active ---
        if self._collision_active:
            try:
                # FK is already updated in get_foxglove_transforms() which runs before this
                T_gripper = self.robot.get_transform(
                    frame_to='gripper_link', frame_from='base_link'
                )
                gx, gy, gz = float(T_gripper[0, 3]), float(T_gripper[1, 3]), float(T_gripper[2, 3])
                entities.append(
                    SceneEntity(
                        frame_id='base_link',
                        id='collision_obstacle',
                        spheres=[SpherePrimitive(
                            pose=FoxglovePose(
                                position=FoxgloveVector3(x=gx, y=gy, z=gz),
                            ),
                            size=FoxgloveVector3(x=0.08, y=0.08, z=0.08),
                            color=FoxgloveColor(r=1.0, g=0.2, b=0.0, a=0.85),
                        )],
                    )
                )
            except Exception:
                pass
        else:
            # Remove the collision sphere when clear
            deletions.append(
                SceneEntityDeletion(
                    type=SceneEntityDeletionType.MatchingId,
                    id='collision_obstacle',
                )
            )

        if not entities and not deletions:
            return None
        return SceneUpdate(entities=entities, deletions=deletions)


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
            # Spin ROS to process incoming /joint_states and /obstacle_markers
            rclpy.spin_once(bridge_node, timeout_sec=0.01)
            
            # Broadcast live FK transforms to Foxglove
            tf_data = bridge_node.get_foxglove_transforms()
            foxglove.log("/tf", tf_data)

            # Broadcast obstacle markers as 3D scene entities
            scene = bridge_node.get_foxglove_scene()
            if scene is not None:
                foxglove.log("/scene", scene)
            
            # ~50Hz refresh rate
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        server.stop()
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
