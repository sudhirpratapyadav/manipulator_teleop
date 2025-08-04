#!/usr/bin/env python3

from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter
import rclpy
from rclpy.node import Node
from manip_teleop.msg import WebXRControl
from sensor_msgs.msg import JointState

from ament_index_python.packages import get_package_share_directory
_HERE = Path(get_package_share_directory("manip_teleop")) / "models"
_XML = _HERE / "omx" / "scene.xml"


def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions [x, y, z, w]
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
    return np.array([x, y, z, w])

def quaternion_inverse(q):
    """
    Compute the inverse of a quaternion [x, y, z, w]
    """
    x, y, z, w = q
    norm_squared = x*x + y*y + z*z + w*w
    return np.array([-x, -y, -z, w]) / norm_squared

def quaternion_difference(q1, q2):
    """
    Find the quaternion that rotates from q1 to q2
    q_diff = q2 * q1^(-1)
    """
    q1_inv = quaternion_inverse(q1)
    return quaternion_multiply(q2, q1_inv)


class OmxRealWebXRNode(Node):
    def __init__(self):
        super().__init__('omx_real_webxr_node')
        print(f"[DEBUG] Initializing OmxRealWebXRNode...")
        # Load model & data
        print(f"[DEBUG] Loading MuJoCo model from: {_XML.as_posix()}")
        self.model = mujoco.MjModel.from_xml_path(_XML.as_posix())
        self.data = mujoco.MjData(self.model)
        print(f"[DEBUG] MuJoCo model loaded successfully. Model has {self.model.nq} DOF")
        
        # Joint names from the real robot
        self.joint_names = [
            'joint1',
            'joint2', 
            'joint3',
            'joint4',
            'gripper_left_joint',  # Only using left gripper joint
        ]
        
        # Current joint positions from real robot
        self.current_joint_positions = None
        
        # Initialize position and quaternion variables for WebXR (future use)
        self.initial_webxr_pos = None        
        self.initial_webxr_quat = None       
        self.quat_diff = None                
        self.pos_offset = None               
        
        # Subscribe to real robot joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscribe to WebXR control messages (for future expansion)
        self.webxr_sub = self.create_subscription(
            WebXRControl,
            '/webxr/control',
            self.webxr_control_callback,
            10
        )
        
        # Store the latest pose information
        self.latest_webxr_pos = np.zeros(3)
        self.latest_webxr_quat = np.array([0, 0, 0, 1])  # [x, y, z, w]
        
        # Control states
        self.gripper_open = False
        self.move_enabled = False
        self.prev_move_enabled = False
        
        print(f"[DEBUG] Node initialization complete. Joint names: {self.joint_names}")
        self.get_logger().info('OmxRealWebXRNode started, reading from real robot and visualizing in MuJoCo.')

    def joint_state_callback(self, joint_state):
        """Handle incoming joint state data from real robot"""
        print(f"[DEBUG] Received joint state with {len(joint_state.name)} joints: {joint_state.name}")
        joint_pos_ordered = [None] * len(self.joint_names)
        
        # Order the joint positions according to our joint names
        for i, name in enumerate(joint_state.name):
            if name == self.joint_names[0]:  # joint1
                joint_pos_ordered[0] = joint_state.position[i]
            elif name == self.joint_names[1]:  # joint2
                joint_pos_ordered[1] = joint_state.position[i]
            elif name == self.joint_names[2]:  # joint3
                joint_pos_ordered[2] = joint_state.position[i]
            elif name == self.joint_names[3]:  # joint4
                joint_pos_ordered[3] = joint_state.position[i]
            elif name == self.joint_names[4]:  # gripper_left_joint
                joint_pos_ordered[4] = joint_state.position[i]
        
        # Store the current joint positions
        if all(pos is not None for pos in joint_pos_ordered):
            self.current_joint_positions = joint_pos_ordered
            print(f"[DEBUG] Updated joint positions: {[f'{pos:.3f}' for pos in joint_pos_ordered]}")
            self.get_logger().debug(f'Real robot joint positions: {[f"{pos:.3f}" for pos in joint_pos_ordered]}')
        else:
            print(f"[DEBUG] Incomplete joint data received. Missing: {[i for i, pos in enumerate(joint_pos_ordered) if pos is None]}")

    def webxr_control_callback(self, msg):
        """Handle incoming WebXR control data (for future expansion)"""
        print(f"[DEBUG] Received WebXR control message")
        # Extract position and orientation from the WebXRControl message
        pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        quat = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        
        # Update the latest WebXR pose
        self.latest_webxr_pos = pos
        self.latest_webxr_quat = quat
        
        # Store previous move_enabled state to detect changes
        self.prev_move_enabled = self.move_enabled
        
        # Update the control states
        self.gripper_open = msg.gripper_open
        self.move_enabled = msg.move_enabled
        
        self.get_logger().debug(
            f'Received WebXR control: pos({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) '
            f'quat({quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}) '
            f'gripper_open({msg.gripper_open}) move_enabled({msg.move_enabled})'
        )

    def visualization_loop(self):
        print(f"[DEBUG] Starting visualization loop...")
        with mujoco.viewer.launch_passive(
            model=self.model, data=self.data, show_left_ui=False, show_right_ui=False
        ) as viewer:
            print(f"[DEBUG] MuJoCo viewer launched successfully")
            mujoco.mjv_defaultFreeCamera(self.model, viewer.cam)
            print(f"[DEBUG] Setting up camera and resetting to home keyframe")
            mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
            mujoco.mj_forward(self.model, self.data)
            print(f"[DEBUG] Initial forward kinematics complete")
            
            rate = RateLimiter(frequency=200.0, warn=False)
            print(f"[DEBUG] Starting main visualization loop at 200Hz")
            loop_count = 0
            while rclpy.ok() and viewer.is_running():
                rclpy.spin_once(self, timeout_sec=0)
                dt = rate.dt
                loop_count += 1
                
                # Print debug info every 1000 loops (5 seconds at 200Hz)
                if loop_count % 1000 == 0:
                    print(f"[DEBUG] Loop {loop_count}: dt={dt:.4f}s, joint_pos available: {self.current_joint_positions is not None}")
                
                # Update MuJoCo model with real robot joint positions
                if self.current_joint_positions is not None:
                    if loop_count % 1000 == 0:  # Debug every 1000 loops
                        print(f"[DEBUG] Updating MuJoCo with joint positions: {[f'{pos:.3f}' for pos in self.current_joint_positions]}")
                    # Set the first 4 joint positions (arm joints) directly in MuJoCo
                    for i in range(4):
                        if i < len(self.current_joint_positions) and self.current_joint_positions[i] is not None:
                            old_pos = self.data.qpos[i]
                            self.data.qpos[i] = self.current_joint_positions[i]
                            if loop_count % 1000 == 0:  # Debug every 1000 loops
                                print(f"[DEBUG] Joint {i}: {old_pos:.3f} -> {self.current_joint_positions[i]:.3f}")
                    
                    # Set gripper position if available
                    if len(self.current_joint_positions) > 4 and self.current_joint_positions[4] is not None:
                        # The gripper joint in MuJoCo might be at index 4
                        if len(self.data.qpos) > 4:
                            old_gripper = self.data.qpos[4]
                            self.data.qpos[4] = self.current_joint_positions[4]
                            if loop_count % 1000 == 0:  # Debug every 1000 loops
                                print(f"[DEBUG] Gripper: {old_gripper:.3f} -> {self.current_joint_positions[4]:.3f}")
                        else:
                            if loop_count % 1000 == 0:
                                print(f"[DEBUG] Warning: MuJoCo model doesn't have gripper joint at index 4")
                else:
                    if loop_count % 1000 == 0:
                        print(f"[DEBUG] No joint positions available from real robot")
                
                # Forward kinematics to update the visualization
                mujoco.mj_forward(self.model, self.data)
                viewer.sync()
                rate.sleep()


def main(args=None):
    print(f"[DEBUG] Starting main function...")
    rclpy.init(args=args)
    print(f"[DEBUG] ROS2 initialized")
    node = OmxRealWebXRNode()
    try:
        node.visualization_loop()
    except KeyboardInterrupt:
        print(f"[DEBUG] Keyboard interrupt received, shutting down...")
        pass
    finally:
        print(f"[DEBUG] Cleaning up node and shutting down ROS2...")
        node.destroy_node()
        rclpy.shutdown()
        print(f"[DEBUG] Shutdown complete")

if __name__ == "__main__":
    main()