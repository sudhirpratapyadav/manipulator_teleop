#!/usr/bin/env python3

from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from manip_teleop.msg import WebXRControl
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import mink

from ament_index_python.packages import get_package_share_directory
_HERE = Path(get_package_share_directory("manip_teleop")) / "models"
_XML = _HERE / "omx" / "scene.xml"

# IK parameters
SOLVER = "daqp"  # Changed from "quadprog" to "daqp" which is available
POS_THRESHOLD = 1e-4
ORI_THRESHOLD = 1e-4
MAX_ITERS = 20
ACTION_SCALE = 0.01


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

def converge_ik(
    configuration, tasks, dt, solver, pos_threshold, ori_threshold, max_iters
):
    """
    Runs up to 'max_iters' of IK steps. Returns True if position and orientation
    are below thresholds, otherwise False.
    """
    for _ in range(max_iters):
        vel = mink.solve_ik(configuration, tasks, dt, solver, 1e-3)
        configuration.integrate_inplace(vel, dt)

        # Only checking the first FrameTask here (end_effector_task).
        # If you want to check multiple tasks, sum or combine their errors.
        err = tasks[0].compute_error(configuration)
        pos_achieved = np.linalg.norm(err[:3]) <= pos_threshold
        ori_achieved = np.linalg.norm(err[3:]) <= ori_threshold

        if pos_achieved and ori_achieved:
            return True
    return False


class OmxRealWebXRNode(Node):
    def __init__(self):
        super().__init__('omx_real_webxr_node')
        print(f"[DEBUG] Initializing OmxRealWebXRNode...")
        # Load model & data
        print(f"[DEBUG] Loading MuJoCo model from: {_XML.as_posix()}")
        self.model = mujoco.MjModel.from_xml_path(_XML.as_posix())
        self.data = mujoco.MjData(self.model)
        print(f"[DEBUG] MuJoCo model loaded successfully. Model has {self.model.nq} DOF")
        
        # Initialize mink configuration and tasks
        print(f"[DEBUG] Setting up mink IK configuration...")
        self.configuration = mink.Configuration(self.model)
        self.end_effector_task = mink.FrameTask(
            frame_name="pinch_site",
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        )
        self.posture_task = mink.PostureTask(model=self.model, cost=1e-2)
        self.tasks = [self.end_effector_task, self.posture_task]
        print(f"[DEBUG] Mink IK setup complete")
        
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
        self.prev_gripper_open = False
        
        # Action clients for controlling real robot
        print(f"[DEBUG] Setting up action clients for real robot...")
        self.arm_action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_action_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        
        # Target joint positions to send to real robot
        self.target_joint_positions = None
        
        # Single initialization flag - set when we receive joint angles (can be expanded later)
        self.initialization_complete = False
        
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
            
            # Initialize system with real robot position (only once)
            if not self.initialization_complete:
                self.initialize_mujoco_from_real_robot()
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

    def initialize_mujoco_from_real_robot(self):
        """Initialize MuJoCo simulation with current real robot joint positions"""
        if self.current_joint_positions is None:
            return
            
        print(f"[DEBUG] Initializing MuJoCo with real robot positions: {self.current_joint_positions}")
        
        # Set MuJoCo joint positions to match real robot
        # First 4 joints (arm)
        for i in range(4):
            if i < len(self.current_joint_positions):
                self.data.qpos[i] = self.current_joint_positions[i]
        
        # Gripper joint
        if len(self.current_joint_positions) > 4 and len(self.data.qpos) > 4:
            self.data.qpos[4] = self.current_joint_positions[4]
        
        # Update mink configuration with the new joint positions
        self.configuration.update(self.data.qpos)
        
        # Perform forward kinematics to update the model
        mujoco.mj_forward(self.model, self.data)
        
        # Update the posture task target to the current configuration
        self.posture_task.set_target_from_configuration(self.configuration)
        
        # Move the mocap target to the current end-effector position
        mink.move_mocap_to_frame(self.model, self.data, "target", "pinch_site", "site")
        
        print(f"[DEBUG] MuJoCo initialized successfully with real robot position")
        print(f"[DEBUG] End-effector position: {self.data.mocap_pos[0]}")
        print(f"[DEBUG] End-effector orientation: {self.data.mocap_quat[0]}")
        
        # Mark initialization as complete
        self.initialization_complete = True
        print(f"[DEBUG] System initialization complete! Ready for WebXR control.")

    def send_joint_pose_goal(self, joint_pose_goal, action_name='move', time_step=2.0):
        """Send joint trajectory goal to real robot"""
        print(f"[DEBUG] Sending joint pose goal: {joint_pose_goal}")
        
        goal_msg = FollowJointTrajectory.Goal()
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names[:4]  # Only arm joints
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_pose_goal
        point.time_from_start.sec = int(time_step)
        point.time_from_start.nanosec = int((time_step - int(time_step)) * 1e9)
        
        trajectory.points = [point]
        goal_msg.trajectory = trajectory

        self.get_logger().debug(f'Exec: {action_name}')
        
        # Send goal asynchronously (non-blocking)
        send_goal_future = self.arm_action_client.send_goal_async(goal_msg)
        return send_goal_future

    def send_gripper_command(self, gripper_open, action_name='gripper', time_step=1.0):
        """Send gripper command to real robot"""
        goal_msg = GripperCommand.Goal()
        
        # Set gripper position based on open/close state
        goal_msg.command.position = -0.02 if gripper_open else -0.01
        goal_msg.command.max_effort = 10.0

        gripper_state = 'open' if gripper_open else 'closed'
        print(f"[DEBUG] Sending gripper command: {gripper_state} (pos: {goal_msg.command.position})")
        
        # Send goal asynchronously (non-blocking)
        send_goal_future = self.gripper_action_client.send_goal_async(goal_msg)
        return send_goal_future

    def visualization_loop(self):
        print(f"[DEBUG] Starting visualization loop...")
        with mujoco.viewer.launch_passive(
            model=self.model, data=self.data, show_left_ui=False, show_right_ui=False
        ) as viewer:
            print(f"[DEBUG] MuJoCo viewer launched successfully")
            mujoco.mjv_defaultFreeCamera(self.model, viewer.cam)
            print(f"[DEBUG] Setting up camera and waiting for real robot joint states...")
            # Don't reset to home keyframe - wait for real robot data instead
            print(f"[DEBUG] Waiting for real robot initialization...")
            
            rate = RateLimiter(frequency=200.0, warn=False)
            print(f"[DEBUG] Starting main visualization loop at 200Hz")
            loop_count = 0
            while rclpy.ok() and viewer.is_running():
                rclpy.spin_once(self, timeout_sec=0)
                dt = rate.dt
                loop_count += 1
                
                # Print debug info every 1000 loops (5 seconds at 200Hz)
                if loop_count % 1000 == 0:
                    print(f"[DEBUG] Loop {loop_count}: dt={dt:.4f}s, joint_pos available: {self.current_joint_positions is not None}, initialization_complete: {self.initialization_complete}")
                
                # Skip processing if initialization is not complete yet
                if not self.initialization_complete:
                    if loop_count % 1000 == 0:
                        print(f"[DEBUG] Waiting for system initialization to complete...")
                    viewer.sync()
                    rate.sleep()
                    continue
                
                # WebXR processing for inverse kinematics (similar to omx_sim.py)
                # Check if move_enabled was just activated (transition from False to True)
                if self.move_enabled and not self.prev_move_enabled and np.any(self.latest_webxr_pos) and np.any(self.latest_webxr_quat):
                    # Get current robot end-effector position and orientation from MuJoCo
                    current_robot_ee_pos = self.data.mocap_pos[0].copy()
                    current_robot_ee_quat = self.data.mocap_quat[0].copy()
                    
                    # Calibrate when move is first enabled
                    self.initial_webxr_pos = self.latest_webxr_pos.copy()
                    self.initial_webxr_quat = self.latest_webxr_quat.copy()
                    
                    # Calculate the quaternion difference
                    self.quat_diff = quaternion_difference(self.initial_webxr_quat, current_robot_ee_quat)
                    
                    # Calculate position offset
                    self.pos_offset = current_robot_ee_pos - self.initial_webxr_pos
                    
                    print(f"[DEBUG] WebXR-Robot calibration performed")
                    print(f"[DEBUG] Current robot position: {current_robot_ee_pos}")
                    print(f"[DEBUG] WebXR position: {self.initial_webxr_pos}")
                    print(f"[DEBUG] Position offset: {self.pos_offset}")
                
                # Check if we have calibrated (position offset exists)
                if self.pos_offset is not None:
                    # Calculate target position by applying the offset to the current WebXR position
                    target_pos = self.latest_webxr_pos + self.pos_offset
                    
                    # Calculate target quaternion
                    target_quat = quaternion_multiply(self.quat_diff, self.latest_webxr_quat)
                    
                    # Set mocap target only if move is enabled
                    if self.move_enabled:
                        self.data.mocap_pos[0] = target_pos
                        self.data.mocap_quat[0] = target_quat
                        if loop_count % 1000 == 0:
                            print(f"[DEBUG] Moving robot: move_enabled is True")
                    else:
                        if loop_count % 1000 == 0:
                            print(f"[DEBUG] Robot stationary: move_enabled is False")
                else:
                    # Not calibrated yet, robot will be stationary
                    if loop_count % 1000 == 0:
                        print(f"[DEBUG] Waiting for move_enabled to calibrate WebXR-Robot position")
                
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
                
                # Control the gripper based on the gripper_open state
                if hasattr(self, 'gripper_open'):
                    # Set gripper width: 0.04 for open, 0.0 for closed
                    gripper_width = 0.04 if self.gripper_open else 0.0
                    # The 5th element (index 4) of the control array controls the gripper
                    self.data.ctrl[4] = gripper_width
                    if loop_count % 1000 == 0:
                        print(f"[DEBUG] Gripper state: {'open' if self.gripper_open else 'closed'} (width: {gripper_width})")
                
                # Update the end effector task target from the mocap body and perform IK
                T_wt = mink.SE3.from_mocap_name(self.model, self.data, "target")
                self.end_effector_task.set_target(T_wt)
                converge_ik(
                    self.configuration,
                    self.tasks,
                    dt,
                    SOLVER,
                    POS_THRESHOLD,
                    ORI_THRESHOLD,
                    MAX_ITERS,
                )
                
                # Get target joint positions from IK solution
                target_joint_positions = self.configuration.q[:4].copy()  # Only first 4 joints (arm)
                
                # Send commands to real robot if we have valid targets and move is enabled
                if self.move_enabled and self.pos_offset is not None:
                    # Send arm joint positions to real robot (asynchronously)
                    if loop_count % 50 == 0:  # Send commands at 4Hz (200Hz/50)
                        self.send_joint_pose_goal(target_joint_positions.tolist(), "webxr_ik", 0.25)
                        if loop_count % 1000 == 0:
                            print(f"[DEBUG] Sent target joints to real robot: {target_joint_positions}")
                
                    # Send gripper command if gripper state changed
                    if hasattr(self, 'prev_gripper_open') and self.gripper_open != self.prev_gripper_open:
                        self.send_gripper_command(self.gripper_open, "webxr_gripper", 0.5)
                        print(f"[DEBUG] Gripper state changed to: {'open' if self.gripper_open else 'closed'}")
                    self.prev_gripper_open = self.gripper_open
                
                # Store target joint positions for debugging
                self.target_joint_positions = target_joint_positions
                
                # Update MuJoCo control with IK solution (don't use mj_step as requested)
                self.data.ctrl[:4] = target_joint_positions
                
                # Preserve the gripper control value
                if hasattr(self, 'gripper_open'):
                    self.data.ctrl[4] = 0.04 if self.gripper_open else 0.0
                
                # Forward kinematics to update the visualization (skip mj_step as requested)
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