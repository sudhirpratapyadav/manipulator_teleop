#!/usr/bin/env python3

from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter
import rclpy
from rclpy.node import Node
from manip_teleop.msg import WebXRControl

import mink

from ament_index_python.packages import get_package_share_directory
_HERE = Path(get_package_share_directory("manip_teleop")) / "models"
_XML = _HERE / "franka_emika_panda" / "mjx_scene.xml"

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


class FrankaSimWebXRNode(Node):
    def __init__(self):
        super().__init__('franka_sim_webxr_node')
        # Load model & data
        self.model = mujoco.MjModel.from_xml_path(_XML.as_posix())
        self.data = mujoco.MjData(self.model)
        self.configuration = mink.Configuration(self.model)
        self.end_effector_task = mink.FrameTask(
            frame_name="attachment_site",
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        )
        self.posture_task = mink.PostureTask(model=self.model, cost=1e-2)
        self.tasks = [self.end_effector_task, self.posture_task]
        
        # Initialize position and quaternion variables
        self.initial_webxr_pos = None        # WebXR position when move is enabled
        self.initial_webxr_quat = None       # WebXR quaternion when move is enabled
        self.quat_diff = None                # Difference quaternion
        self.pos_offset = None               # Position offset
        
        # Subscribe to WebXR control messages
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
        self.prev_move_enabled = False  # To track changes in move_enabled state
        
        self.get_logger().info('FrankaSimWebXRNode started, waiting for /webxr/control messages.')

    def webxr_control_callback(self, msg):
        """Handle incoming WebXR control data"""
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

    def sim_loop(self):
        with mujoco.viewer.launch_passive(
            model=self.model, data=self.data, show_left_ui=False, show_right_ui=False
        ) as viewer:
            mujoco.mjv_defaultFreeCamera(self.model, viewer.cam)
            mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
            self.configuration.update(self.data.qpos)
            self.posture_task.set_target_from_configuration(self.configuration)
            mujoco.mj_forward(self.model, self.data)
            mink.move_mocap_to_frame(self.model, self.data, "target", "attachment_site", "site")
            
            rate = RateLimiter(frequency=200.0, warn=False)
            while rclpy.ok() and viewer.is_running():
                rclpy.spin_once(self, timeout_sec=0)
                dt = rate.dt
                
                # Check if move_enabled was just activated (transition from False to True)
                if self.move_enabled and not self.prev_move_enabled and np.any(self.latest_webxr_pos) and np.any(self.latest_webxr_quat):
                    # Get current robot end-effector position and orientation
                    current_robot_ee_pos = self.data.mocap_pos[0].copy()
                    current_robot_ee_quat = self.data.mocap_quat[0].copy()
                    
                    # Calibrate when move is first enabled
                    self.initial_webxr_pos = self.latest_webxr_pos.copy()
                    self.initial_webxr_quat = self.latest_webxr_quat.copy()
                    
                    # Calculate the quaternion difference
                    self.quat_diff = quaternion_difference(self.initial_webxr_quat, current_robot_ee_quat)
                    
                    # Calculate position offset
                    self.pos_offset = current_robot_ee_pos - self.initial_webxr_pos
                    
                    self.get_logger().info('WebXR-Robot calibration performed')
                    self.get_logger().info(f'Current robot position: {current_robot_ee_pos}')
                    self.get_logger().info(f'WebXR position: {self.initial_webxr_pos}')
                    self.get_logger().info(f'Position offset: {self.pos_offset}')
                
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
                        self.get_logger().debug('Moving robot: move_enabled is True')
                    else:
                        self.get_logger().debug('Robot stationary: move_enabled is False')
                else:
                    # Not calibrated yet, robot will be stationary
                    self.get_logger().debug('Waiting for move_enabled to calibrate WebXR-Robot position')
                
                # Control the gripper based on the gripper_open state
                if hasattr(self, 'gripper_open'):
                    # Set gripper width: 0.04 for open, 0.0 for closed
                    gripper_width = 0.04 if self.gripper_open else 0.0
                    # The 8th element (index 7) of the control array controls the gripper
                    self.data.ctrl[7] = gripper_width
                    self.get_logger().debug(f'Gripper state: {"open" if self.gripper_open else "closed"} (width: {gripper_width})')
                
                # Update the end effector task target from the mocap body
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
                self.data.ctrl = self.configuration.q[:8]
                
                # Preserve the gripper control value
                if hasattr(self, 'gripper_open'):
                    # Set the 8th element (index 7) to maintain the gripper width
                    self.data.ctrl[7] = 0.04 if self.gripper_open else 0.0
                
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = FrankaSimWebXRNode()
    try:
        node.sim_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
