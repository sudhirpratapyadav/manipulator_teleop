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

# Paths of MuJoCo model files
from ament_index_python.packages import get_package_share_directory
_HERE = Path(get_package_share_directory("manip_teleop")) / "models"
_ARM_XML = _HERE / "kinova_gen3" / "scene.xml"
_HAND_XML = _HERE / "robotiq_2f85" / "2f85.xml"

# Updated HOME_QPOS for Kinova Gen3 with Robotiq 2F85 gripper
# fmt: off
HOME_QPOS = [
    # Kinova Gen3 (joint_1 to joint_7)
    0, 0.26179939, 3.14159265, -2.26892803, 0, 0.95993109, 1.57079633,
    # Robotiq 2F85 gripper (all joints that get created after attachment)
    0, 0, 0, 0, 0, 0, 0, 0
]
# fmt: on


# IK parameters
SOLVER = "daqp"
POS_THRESHOLD = 1e-4
ORI_THRESHOLD = 1e-4
MAX_ITERS = 20
ACTION_SCALE = 0.01


def construct_model() -> mujoco.MjModel:
    """
    Construct a MuJoCo model with a Kinova Gen3 arm and Robotiq 2F85 gripper.
    """
    arm = mujoco.MjSpec.from_file(_ARM_XML.as_posix())
    hand = mujoco.MjSpec.from_file(_HAND_XML.as_posix())

    # Find the base mount of the Robotiq 2F85 gripper
    base_mount = hand.body("base_mount")
    base_mount.pos = (0, 0, 0.0)  # Adjust position as needed
    base_mount.quat = (0, 0, 0, 1)  # Adjust orientation as needed
    arm.attach(hand, prefix="robotiq_2f85/", site=arm.site("attachment_site"))

    # Remove any existing keyframes and add our own
    for key in arm.keys:
        if key.name in ["home", "retract"]:
            key.delete()
    arm.add_key(name="home", qpos=HOME_QPOS)

    # Add a mocap body for the target
    body = arm.worldbody.add_body(name="target", mocap=True)
    body.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=(0.05,) * 3,
        contype=0,
        conaffinity=0,
        rgba=(0.6, 0.3, 0.3, 0.2),
    )
    return arm.compile()

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


class KinovaSimWebXRNode(Node):
    def __init__(self):
        super().__init__('kinova_sim_webxr_node')
        # Construct model using the new function
        self.model = construct_model()
        self.data = mujoco.MjData(self.model)
        
        # Create a list of all arm joints (no base joints in this model)
        # Kinova Gen3 has 7 arm joints (joint1 to joint7)
        # fmt: off
        self.joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7",
        ]
        # fmt: on
        
        # Get the joint and actuator IDs
        self.dof_ids = np.array([self.model.joint(name).id for name in self.joint_names])
        # Actuator names should match joint names in this model
        self.actuator_ids = np.array([i for i in range(len(self.joint_names))])
        
        self.configuration = mink.Configuration(self.model)
        self.end_effector_task = mink.FrameTask(
            frame_name="robotiq_2f85/pinch",  # Updated to use the Robotiq gripper's pinch site
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        )
        
        # Simple posture task with small cost
        self.posture_task = mink.PostureTask(model=self.model, cost=1e-2)
        
        # Just use the end effector and posture tasks
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
        self.gripper_open = False  # Placeholder for future gripper implementation
        self.move_enabled = False
        self.prev_move_enabled = False  # To track changes in move_enabled state
        
        self.get_logger().info('KinovaSimWebXRNode started, waiting for /webxr/control messages.')

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
        self.gripper_open = msg.gripper_open  # Store for future gripper implementation
        self.move_enabled = msg.move_enabled
        
        self.get_logger().debug(
            f'Received WebXR control: pos({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) '
            f'quat({quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}) '
            f'gripper_open({msg.gripper_open}) move_enabled({msg.move_enabled})'
        )
        
    def control_gripper(self, data, position):
        """
        Control the Robotiq 2F85 gripper.
        
        Args:
            data: MjData object containing the simulation state
            position: Normalized position value [0, 1] where:
                     0 = fully open
                     1 = fully closed
        """
        # Clamp input to valid range [0, 1]
        position = max(0.0, min(1.0, position))
        
        # Map from [0, 1] to actuator control range [0, 255]
        # The gripper actuator is defined in the XML with ctrlrange="0 255"
        ctrl_value = position * 255.0
        
        # Find the gripper actuator index (assuming it's the last actuator)
        # For the Robotiq 2F85, the actuator name is "fingers_actuator"
        gripper_actuator_idx = data.ctrl.shape[0] - 1
        
        # Set the control value
        data.ctrl[gripper_actuator_idx] = ctrl_value
        
        return ctrl_value

    def sim_loop(self):
        with mujoco.viewer.launch_passive(
            model=self.model, data=self.data, show_left_ui=False, show_right_ui=False
        ) as viewer:
            mujoco.mjv_defaultFreeCamera(self.model, viewer.cam)
            mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
            self.configuration.update(self.data.qpos)
            self.posture_task.set_target_from_configuration(self.configuration)
            mujoco.mj_forward(self.model, self.data)
            
            # Initialize the mocap target at the end-effector site
            mink.move_mocap_to_frame(self.model, self.data, "target", "robotiq_2f85/pinch", "site")
            
            rate = RateLimiter(frequency=200.0, warn=False)
            while rclpy.ok() and viewer.is_running():
                rclpy.spin_once(self, timeout_sec=0)
                dt = rate.dt
                
                # viewer.opt.frame = mujoco.mjtFrame.mjFRAME_BODY
                
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
                
                # Implement gripper control based on WebXR gripper state
                if hasattr(self, 'gripper_open'):
                    # Map gripper_open (boolean) to gripper position (0-1)
                    # 0 = fully open, 1 = fully closed
                    gripper_position = 0.0 if self.gripper_open else 1.0
                    self.control_gripper(self.data, gripper_position)
                    self.get_logger().debug(f'Gripper state: {"open" if self.gripper_open else "closed"}')
                
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
                
                # Set control for arm joints
                self.data.ctrl[:len(self.joint_names)] = self.configuration.q[self.dof_ids]
                
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = KinovaSimWebXRNode()
    try:
        node.sim_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
