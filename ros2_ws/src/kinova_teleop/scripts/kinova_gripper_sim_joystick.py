#!/usr/bin/env python3

from pathlib import Path
import time
import os

import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter
import rclpy
from rclpy.node import Node
from kinova_teleop.msg import WebXRControl
import pygame

import mink

# For joystick handling
os.environ["SDL_VIDEODRIVER"] = "dummy"  # Headless mode for pygame
pygame.init()
pygame.joystick.init()

# Xbox 360 controller constants
XBOX_A_BUTTON = 0
XBOX_B_BUTTON = 1
XBOX_X_BUTTON = 2
XBOX_Y_BUTTON = 3
XBOX_LB_BUTTON = 4
XBOX_RB_BUTTON = 5
XBOX_BACK_BUTTON = 6
XBOX_START_BUTTON = 7
XBOX_LEFT_STICK_BUTTON = 8
XBOX_RIGHT_STICK_BUTTON = 9

# Xbox 360 axes
XBOX_LEFT_STICK_X = 0   # Left = -1, Right = 1
XBOX_LEFT_STICK_Y = 1   # Up = -1, Down = 1
XBOX_LT_AXIS = 2        # Rest = -1, Pressed = 1
XBOX_RIGHT_STICK_X = 3  # Left = -1, Right = 1
XBOX_RIGHT_STICK_Y = 4  # Up = -1, Down = 1
XBOX_RT_AXIS = 5        # Rest = -1, Pressed = 1
XBOX_DPAD_X = 6         # Left = -1, Right = 1
XBOX_DPAD_Y = 7         # Up = -1, Down = 1

# Joint control speed constants
EE_TRANSLATION_SPEED = 0.005  # Rate of end-effector position change (meters)
EE_ROTATION_SPEED = 0.01      # Rate of end-effector orientation change (radians)
DEADZONE = 0.1                # Ignore small joystick movements
ACTIVATION_THRESHOLD = 0.8    # Only trigger movement when joystick is at 80% or more

# Update paths to use MuJoCo Menagerie models
_ARM_XML = Path("/home/robot/yash/ros2_ws/src/mujoco_menagerie/kinova_gen3/scene.xml")
_HAND_XML = Path("/home/robot/yash/ros2_ws/src/mujoco_menagerie/robotiq_2f85/2f85.xml")

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
SOLVER = "daqp"  # 'osqp' or 'daqp' solvers can be tried
POS_THRESHOLD = 1e-3  # Even more relaxed threshold
ORI_THRESHOLD = 1e-3  # Even more relaxed threshold
MAX_ITERS = 100       # More iterations
ACTION_SCALE = 0.01
IK_DAMPING = 0.1      # Higher damping for more stability


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
    site = arm.site("pinch_site")
    arm.attach(hand, prefix="robotiq_2f85/", site=site)

    # Remove any existing keyframes and add our own
    for key in arm.keys:
        if key.name in ["home", "retract"]:
            key.delete()
    arm.add_key(name="home", qpos=HOME_QPOS)

    # Add a mocap body for the target
    body = arm.worldbody.add_body(name="target", mocap=True)
    body.add_geom(
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=(0.02,) * 3,
        contype=0,
        conaffinity=0,
        rgba=(0.6, 0.3, 0.3, 0.5),
    )
    
    # Add reference to the gripper's pinch site
    arm.worldbody.add_site(
        name="gripper_pinch_site", 
        pos=(0, 0, 0), 
        quat=(1, 0, 0, 0), 
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=(0.01, 0.01, 0.01),
        rgba=(0, 1, 0, 0.5)
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
    
    # Prevent division by zero
    if norm_squared < 1e-10:
        print(f"Warning: Quaternion {q} has near-zero norm, returning identity quaternion")
        return np.array([0, 0, 0, 1])
        
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
    last_error = float('inf')
    
    for iter_num in range(max_iters):
        # Solve IK with appropriate damping for stability
        vel = mink.solve_ik(configuration, tasks, dt, solver, IK_DAMPING)
        configuration.integrate_inplace(vel, dt)

        # Check the first FrameTask error (end_effector_task)
        err = tasks[0].compute_error(configuration)
        
        # Check error magnitudes
        pos_err = np.linalg.norm(err[:3])
        ori_err = np.linalg.norm(err[3:])
        
        pos_achieved = pos_err <= pos_threshold
        ori_achieved = ori_err <= ori_threshold
        
        # Check if error is decreasing
        current_error = pos_err + ori_err
        progress = last_error - current_error
        last_error = current_error
        
        # Exit early if we've achieved the goal
        if pos_achieved and ori_achieved:
            return True
            
        # Exit if we're making no progress (prevents getting stuck)
        if iter_num > 10 and abs(progress) < 1e-6:
            return False
            
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
        
        # Initialize joystick
        self.init_joystick()
        
        # Current joint angles and joystick state
        self.joint_angles = np.zeros(len(self.joint_names))
        # Start with joystick control active by default
        self.joystick_control_active = True
        self.webxr_control_active = False
        self.last_button_press_time = time.time()
        self.button_cooldown = 0.25  # Seconds between button presses
        
        # Target end-effector position and orientation
        self.target_ee_pos = np.zeros(3)
        self.target_ee_quat = np.array([0, 0, 0, 1])  # [x, y, z, w]
        
        # Store the latest pose information
        self.latest_webxr_pos = np.zeros(3)
        self.latest_webxr_quat = np.array([0, 0, 0, 1])  # [x, y, z, w]
        
        # Control states
        self.gripper_open = False  # Placeholder for future gripper implementation
        self.move_enabled = False
        self.prev_move_enabled = False  # To track changes in move_enabled state
        
        self.get_logger().info('KinovaSimWebXRNode started, waiting for /webxr/control messages.')
        self.get_logger().info(f'Joystick control active: {self.joystick_control_active}')
        self.get_logger().info(f'Initial joint angles: {self.joint_angles}')

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
            
            # Initialize joint angles from home position
            for i, joint_id in enumerate(self.dof_ids):
                self.joint_angles[i] = self.data.qpos[joint_id]
            
            # Use a more realistic rate that the system can handle
            rate = RateLimiter(frequency=100.0, warn=True)
            while rclpy.ok() and viewer.is_running():
                # Process ROS messages
                rclpy.spin_once(self, timeout_sec=0)
                dt = rate.dt
                
                # Process joystick input first - moved to beginning of loop
                self.process_joystick_input()
                
                # For joystick end-effector control, we don't need to calculate FK anymore
                # as we directly set the mocap target in process_joystick_input
                
                # Handle WebXR control
                if self.webxr_control_active:
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
                            self.get_logger().debug('Moving robot with WebXR: move_enabled is True')
                        else:
                            self.get_logger().debug('Robot stationary: move_enabled is False')
                    else:
                        # Not calibrated yet, robot will be stationary
                        self.get_logger().debug('Waiting for move_enabled to calibrate WebXR-Robot position')
                
                # Implement gripper control based on current state
                if hasattr(self, 'gripper_open'):
                    # Map gripper_open (boolean) to gripper position (0-1)
                    # 0 = fully open, 1 = fully closed
                    gripper_position = 0.0 if self.gripper_open else 1.0
                    self.control_gripper(self.data, gripper_position)
                    self.get_logger().debug(f'Gripper state: {"open" if self.gripper_open else "closed"}')
                
                # Update the end effector task target from the mocap body
                mocap_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "target")
                # Make sure the mocap_id is valid
                if mocap_id >= 0:
                    # Create the SE3 manually using the mocap position and quaternion
                    mocap_pos = self.data.mocap_pos[0].copy()  # Using index 0 since there's only one mocap body
                    mocap_quat = self.data.mocap_quat[0].copy()
                    # Create the SE3 transform - format is [qw, qx, qy, qz, x, y, z]
                    wxyz_xyz = np.concatenate([mocap_quat, mocap_pos])
                    T_wt = mink.SE3(wxyz_xyz)
                    self.end_effector_task.set_target(T_wt)
                else:
                    self.get_logger().error(f"Mocap body 'target' not found in model")
                
                # Run IK to solve for joint angles that achieve the target end-effector pose
                ik_converged = converge_ik(
                    self.configuration,
                    self.tasks,
                    dt,
                    SOLVER,
                    POS_THRESHOLD,
                    ORI_THRESHOLD,
                    MAX_ITERS,
                )
                
                if not ik_converged:
                    self.get_logger().warning('IK did not converge within iteration limit')
                
                # Update the joint angles from the IK solution if using joystick control
                if self.joystick_control_active:
                    for i, joint_id in enumerate(self.dof_ids):
                        self.joint_angles[i] = self.configuration.q[joint_id]
                    self.get_logger().debug(f'Updated joint angles from IK solution: {self.joint_angles}')
                
                # Set control for arm joints - apply the updated configuration to the simulation
                try:
                    self.data.ctrl[:len(self.joint_names)] = self.configuration.q[self.dof_ids]
                    self.get_logger().debug(f'Applied control values: {self.configuration.q[self.dof_ids]}')
                except Exception as e:
                    self.get_logger().error(f'Error setting control values: {str(e)}')
                
                # Step the simulation forward
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                rate.sleep()

                # Process joystick input for manual control
                self.process_joystick_input()

    def init_joystick(self):
        """Initialize joystick if available"""
        self.joystick = None
        self.joystick_initialized = False
        
        try:
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.joystick_initialized = True
                joystick_name = self.joystick.get_name()
                self.get_logger().info(f'Joystick initialized: {joystick_name}')
                
                # Make sure joystick control is active when a joystick is detected
                self.joystick_control_active = True
                self.webxr_control_active = False
                self.get_logger().info('Joystick control activated automatically')
                
                # Log the number of axes and buttons
                num_axes = self.joystick.get_numaxes()
                num_buttons = self.joystick.get_numbuttons()
                self.get_logger().info(f'Joystick has {num_axes} axes and {num_buttons} buttons')
                
                import atexit
                atexit.register(self.quit_pygame)
            else:
                self.get_logger().warning('No joystick detected')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize joystick: {str(e)}')
    
    def quit_pygame(self):
        """Clean up pygame resources"""
        if pygame.joystick.get_init():
            pygame.joystick.quit()
        if pygame.get_init():
            pygame.quit()
    
    def process_joystick_input(self):
        """Process joystick inputs to control the robot end-effector"""
        if not self.joystick_initialized:
            self.get_logger().debug('Joystick not initialized, skipping input processing')
            return
        
        # Process events to get fresh joystick data
        pygame.event.pump()
        
        current_time = time.time()
        
        # Toggle control mode (joystick/WebXR) with Start button
        if self.joystick.get_button(XBOX_START_BUTTON) and current_time - self.last_button_press_time > self.button_cooldown:
            self.joystick_control_active = not self.joystick_control_active
            self.webxr_control_active = not self.joystick_control_active
            self.last_button_press_time = current_time
            self.get_logger().info(f'Control mode: {"Joystick" if self.joystick_control_active else "WebXR"}')
        
        # Toggle gripper with A button
        if self.joystick.get_button(XBOX_A_BUTTON) and current_time - self.last_button_press_time > self.button_cooldown:
            self.gripper_open = not self.gripper_open
            self.last_button_press_time = current_time
            self.get_logger().info(f'Gripper: {"Open" if self.gripper_open else "Closed"}')
        
        # Only process end-effector control if joystick mode is active
        if not self.joystick_control_active:
            self.get_logger().debug('Joystick control not active, skipping end-effector control')
            return
            
        # For end-effector control, we'll use the current mocap position and orientation as a starting point
        # This ensures continuity of movement
        current_ee_pos = self.data.mocap_pos[0].copy()
        current_ee_quat = self.data.mocap_quat[0].copy()
        
        # Map joystick inputs to Cartesian movements
        # Left stick: X/Y translation in horizontal plane
        x_left = self.joystick.get_axis(XBOX_LEFT_STICK_X)
        y_left = self.joystick.get_axis(XBOX_LEFT_STICK_Y)
        
        # Right stick: Y for Z translation (up/down), X for rotation around Z
        x_right = self.joystick.get_axis(XBOX_RIGHT_STICK_X)
        y_right = self.joystick.get_axis(XBOX_RIGHT_STICK_Y)
        
        # LT/RT for roll control
        lt = (self.joystick.get_axis(XBOX_LT_AXIS) + 1) / 2  # Convert from [-1,1] to [0,1]
        rt = (self.joystick.get_axis(XBOX_RT_AXIS) + 1) / 2  # Convert from [-1,1] to [0,1]
        
        # D-pad for pitch control
        dpad_x = self.joystick.get_axis(XBOX_DPAD_X) if hasattr(self.joystick, 'get_axis') and self.joystick.get_numaxes() > XBOX_DPAD_X else 0
        
        # Log joystick values for debugging
        self.get_logger().debug(f'Joystick values - Left: ({x_left:.2f}, {y_left:.2f}), Right: ({x_right:.2f}, {y_right:.2f}), LT: {lt:.2f}, RT: {rt:.2f}, D-pad X: {dpad_x:.2f}')
        
        # Apply deadzone
        x_left = 0 if abs(x_left) < DEADZONE else x_left
        y_left = 0 if abs(y_left) < DEADZONE else y_left
        x_right = 0 if abs(x_right) < DEADZONE else x_right
        y_right = 0 if abs(y_right) < DEADZONE else y_right
        dpad_x = 0 if abs(dpad_x) < DEADZONE else dpad_x
        
        # Calculate position changes based on joystick input
        pos_change = np.zeros(3)
        
        # Only apply changes when joystick is beyond activation threshold
        if abs(x_left) > ACTIVATION_THRESHOLD:
            pos_change[0] += x_left * EE_TRANSLATION_SPEED  # X-axis translation
            self.get_logger().debug(f"End-effector X translation: {x_left:.2f}")
            
        if abs(y_left) > ACTIVATION_THRESHOLD:
            pos_change[1] += -y_left * EE_TRANSLATION_SPEED  # Y-axis translation (negative because Y-axis is inverted)
            self.get_logger().debug(f"End-effector Y translation: {-y_left:.2f}")
            
        if abs(y_right) > ACTIVATION_THRESHOLD:
            pos_change[2] += -y_right * EE_TRANSLATION_SPEED  # Z-axis translation (up/down)
            self.get_logger().debug(f"End-effector Z translation: {-y_right:.2f}")
        
        # Apply position changes to current position
        new_ee_pos = current_ee_pos + pos_change
        
        # Calculate orientation changes using quaternions
        # Create rotation quaternions for each axis
        if abs(x_right) > ACTIVATION_THRESHOLD:
            # Rotation around Z axis (yaw)
            angle_z = x_right * EE_ROTATION_SPEED
            quat_z = np.array([np.cos(angle_z/2), 0, 0, np.sin(angle_z/2)])  # [w, x, y, z]
            self.get_logger().debug(f"End-effector yaw rotation: {angle_z:.2f}")
        else:
            quat_z = np.array([1, 0, 0, 0])  # Identity quaternion
            
        # LB/RB buttons for pitch control (rotation around Y)
        angle_y = 0
        if self.joystick.get_button(XBOX_LB_BUTTON):
            angle_y = -EE_ROTATION_SPEED
            self.get_logger().debug("End-effector pitch down")
        elif self.joystick.get_button(XBOX_RB_BUTTON):
            angle_y = EE_ROTATION_SPEED
            self.get_logger().debug("End-effector pitch up")
            
        quat_y = np.array([np.cos(angle_y/2), 0, np.sin(angle_y/2), 0])  # [w, x, y, z]
        
        # Roll control with triggers (rotation around X)
        roll_input = 0
        if lt > ACTIVATION_THRESHOLD:
            roll_input = -lt * EE_ROTATION_SPEED
            self.get_logger().debug(f"End-effector roll left: {-lt:.2f}")
        elif rt > ACTIVATION_THRESHOLD:
            roll_input = rt * EE_ROTATION_SPEED
            self.get_logger().debug(f"End-effector roll right: {rt:.2f}")
            
        quat_x = np.array([np.cos(roll_input/2), np.sin(roll_input/2), 0, 0])  # [w, x, y, z]
            
        # Combine quaternion rotations (x * y * z)
        combined_quat = quaternion_multiply(quat_x, quaternion_multiply(quat_y, quat_z))
        
        # Apply rotation to current orientation
        new_ee_quat = quaternion_multiply(combined_quat, current_ee_quat)
        
        # Normalize the quaternion
        quat_norm = np.linalg.norm(new_ee_quat)
        if quat_norm > 1e-10:  # Avoid division by near-zero
            new_ee_quat = new_ee_quat / quat_norm
            
        # Update the mocap target directly with our computed end-effector pose
        if np.any(pos_change) or np.any(combined_quat != np.array([1, 0, 0, 0])):
            self.data.mocap_pos[0] = new_ee_pos
            self.data.mocap_quat[0] = new_ee_quat
            self.get_logger().debug(f"Updated end-effector target: pos={new_ee_pos}, quat={new_ee_quat}")
    
    def calculate_forward_kinematics(self):
        """Calculate forward kinematics using mink to get end-effector pose"""
        if not self.joystick_control_active:
            self.get_logger().debug('Not calculating FK because joystick control is not active')
            return
            
        # Create a temporary configuration with our joint angles
        temp_config = mink.Configuration(self.model)
        
        # Set the joint positions in the configuration
        q = self.model.keyframe("home").qpos.copy()
        for i, joint_id in enumerate(self.dof_ids):
            q[joint_id] = self.joint_angles[i]
        
        # Update the configuration with our joint angles
        temp_config.update(q)
        
        # Create temporary MjData to compute forward kinematics
        try:
            temp_data = mujoco.MjData(self.model)
            # Copy the joint positions to the data
            temp_data.qpos[:] = q
            # Compute forward kinematics
            mujoco.mj_forward(self.model, temp_data)
            
            # Get the site position and orientation
            site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "robotiq_2f85/pinch")
            if site_id == -1:
                raise ValueError(f"Site 'robotiq_2f85/pinch' not found in model")
                
            # Get position and orientation of the site
            self.get_logger().debug(f"Getting position and orientation for site ID: {site_id}")
            pos = temp_data.site_xpos[site_id].copy()
            # Extract rotation matrix and convert to quaternion
            rot_mat = temp_data.site_xmat[site_id].reshape(3, 3).copy()
            quat = np.zeros(4, dtype=np.float64)
            mujoco.mju_mat2Quat(quat, rot_mat.flatten())
            
            # Create an SE3 object from the position and orientation
            ee_frame_params = np.concatenate([quat, pos])
            ee_frame = mink.SE3(ee_frame_params)
            
            # Extract position and orientation
            self.target_ee_pos = ee_frame.translation()
            self.target_ee_quat = ee_frame.parameters()[:4]  # Extract quaternion (wxyz)
            
            self.get_logger().debug(f'FK calculated - Target EE pos: {self.target_ee_pos}, quat: {self.target_ee_quat}')
        except Exception as e:
            self.get_logger().error(f'Error calculating forward kinematics: {str(e)}')

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
