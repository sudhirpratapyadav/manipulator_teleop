#!/usr/bin/env python3
"""
OmxRealWebXRNode V2 - Redesigned Architecture

This version fixes critical concurrency issues in the original implementation:
- Eliminates deadlocks from blocking calls in ROS callbacks
- Single-threaded main loop for thread safety
- Event-driven state machine architecture
- Async action management for robot control
- Robust error handling and recovery
"""

from pathlib import Path
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Dict, Tuple, Any
import time
import math
import logging

import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from manip_teleop.msg import WebXRControl
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import mink

from ament_index_python.packages import get_package_share_directory

# Configuration constants
_HERE = Path(get_package_share_directory("manip_teleop")) / "models"
_XML = _HERE / "omx" / "scene.xml"

# Utility functions
def deg2rad(joint_pos_deg):
    """Convert degrees to radians"""
    if isinstance(joint_pos_deg, list):
        return [angle * (math.pi / 180) for angle in joint_pos_deg]
    return joint_pos_deg * (math.pi / 180)

def quaternion_multiply(q1, q2):
    """Multiply two quaternions [x, y, z, w]"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
    return np.array([x, y, z, w])

def quaternion_inverse(q):
    """Compute the inverse of a quaternion [x, y, z, w]"""
    x, y, z, w = q
    norm_squared = x*x + y*y + z*z + w*w
    return np.array([-x, -y, -z, w]) / norm_squared

def quaternion_difference(q1, q2):
    """Find the quaternion that rotates from q1 to q2"""
    q1_inv = quaternion_inverse(q1)
    return quaternion_multiply(q2, q1_inv)

class SystemState(Enum):
    """System state machine states"""
    STARTING = "starting"
    WAITING_FOR_JOINTS = "waiting_for_joints"
    SYNCING_MUJOCO = "syncing_mujoco"
    GOING_TO_HOME = "going_to_home"
    TESTING_GRIPPER_OPEN = "testing_gripper_open"
    TESTING_GRIPPER_CLOSE = "testing_gripper_close"
    READY_FOR_WEBXR = "ready_for_webxr"
    WEBXR_ACTIVE = "webxr_active"
    ERROR_RECOVERY = "error_recovery"
    FAILED = "failed"

@dataclass
class RobotData:
    """Thread-safe data container updated only by ROS callbacks"""
    current_joints: Optional[List[float]] = None
    webxr_pos: Optional[np.ndarray] = None
    webxr_quat: Optional[np.ndarray] = None
    gripper_open: bool = False
    move_enabled: bool = False
    prev_move_enabled: bool = False
    prev_gripper_open: bool = False
    last_joint_update: float = 0.0
    last_webxr_update: float = 0.0
    
    def update_joints(self, joints: List[float]):
        """Called from joint_state_callback"""
        self.current_joints = joints
        self.last_joint_update = time.time()
        
    def update_webxr(self, pos: np.ndarray, quat: np.ndarray, 
                    gripper_open: bool, move_enabled: bool):
        """Called from webxr_callback"""
        self.prev_move_enabled = self.move_enabled
        self.prev_gripper_open = self.gripper_open
        
        self.webxr_pos = pos.copy()
        self.webxr_quat = quat.copy()
        self.gripper_open = gripper_open
        self.move_enabled = move_enabled
        self.last_webxr_update = time.time()

@dataclass
class SystemConfig:
    """System configuration"""
    loop_frequency: float = 200.0
    action_timeout: float = 10.0
    ik_solver: str = "daqp"
    pos_threshold: float = 1e-4
    ori_threshold: float = 1e-4
    max_iters: int = 20
    action_scale: float = 0.01
    
    home_pose: List[float] = field(default_factory=lambda: deg2rad([0.0, -60.0, 21.0, 40.3]))
    gripper_open_pos: float = -0.02
    gripper_closed_pos: float = -0.01
    
    joint_names: List[str] = field(default_factory=lambda: [
        'joint1', 'joint2', 'joint3', 'joint4', 'gripper_left_joint'
    ])

class SystemLogger:
    """Structured logging with different levels"""
    
    def __init__(self, node_logger):
        self.logger = node_logger
        self.loop_count = 0
        
    def log_state_transition(self, from_state: SystemState, to_state: SystemState, context: str = ""):
        """Log state machine transitions"""
        self.logger.info(f"State transition: {from_state.value} -> {to_state.value} {context}")
        
    def log_action_result(self, action_type: str, action_id: str, result: str, duration: float):
        """Log action completion results"""
        self.logger.info(f"Action {action_type}[{action_id}]: {result} ({duration:.2f}s)")
        
    def log_debug_periodic(self, message: str, frequency: int = 1000):
        """Log debug messages periodically"""
        self.loop_count += 1
        if self.loop_count % frequency == 0:
            self.logger.debug(f"[Loop {self.loop_count}] {message}")

class AsyncActionManager:
    """Manages robot actions without blocking main thread"""
    
    def __init__(self, node: Node, config: SystemConfig):
        self.node = node
        self.config = config
        self.pending_actions: Dict[str, Dict] = {}
        self.action_results: Dict[str, Dict] = {}
        
        # Action clients
        self.arm_client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(node, GripperCommand, '/gripper_controller/gripper_cmd')
        
    def wait_for_servers(self, timeout: float = 10.0) -> bool:
        """Wait for action servers to become available"""
        arm_ready = self.arm_client.wait_for_server(timeout_sec=timeout)
        gripper_ready = self.gripper_client.wait_for_server(timeout_sec=timeout)
        return arm_ready and gripper_ready
        
    def send_joint_goal_async(self, joints: List[float], action_id: str, time_step: float = 2.0):
        """Send joint trajectory goal asynchronously"""
        goal_msg = FollowJointTrajectory.Goal()
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.config.joint_names[:4]  # Only arm joints
        
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start.sec = int(time_step)
        point.time_from_start.nanosec = int((time_step - int(time_step)) * 1e9)
        
        trajectory.points = [point]
        goal_msg.trajectory = trajectory
        
        # Send goal and set up callback
        future = self.arm_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self._on_arm_goal_response(f, action_id))
        
        self.pending_actions[action_id] = {
            'type': 'arm',
            'future': future,
            'start_time': time.time(),
            'timeout': self.config.action_timeout,
            'goal_handle': None
        }
        
    def send_gripper_goal_async(self, position: float, action_id: str, time_step: float = 1.0):
        """Send gripper command asynchronously"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 10.0
        
        future = self.gripper_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self._on_gripper_goal_response(f, action_id))
        
        self.pending_actions[action_id] = {
            'type': 'gripper',
            'future': future,
            'start_time': time.time(),
            'timeout': self.config.action_timeout,
            'goal_handle': None
        }
        
    def check_action_status(self, action_id: str) -> Dict[str, Any]:
        """Check if action completed, failed, or timed out"""
        if action_id in self.action_results:
            return self.action_results[action_id]
        elif action_id in self.pending_actions:
            action = self.pending_actions[action_id]
            elapsed = time.time() - action['start_time']
            if elapsed > action['timeout']:
                self._mark_action_timeout(action_id)
                return {'status': 'timeout', 'elapsed': elapsed}
            return {'status': 'pending', 'elapsed': elapsed}
        return {'status': 'unknown'}
        
    def _on_arm_goal_response(self, future, action_id: str):
        """Handle arm goal response"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.action_results[action_id] = {'status': 'rejected'}
                self._cleanup_action(action_id)
                return
                
            # Store goal handle and wait for result
            self.pending_actions[action_id]['goal_handle'] = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f: self._on_arm_result(f, action_id))
            
        except Exception as e:
            self.action_results[action_id] = {'status': 'error', 'error': str(e)}
            self._cleanup_action(action_id)
            
    def _on_gripper_goal_response(self, future, action_id: str):
        """Handle gripper goal response"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.action_results[action_id] = {'status': 'rejected'}
                self._cleanup_action(action_id)
                return
                
            # Store goal handle and wait for result
            self.pending_actions[action_id]['goal_handle'] = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f: self._on_gripper_result(f, action_id))
            
        except Exception as e:
            self.action_results[action_id] = {'status': 'error', 'error': str(e)}
            self._cleanup_action(action_id)
            
    def _on_arm_result(self, future, action_id: str):
        """Handle arm action result"""
        try:
            result_response = future.result()
            if result_response and result_response.result:
                result = result_response.result
                if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                    self.action_results[action_id] = {'status': 'success'}
                else:
                    self.action_results[action_id] = {
                        'status': 'failed', 
                        'error_code': result.error_code
                    }
            else:
                self.action_results[action_id] = {'status': 'failed', 'error': 'No result'}
        except Exception as e:
            self.action_results[action_id] = {'status': 'error', 'error': str(e)}
        finally:
            self._cleanup_action(action_id)
            
    def _on_gripper_result(self, future, action_id: str):
        """Handle gripper action result"""
        try:
            result_response = future.result()
            if result_response and result_response.result:
                result = result_response.result
                if result.reached_goal or result.stalled:  # Consider stalled as success
                    self.action_results[action_id] = {'status': 'success', 'position': result.position}
                else:
                    self.action_results[action_id] = {
                        'status': 'failed', 
                        'position': result.position,
                        'stalled': result.stalled
                    }
            else:
                self.action_results[action_id] = {'status': 'failed', 'error': 'No result'}
        except Exception as e:
            self.action_results[action_id] = {'status': 'error', 'error': str(e)}
        finally:
            self._cleanup_action(action_id)
            
    def _mark_action_timeout(self, action_id: str):
        """Mark action as timed out and attempt to cancel"""
        self.action_results[action_id] = {'status': 'timeout'}
        
        # Try to cancel the goal
        if action_id in self.pending_actions:
            action = self.pending_actions[action_id]
            if action.get('goal_handle'):
                try:
                    action['goal_handle'].cancel_goal_async()
                except Exception:
                    pass  # Ignore cancel errors
                    
        self._cleanup_action(action_id)
        
    def _cleanup_action(self, action_id: str):
        """Clean up completed action"""
        if action_id in self.pending_actions:
            del self.pending_actions[action_id]

class MuJoCoSystem:
    """Handles all MuJoCo operations"""
    
    def __init__(self, model_path: Path, config: SystemConfig):
        self.config = config
        self.model = mujoco.MjModel.from_xml_path(model_path.as_posix())
        self.data = mujoco.MjData(self.model)
        
        # Initialize mink configuration and tasks
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
        
    def sync_with_robot(self, joint_positions: List[float]):
        """Sync MuJoCo state with real robot"""
        # Set arm joint positions
        for i in range(min(4, len(joint_positions))):
            if joint_positions[i] is not None:
                self.data.qpos[i] = joint_positions[i]
        
        # Set gripper position if available
        if len(joint_positions) > 4 and joint_positions[4] is not None:
            if len(self.data.qpos) > 4:
                self.data.qpos[4] = joint_positions[4]
        
        # Update mink configuration and perform forward kinematics
        self.configuration.update(self.data.qpos)
        mujoco.mj_forward(self.model, self.data)
        self.posture_task.set_target_from_configuration(self.configuration)
        mink.move_mocap_to_frame(self.model, self.data, "target", "pinch_site", "site")
        
    def update_from_webxr(self, target_pos: np.ndarray, target_quat: np.ndarray):
        """Update MuJoCo targets from WebXR"""
        self.data.mocap_pos[0] = target_pos
        self.data.mocap_quat[0] = target_quat
        
    def get_ik_solution(self, dt: float) -> np.ndarray:
        """Get IK solution for current targets"""
        # Update the end effector task target from the mocap body
        T_wt = mink.SE3.from_mocap_name(self.model, self.data, "target")
        self.end_effector_task.set_target(T_wt)
        
        # Perform IK
        self._converge_ik(dt)
        
        # Return target joint positions (only first 4 joints - arm)
        return self.configuration.q[:4].copy()
        
    def update_visualization(self, target_joints: np.ndarray, gripper_open: bool):
        """Update MuJoCo visualization with IK solution"""
        # Update control with IK solution
        self.data.ctrl[:4] = target_joints
        
        # Set gripper control
        gripper_width = 0.04 if gripper_open else 0.0
        self.data.ctrl[4] = gripper_width
        
        # Forward kinematics to update visualization
        mujoco.mj_forward(self.model, self.data)
        
    def get_end_effector_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get current end effector position and orientation"""
        return self.data.mocap_pos[0].copy(), self.data.mocap_quat[0].copy()
        
    def _converge_ik(self, dt: float) -> bool:
        """Run IK solver to convergence"""
        for _ in range(self.config.max_iters):
            vel = mink.solve_ik(self.configuration, self.tasks, dt, self.config.ik_solver, 1e-3)
            self.configuration.integrate_inplace(vel, dt)

            # Check convergence
            err = self.tasks[0].compute_error(self.configuration)
            pos_achieved = np.linalg.norm(err[:3]) <= self.config.pos_threshold
            ori_achieved = np.linalg.norm(err[3:]) <= self.config.ori_threshold

            if pos_achieved and ori_achieved:
                return True
        return False

class WebXRSystem:
    """Handles WebXR processing and calibration"""
    
    def __init__(self):
        self.calibrated = False
        self.pos_offset: Optional[np.ndarray] = None
        self.quat_diff: Optional[np.ndarray] = None
        self.initial_webxr_pos: Optional[np.ndarray] = None
        self.initial_webxr_quat: Optional[np.ndarray] = None
        
    def calibrate(self, current_ee_pos: np.ndarray, current_ee_quat: np.ndarray, 
                 webxr_pos: np.ndarray, webxr_quat: np.ndarray):
        """Calibrate WebXR to robot coordinate system"""
        self.initial_webxr_pos = webxr_pos.copy()
        self.initial_webxr_quat = webxr_quat.copy()
        
        # Calculate the quaternion difference
        self.quat_diff = quaternion_difference(self.initial_webxr_quat, current_ee_quat)
        
        # Calculate position offset
        self.pos_offset = current_ee_pos - self.initial_webxr_pos
        
        self.calibrated = True
        
    def process_webxr_input(self, webxr_pos: np.ndarray, webxr_quat: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Convert WebXR input to robot targets"""
        if not self.calibrated:
            return webxr_pos, webxr_quat
            
        # Calculate target position by applying the offset
        target_pos = webxr_pos + self.pos_offset
        
        # Calculate target quaternion
        target_quat = quaternion_multiply(self.quat_diff, webxr_quat)
        
        return target_pos, target_quat
        
    def reset_calibration(self):
        """Reset calibration state"""
        self.calibrated = False
        self.pos_offset = None
        self.quat_diff = None
        self.initial_webxr_pos = None
        self.initial_webxr_quat = None

class ErrorHandler:
    """Centralized error handling and recovery"""
    
    def __init__(self, logger: SystemLogger):
        self.logger = logger
        self.error_count = 0
        self.max_retries = 3
        
    def handle_initialization_error(self, error_type: str, context: str) -> SystemState:
        """Handle errors during initialization"""
        self.error_count += 1
        self.logger.logger.error(f"Initialization error ({error_type}): {context}")
        
        if self.error_count < self.max_retries:
            self.logger.logger.info(f"Attempting recovery (attempt {self.error_count}/{self.max_retries})")
            return SystemState.ERROR_RECOVERY
        else:
            self.logger.logger.error("Max retries exceeded, marking as failed")
            return SystemState.FAILED
            
    def handle_runtime_error(self, error_type: str, context: str) -> SystemState:
        """Handle errors during normal operation"""
        self.logger.logger.error(f"Runtime error ({error_type}): {context}")
        return SystemState.ERROR_RECOVERY
        
    def reset_error_count(self):
        """Reset error count after successful operation"""
        self.error_count = 0

class OmxRealController(Node):
    """Main controller with single-threaded main loop"""
    
    def __init__(self):
        super().__init__('omx_real_webxr_v2_node')
        
        # Initialize configuration and logging
        self.config = SystemConfig()
        self.logger = SystemLogger(self.get_logger())
        self.error_handler = ErrorHandler(self.logger)
        
        # Initialize data storage
        self.robot_data = RobotData()
        
        # Initialize state machine
        self.state = SystemState.STARTING
        self.state_start_time = time.time()
        
        # Initialize systems
        self.mujoco_system = MuJoCoSystem(_XML, self.config)
        self.webxr_system = WebXRSystem()
        self.action_manager = AsyncActionManager(self, self.config)
        
        # Viewer will be initialized in main loop
        self.viewer = None
        
        # Initialize ROS subscriptions
        self._setup_ros_subscriptions()
        
        self.get_logger().info('OmxRealController V2 initialized')
        
    def _setup_ros_subscriptions(self):
        """Set up ROS subscriptions with data-only callbacks"""
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        self.webxr_sub = self.create_subscription(
            WebXRControl,
            '/webxr/control',
            self._webxr_control_callback,
            10
        )
        
    def _joint_state_callback(self, joint_state: JointState):
        """Handle incoming joint state data - DATA ONLY, NO LOGIC"""
        joint_pos_ordered = [None] * len(self.config.joint_names)
        
        # Order the joint positions according to our joint names
        for i, name in enumerate(joint_state.name):
            try:
                joint_idx = self.config.joint_names.index(name)
                joint_pos_ordered[joint_idx] = joint_state.position[i]
            except ValueError:
                continue  # Unknown joint name
        
        # Store if we have all required joints
        if all(pos is not None for pos in joint_pos_ordered):
            self.robot_data.update_joints(joint_pos_ordered)
    
    def _webxr_control_callback(self, msg: WebXRControl):
        """Handle incoming WebXR control data - DATA ONLY, NO LOGIC"""
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, 
                        msg.pose.orientation.z, msg.pose.orientation.w])
        
        self.robot_data.update_webxr(pos, quat, msg.gripper_open, msg.move_enabled)
    
    def run_main_loop(self):
        """Single-threaded main loop - handles everything"""
        self.get_logger().info("Starting main loop...")
        
        # Initialize MuJoCo viewer
        self.viewer = mujoco.viewer.launch_passive(
            model=self.mujoco_system.model, 
            data=self.mujoco_system.data, 
            show_left_ui=False, 
            show_right_ui=False
        )
        
        mujoco.mjv_defaultFreeCamera(self.mujoco_system.model, self.viewer.cam)
        rate = RateLimiter(frequency=self.config.loop_frequency, warn=False)
        
        try:
            with self.viewer:
                while rclpy.ok() and self.viewer.is_running():
                    # 1. Process ROS messages (non-blocking)
                    rclpy.spin_once(self, timeout_sec=0)
                    
                    # 2. Update state machine
                    self._update_state_machine()
                    
                    # 3. Update MuJoCo based on current state
                    self._update_mujoco()
                    
                    # 4. Process WebXR if ready
                    if self.state == SystemState.WEBXR_ACTIVE:
                        self._process_webxr()
                    
                    # 5. Render and sync
                    self.viewer.sync()
                    rate.sleep()
                    
        except KeyboardInterrupt:
            self.get_logger().info("Keyboard interrupt received")
        except Exception as e:
            self.get_logger().error(f"Main loop error: {e}")
        finally:
            self.get_logger().info("Main loop finished")
    
    def _update_state_machine(self):
        """State machine logic - no blocking operations"""
        old_state = self.state
        state_elapsed = time.time() - self.state_start_time
        
        if self.state == SystemState.STARTING:
            # Wait for action servers
            if self.action_manager.wait_for_servers(timeout=0.1):  # Non-blocking check
                self.state = SystemState.WAITING_FOR_JOINTS
                
        elif self.state == SystemState.WAITING_FOR_JOINTS:
            if self.robot_data.current_joints is not None:
                self._start_mujoco_sync()
                self.state = SystemState.SYNCING_MUJOCO
                
        elif self.state == SystemState.SYNCING_MUJOCO:
            self._complete_mujoco_sync()
            self._start_home_motion()
            self.state = SystemState.GOING_TO_HOME
            
        elif self.state == SystemState.GOING_TO_HOME:
            result = self.action_manager.check_action_status('home')
            if result['status'] == 'success':
                self.logger.log_action_result('home', 'home', 'success', result.get('elapsed', 0))
                self._start_gripper_test()
                self.state = SystemState.TESTING_GRIPPER_OPEN
            elif result['status'] in ['failed', 'timeout', 'error']:
                self.state = self.error_handler.handle_initialization_error('home_motion', str(result))
                
        elif self.state == SystemState.TESTING_GRIPPER_OPEN:
            result = self.action_manager.check_action_status('gripper_open')
            if result['status'] == 'success':
                self.logger.log_action_result('gripper', 'gripper_open', 'success', result.get('elapsed', 0))
                self._start_gripper_close()
                self.state = SystemState.TESTING_GRIPPER_CLOSE
            elif result['status'] in ['failed', 'timeout', 'error']:
                self.state = self.error_handler.handle_initialization_error('gripper_open', str(result))
                
        elif self.state == SystemState.TESTING_GRIPPER_CLOSE:
            result = self.action_manager.check_action_status('gripper_close')
            if result['status'] == 'success':
                self.logger.log_action_result('gripper', 'gripper_close', 'success', result.get('elapsed', 0))
                self.error_handler.reset_error_count()
                self.state = SystemState.READY_FOR_WEBXR
            elif result['status'] in ['failed', 'timeout', 'error']:
                self.state = self.error_handler.handle_initialization_error('gripper_close', str(result))
                
        elif self.state == SystemState.READY_FOR_WEBXR:
            # Transition to active when WebXR data is available and move enabled
            if (self.robot_data.webxr_pos is not None and 
                self.robot_data.move_enabled and 
                not self.robot_data.prev_move_enabled):
                self._start_webxr_mode()
                self.state = SystemState.WEBXR_ACTIVE
                
        elif self.state == SystemState.WEBXR_ACTIVE:
            # Stay active while move is enabled
            if not self.robot_data.move_enabled:
                self.state = SystemState.READY_FOR_WEBXR
                
        elif self.state == SystemState.ERROR_RECOVERY:
            # Simple recovery: go back to waiting for joints after delay
            if state_elapsed > 5.0:  # Wait 5 seconds before retry
                self.state = SystemState.WAITING_FOR_JOINTS
                
        # Log state transitions
        if old_state != self.state:
            self.logger.log_state_transition(old_state, self.state)
            self.state_start_time = time.time()
    
    def _start_mujoco_sync(self):
        """Start MuJoCo synchronization with real robot"""
        if self.robot_data.current_joints:
            self.mujoco_system.sync_with_robot(self.robot_data.current_joints)
            self.get_logger().info("MuJoCo synced with real robot position")
    
    def _complete_mujoco_sync(self):
        """Complete MuJoCo synchronization"""
        pass  # Already done in _start_mujoco_sync
        
    def _start_home_motion(self):
        """Start motion to home position"""
        self.action_manager.send_joint_goal_async(self.config.home_pose, 'home', 2.0)
        self.get_logger().info("Sending robot to home position")
        
    def _start_gripper_test(self):
        """Start gripper opening test"""
        self.action_manager.send_gripper_goal_async(self.config.gripper_open_pos, 'gripper_open', 1.0)
        self.get_logger().info("Testing gripper - opening")
        
    def _start_gripper_close(self):
        """Start gripper closing test"""
        self.action_manager.send_gripper_goal_async(self.config.gripper_closed_pos, 'gripper_close', 1.0)
        self.get_logger().info("Testing gripper - closing")
        
    def _start_webxr_mode(self):
        """Initialize WebXR mode"""
        if (self.robot_data.webxr_pos is not None and 
            self.robot_data.webxr_quat is not None):
            
            current_ee_pos, current_ee_quat = self.mujoco_system.get_end_effector_pose()
            self.webxr_system.calibrate(
                current_ee_pos, current_ee_quat,
                self.robot_data.webxr_pos, self.robot_data.webxr_quat
            )
            self.get_logger().info("WebXR calibrated and activated")
    
    def _update_mujoco(self):
        """Update MuJoCo visualization with current robot state"""
        if self.robot_data.current_joints:
            self.mujoco_system.sync_with_robot(self.robot_data.current_joints)
    
    def _process_webxr(self):
        """Process WebXR input and send commands to robot"""
        if (self.robot_data.webxr_pos is None or 
            self.robot_data.webxr_quat is None or 
            not self.webxr_system.calibrated):
            return
            
        # Convert WebXR input to robot targets
        target_pos, target_quat = self.webxr_system.process_webxr_input(
            self.robot_data.webxr_pos, self.robot_data.webxr_quat
        )
        
        # Update MuJoCo with WebXR targets
        self.mujoco_system.update_from_webxr(target_pos, target_quat)
        
        # Get IK solution
        dt = 1.0 / self.config.loop_frequency
        target_joints = self.mujoco_system.get_ik_solution(dt)
        
        # Update visualization
        self.mujoco_system.update_visualization(target_joints, self.robot_data.gripper_open)
        
        # Send commands to real robot at reduced frequency
        current_time = time.time()
        if not hasattr(self, '_last_command_time'):
            self._last_command_time = 0
            
        if current_time - self._last_command_time > 0.25:  # 4Hz command rate
            # Send arm command
            self.action_manager.send_joint_goal_async(
                target_joints.tolist(), f'webxr_arm_{int(current_time)}', 0.25
            )
            
            # Send gripper command if state changed
            if self.robot_data.gripper_open != self.robot_data.prev_gripper_open:
                gripper_pos = (self.config.gripper_open_pos if self.robot_data.gripper_open 
                             else self.config.gripper_closed_pos)
                self.action_manager.send_gripper_goal_async(
                    gripper_pos, f'webxr_gripper_{int(current_time)}', 0.5
                )
                
            self._last_command_time = current_time

def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        controller = OmxRealController()
        controller.run_main_loop()
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()