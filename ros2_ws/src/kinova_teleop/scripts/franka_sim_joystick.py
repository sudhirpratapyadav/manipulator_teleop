#!/usr/bin/env python3

from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist, Quaternion

import mink

from ament_index_python.packages import get_package_share_directory
_HERE = Path(get_package_share_directory("kinova_teleop")) / "models"
_XML = _HERE / "franka_emika_panda" / "mjx_scene.xml"

# IK parameters
SOLVER = "quadprog"
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


class FrankaSimNode(Node):
    def __init__(self):
        super().__init__('franka_sim_node')
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
        self.delta_ee = np.zeros(6)  # [dx, dy, dz, droll, dpitch, dyaw]
        self.teleop_ee_quat = np.array([0, 0, 0, 1])  # [x, y, z, w] - identity quaternion
        
        # Quaternion synchronization variables
        self.initial_robot_ee_quat = None  # Initial robot EE quaternion
        self.initial_teleop_quat = None   # Initial teleop quaternion
        self.quat_diff = None             # Difference quaternion between initial robot and teleop
        self.quat_initialized = False     # Flag to check if quaternions are initialized
        
        self.sub = self.create_subscription(
            Twist,
            '/delta_ee',
            self.twist_callback,
            10
        )

        self.quat_sub = self.create_subscription(
            Quaternion,
            '/teleop/quat_abs',
            self.quat_callback,
            10
        )
        self.get_logger().info('FrankaSimNode started, waiting for /delta_ee Twist messages.')

    def twist_callback(self, msg):
        self.delta_ee = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z
        ])
        self.delta_ee *= ACTION_SCALE
    
    def quat_callback(self, msg):
        current_teleop_quat = np.array([msg.x, msg.y, msg.z, msg.w])
        
        # If this is the first quaternion we receive, use it as a reference
        if not self.quat_initialized and self.initial_robot_ee_quat is not None:
            self.initial_teleop_quat = current_teleop_quat.copy()
            # Calculate the quaternion difference (quat_diff = robot_ee * teleop^-1)
            self.quat_diff = quaternion_difference(self.initial_teleop_quat, self.initial_robot_ee_quat)
            self.quat_initialized = True
            self.get_logger().info('Quaternion synchronization initialized')
            
        # Update the current teleop quaternion
        self.teleop_ee_quat = current_teleop_quat

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
            # Store initial mocap position
            initial_target_position = self.data.mocap_pos[0].copy()
            # Store initial robot EE quaternion
            self.initial_robot_ee_quat = self.data.mocap_quat[0].copy()
            
            rate = RateLimiter(frequency=200.0, warn=False)
            while rclpy.ok() and viewer.is_running():
                rclpy.spin_once(self, timeout_sec=0)
                dt = rate.dt
                # Use the latest delta_ee from the ROS topic
                delta = self.delta_ee.copy()
                
                # Apply quaternion transformation if initialized
                if self.quat_initialized:
                    # Calculate target quaternion based on the current teleop quaternion and the initial difference
                    # target_quat = quat_diff * teleop_quat
                    current_teleop_quat = self.teleop_ee_quat.copy()
                    target_quat = quaternion_multiply(self.quat_diff, current_teleop_quat)
                else:
                    # Before initialization, use the initial robot quaternion
                    target_quat = self.initial_robot_ee_quat
                
                # Move the mocap target (target box) in the simulation
                self.data.mocap_pos[0] += delta[:3]
                self.data.mocap_quat[0] = target_quat
                
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
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = FrankaSimNode()
    try:
        node.sim_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
