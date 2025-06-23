#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
import pybullet as p
import pybullet_data
import numpy as np
import time

class EESimNode(Node):
    def __init__(self):
        super().__init__('ee_sim_node')

        # Linear movement variables
        self.delta_ee = np.zeros(3)  # [dx, dy, dz]
        self.ee_pose = np.zeros(3)  # [x, y, z]

        # Absolute orientation as quaternion
        self.ee_quat = [0, 0, 0, 1]  # [x, y, z, w] - identity quaternion

        self.sub = self.create_subscription(
            Twist,
            '/delta_ee',
            self.twist_callback,
            10
        )

        self.quat_sub = self.create_subscription(
            Quaternion,
            '/target_rot_quat',
            self.quat_callback,
            10
        )

        self.dt = 0.01  # simulation timestep
        self.action_scale = 0.1

        # Debug visualization flag
        self.debug_viz = False

        # PyBullet setup
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.ee_id = p.loadURDF("cube_small.urdf", [0, 0, 0])
        self.frame_ids = []
        if self.debug_viz:
            self.create_frame([0, 0, 0], self.ee_quat)

    def twist_callback(self, msg):
        self.delta_ee = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.linear.z
        ])

    def quat_callback(self, msg):
        self.ee_quat = [msg.x, msg.y, msg.z, msg.w]

    def create_frame(self, pos, orn):
        if not self.debug_viz:
            return
        rot_matrix = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        origin = np.array(pos)
        axis_length = 0.1
        x_axis = origin + rot_matrix[:, 0] * axis_length
        y_axis = origin + rot_matrix[:, 1] * axis_length
        z_axis = origin + rot_matrix[:, 2] * axis_length
        self.frame_ids = [
            p.addUserDebugLine(origin, x_axis, [1, 0, 0], lineWidth=3, replaceItemUniqueId=-1),
            p.addUserDebugLine(origin, y_axis, [0, 1, 0], lineWidth=3, replaceItemUniqueId=-1),
            p.addUserDebugLine(origin, z_axis, [0, 0, 1], lineWidth=3, replaceItemUniqueId=-1)
        ]

    def update_frame(self, pos, orn):
        if not self.debug_viz:
            return
        if not self.frame_ids or len(self.frame_ids) != 3:
            return
        rot_matrix = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        origin = np.array(pos)
        axis_length = 0.1
        x_axis = origin + rot_matrix[:, 0] * axis_length
        y_axis = origin + rot_matrix[:, 1] * axis_length
        z_axis = origin + rot_matrix[:, 2] * axis_length
        p.addUserDebugLine(origin, x_axis, [1, 0, 0], lineWidth=3, replaceItemUniqueId=self.frame_ids[0])
        p.addUserDebugLine(origin, y_axis, [0, 1, 0], lineWidth=3, replaceItemUniqueId=self.frame_ids[1])
        p.addUserDebugLine(origin, z_axis, [0, 0, 1], lineWidth=3, replaceItemUniqueId=self.frame_ids[2])

    def update_ee(self):
        self.ee_pose += self.delta_ee * self.action_scale
        pos = self.ee_pose
        orn = self.ee_quat
        p.resetBasePositionAndOrientation(self.ee_id, pos.tolist(), orn)
        if self.debug_viz:
            self.update_frame(pos, orn)

    def sim_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            self.update_ee()
            p.stepSimulation()
            time.sleep(self.dt)

def main(args=None):
    rclpy.init(args=args)
    node = EESimNode()
    try:
        node.sim_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        p.disconnect()

if __name__ == '__main__':
    main()
