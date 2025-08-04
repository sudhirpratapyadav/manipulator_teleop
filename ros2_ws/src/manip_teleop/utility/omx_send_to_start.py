#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def deg2rad(joint_pos_deg):
    if isinstance(joint_pos_deg, list):
        joint_pos_rad = [angle*(math.pi/180) for angle in joint_pos_deg]
    else:
        joint_pos_rad = joint_pos_deg*(math.pi/180)
    return joint_pos_rad

class OpenManipulatorStartPos(Node):

    def __init__(self):
        super().__init__('open_manipulator_start_pos_node')

        # Joint names
        self.joint_names = [
            'joint1',
            'joint2', 
            'joint3',
            'joint4',
        ]

        # Target position in degrees
        self.START_POSE = deg2rad([5.979, -116.354, 86.520, 30.506])

        # Action client for trajectory control
        self.arm_action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        # Wait for action server to be available
        self.get_logger().info('Waiting for arm action server...')
        self.arm_action_client.wait_for_server()
        self.get_logger().info('Arm action server connected!')

    def send_joint_pose_goal(self, joint_pose_goal, time_step=3.0):
        # Create trajectory goal
        goal_msg = FollowJointTrajectory.Goal()
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_pose_goal
        point.time_from_start.sec = int(time_step)
        point.time_from_start.nanosec = int((time_step - int(time_step)) * 1e9)
        
        trajectory.points = [point]
        goal_msg.trajectory = trajectory

        self.get_logger().info('Sending robot to start position...')
        
        # Send goal and wait for result
        send_goal_future = self.arm_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Successfully reached start position!')
            return True
        else:
            self.get_logger().error(f'Trajectory execution failed with error code: {result.error_code}')
            return False

    def move_to_start_position(self):
        success = self.send_joint_pose_goal(self.START_POSE)
        return success

def main(args=None):
    rclpy.init(args=args)

    robot_arm = OpenManipulatorStartPos()
    
    time.sleep(1.0)
    
    # Move to start position
    success = robot_arm.move_to_start_position()
    
    if success:
        robot_arm.get_logger().info('Robot successfully moved to start position')
    else:
        robot_arm.get_logger().error('Failed to move robot to start position')
    
    time.sleep(2.0)

    robot_arm.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()