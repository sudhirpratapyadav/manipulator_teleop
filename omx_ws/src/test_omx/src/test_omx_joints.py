#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from threading import Thread

import time
from threading import Event

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

##
import numpy as np
import math
import os
import pathlib
import sys

OPEN_GRIPPER = [-0.02]
CLOSE_GRIPER = [-0.01]


def rad2deg(joint_pos_rad):
	if isinstance(joint_pos_rad, list):
		if isinstance(joint_pos_rad[0], list):
			joint_pos_deg = [[angle*(180/math.pi) for angle in lst] for lst in joint_pos_rad]
		else:
			joint_pos_deg = [angle*(180/math.pi) for angle in joint_pos_rad]
	else:
		joint_pos_deg = joint_pos_rad*(180/math.pi)
	return joint_pos_deg

def deg2rad(joint_pos_deg):
	if isinstance(joint_pos_deg, list):
		if isinstance(joint_pos_deg[0], list):
			joint_pos_rad = [ [angle*(math.pi/180) for angle in lst] for lst in joint_pos_deg]
		else:
			joint_pos_rad = [angle*(math.pi/180) for angle in joint_pos_deg]
	else:
		joint_pos_rad = joint_pos_deg*(math.pi/180)
	return joint_pos_rad

def print_format_list(lst):
	return ["{0:0.2f}".format(num) for num in lst]

np.set_printoptions(precision=3, suppress=True)

class OpenManipulatorControl(Node):

	def __init__(self):
		super().__init__('open_manipulator_control_node')

		# Joints
		self.joint_names = [
		'joint1',
		'joint2',
		'joint3',
		'joint4',
		'gripper',
		]

		self.init_success = False

		self.HOME_POSE = deg2rad([0.0, -60.0, 21.0, 40.3])
		
        
		self.RESET_POSE = deg2rad([5.979, -116.354,  86.520, 30.506])

		# Subscriber for joint positions
		self.joint_state_sub = self.create_subscription(
			JointState,
			"/joint_states",
			self.joint_state_callback,
			10
		)

		# Action clients for trajectory control
		self.arm_action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
		self.gripper_action_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

		# Wait for action servers to be available
		self.get_logger().info('Waiting for arm action server...')
		self.arm_action_client.wait_for_server()
		self.get_logger().info('Arm action server connected!')
		
		self.get_logger().info('Waiting for gripper action server...')
		self.gripper_action_client.wait_for_server()
		self.get_logger().info('Gripper action server connected!')
		

	def init_robot(self):

		temp_successful = True
		self.get_logger().info(f'\n\n----------------- Initialising Robot ----------------\n')

		self.get_logger().info('Sending to Home Position')
		result = self.send_joint_pose_goal(self.HOME_POSE, action_name='go_to_home_pose', time_step=2.0)
		if result is True:
			self.get_logger().info('Sending to Home Position: successfull\n')
		else:
			self.get_logger().info('Sending to Home Position: Unsuccessfull\n')
			temp_successful = False
		time.sleep(2.5)	

		self.get_logger().info('Checking Gripper: Opening and Closing\n')
		result = self.send_gripper_command(joint_pose_goal=OPEN_GRIPPER, action_name='Opening Gripper', time_step=1.0)
		if result is True:
			self.get_logger().info('Openining Gripper: successfull\n')
		else:
			self.get_logger().info('Openining Gripper: Unsuccessfull\n')
			temp_successful = False
		time.sleep(1.0)
		result = self.send_gripper_command(joint_pose_goal=CLOSE_GRIPER, action_name='Closing Gripper', time_step=1.0)
		if result is True:
			self.get_logger().info('Closing Gripper: successfull\n')
		else:
			self.get_logger().info('Closing Gripper: Unsuccessfull\n')
			temp_successful = False
		time.sleep(1.0)
		self.get_logger().info('Checking Gripper: successfull\n')

		if temp_successful:
			self.get_logger().info(f'\n\n-------------- Robot Successfully Initialised----------\n\n\n')
			self.init_success = True
		else:
			self.get_logger().info(f'\n\n-------------- FAILED to Initialise Robot ----------\n\n\n')
			self.init_success = False



	def reset(self):
		self.get_logger().info('Sending to RESET Position')
		result = self.send_joint_pose_goal(self.RESET_POSE, action_name='RESET', time_step=2.0)
		if result is True:
			self.get_logger().info('Sending to RESET Position: successfull\n')
		else:
			self.get_logger().info('Sending to RESET Position: Unsuccessfull\n')
			temp_successful = False
		time.sleep(2.5)	


	def joint_state_callback(self, joint_state):

		joint_pos_ordered = [None]*len(joint_state.position)

		for i, name in enumerate(joint_state.name):
			if name==self.joint_names[0]:
				joint_pos_ordered[0] = joint_state.position[i]
			elif name==self.joint_names[1]:
				joint_pos_ordered[1] = joint_state.position[i]
			elif name==self.joint_names[2]:
				joint_pos_ordered[2] = joint_state.position[i]
			elif name==self.joint_names[3]:
				joint_pos_ordered[3] = joint_state.position[i]
			elif name==self.joint_names[4]:
				joint_pos_ordered[4] = joint_state.position[i]

				self.current_joint_position = joint_pos_ordered
				# if self.init_success:
				self.get_logger().info(f'joint_angles: {rad2deg(self.current_joint_position)}')

	def send_joint_pose_goal(self, joint_pose_goal, action_name='move', time_step=2.0):
		# This function sends a trajectory goal to the arm controller
		
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

		self.get_logger().info(f'Exec: {action_name}')
		
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
			return True
		else:
			self.get_logger().error(f'Trajectory execution failed with error code: {result.error_code}')
			return False


	def send_gripper_command(self, joint_pose_goal, action_name='move', time_step=2.0):
		# This function sends a gripper command to the gripper controller
		
		goal_msg = GripperCommand.Goal()
		
		# Set gripper position (joint_pose_goal should be a single value list)
		goal_msg.command.position = joint_pose_goal[0] if isinstance(joint_pose_goal, list) else joint_pose_goal
		goal_msg.command.max_effort = 10.0  # Set a reasonable max effort

		self.get_logger().info(f'Exec: {action_name} - Position: {goal_msg.command.position}')
		
		# Send goal and wait for result
		send_goal_future = self.gripper_action_client.send_goal_async(goal_msg)
		rclpy.spin_until_future_complete(self, send_goal_future)
		
		goal_handle = send_goal_future.result()
		if not goal_handle.accepted:
			self.get_logger().error('Gripper goal rejected')
			return False
		
		# Wait for result
		result_future = goal_handle.get_result_async()
		rclpy.spin_until_future_complete(self, result_future)
		
		result = result_future.result().result
		if result.reached_goal:
			self.get_logger().info(f'Gripper reached goal position: {result.position}')
			return True
		elif result.stalled:
			self.get_logger().info(f'Gripper stalled but close to goal. Final position: {result.position}')
			return True  # Consider stalled as success for gripper closing
		else:
			self.get_logger().error(f'Gripper failed to reach goal. Final position: {result.position}, Stalled: {result.stalled}')
			return False


def main(args=None):

	rclpy.init(args=args)

	robot_arm = OpenManipulatorControl()

	time.sleep(1.0)
	robot_arm.init_robot()
	time.sleep(4.0)
	robot_arm.reset()
	time.sleep(4.0)

	try:
		rclpy.spin(robot_arm)
	except KeyboardInterrupt:
		pass
	finally:
		if rclpy.ok():
			robot_arm.destroy_node()
			rclpy.shutdown()


if __name__ == "__main__":
	main()