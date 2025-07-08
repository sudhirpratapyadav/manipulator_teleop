#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from kinova_teleop.msg import WebXRControl
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
import math
import time


class KinovaTestNode(Node):
    def __init__(self):
        super().__init__('kinova_test_node')
        
        # Create publisher for WebXR control messages
        self.webxr_pub = self.create_publisher(
            WebXRControl,
            '/webxr/control',
            10
        )
        
        # Timer for publishing test messages
        self.timer = self.create_timer(0.1, self.publish_test_message)  # 10 Hz
        
        # Initial position (will be used as reference)
        self.initial_pos = [0.5, 0.0, 0.5]  # x, y, z in meters
        
        # Test sequence variables
        self.phase_start_time = time.time()
        self.phase_duration = 15.0  # 3 seconds per phase
        
        self.get_logger().info('Kinova Test Node started - Pitch (Y-axis) test only')

    def create_webxr_message(self, position, quaternion, gripper_open, move_enabled):
        """Create a WebXRControl message with the given parameters"""
        msg = WebXRControl()
        
        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "webxr_controller"
        
        # Pose
        msg.pose = Pose()
        msg.pose.position = Point(x=position[0], y=position[1], z=position[2])
        msg.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        
        # Control flags
        msg.gripper_open = gripper_open
        msg.move_enabled = move_enabled
        
        return msg

    def publish_test_message(self):
        """Publish test messages: pitch test, then return to initial position"""
        current_time = time.time()
        phase_time = current_time - self.phase_start_time
        total_duration = 2 * self.phase_duration  # One round pitch, one round return
        t = phase_time % total_duration
        position = self.initial_pos.copy()
        gripper_open = False
        move_enabled = True
        if t < self.phase_duration:
            # Pitch (Y-axis) test
            norm_t = t / self.phase_duration
            pitch_angle = 0.6 * math.sin(2 * math.pi * norm_t)
            quaternion = [0.0, math.sin(pitch_angle/2), 0.0, math.cos(pitch_angle/2)]
        else:
            # Return to initial orientation
            norm_t = (t - self.phase_duration) / self.phase_duration
            # Interpolate from last pitch quaternion back to identity
            # Last pitch quaternion at end of phase: pitch_angle = 0
            # Use ease-out cubic for smooth return
            ease_t = 1 - (1 - norm_t) ** 3
            # Slerp from current to identity quaternion
            # For pitch, slerp from [0,0,0,1] to [0,0,0,1] (since pitch ends at 0)
            # But for smoothness, let's interpolate from last pitch quaternion (which is identity)
            quaternion = [0.0, 0.0, 0.0, 1.0]
        msg = self.create_webxr_message(position, quaternion, gripper_open, move_enabled)
        self.webxr_pub.publish(msg)
        if int(phase_time * 10) % 10 == 0:
            if t < self.phase_duration:
                self.get_logger().debug(
                    f'Pitch test, t={phase_time:.1f}s: '
                    f'pos=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}), '
                    f'gripper={"open" if gripper_open else "closed"}, '
                    f'move_enabled={move_enabled}'
                )
            else:
                self.get_logger().debug(
                    f'Returning to initial orientation, t={phase_time:.1f}s: '
                    f'pos=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}), '
                    f'gripper={"open" if gripper_open else "closed"}, '
                    f'move_enabled={move_enabled}'
                )


def main(args=None):
    rclpy.init(args=args)
    node = KinovaTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down test node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()