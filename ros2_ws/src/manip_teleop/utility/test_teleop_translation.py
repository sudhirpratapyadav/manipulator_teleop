#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from manip_teleop.msg import WebXRControl
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
import time


class KinovaTranslationTestNode(Node):
    def __init__(self):
        super().__init__('kinova_translation_test_node')
        
        # Create publisher for WebXR control messages
        self.webxr_pub = self.create_publisher(
            WebXRControl,
            '/webxr/control',
            10
        )
        
        # Timer for publishing test messages
        self.timer = self.create_timer(0.1, self.publish_test_message)  # 10 Hz
        
        # Test sequence variables
        self.command_count = -1  # Start at -1 for initial move_disabled phase
        self.max_commands = 11  # -1: move disabled, 0-9: movement commands, 10: final move disabled
        self.command_duration = 0.1  # Hold each command for 0.2 second
        self.command_start_time = time.time()
        self.test_completed = False
        
        # Initial position
        self.initial_pos = [0.0, 0.0, 0.0]  # x, y, z in meters
        self.current_z_offset = 0.0
        
        self.get_logger().info('Kinova Translation Test Node started')
        self.get_logger().info('Test sequence: move disabled -> +1cm Z for 5 times -> -1cm Z for 5 times -> move disabled')

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
        """Publish test messages for Z translation"""
        if self.test_completed:
            return
            
        current_time = time.time()
        
        # Check if we need to execute next command
        if current_time - self.command_start_time >= self.command_duration:
            if self.command_count < self.max_commands:
                self.command_count += 1
                self.command_start_time = current_time
                
                if self.command_count == 0:
                    # Initial phase: move disabled
                    self.get_logger().info('Phase 0: Move disabled (initial)')
                elif self.command_count >= 1 and self.command_count <= 5:
                    # First 5 movement commands: move up (+1cm)
                    self.current_z_offset += 0.01  # +1cm
                    self.get_logger().info(f'Command {self.command_count}: Moving +1cm in Z (total offset: {self.current_z_offset*100:.1f}cm)')
                elif self.command_count >= 6 and self.command_count <= 10:
                    # Next 5 movement commands: move down (-1cm)
                    self.current_z_offset -= 0.01  # -1cm
                    self.get_logger().info(f'Command {self.command_count}: Moving -1cm in Z (total offset: {self.current_z_offset*100:.1f}cm)')
                elif self.command_count == 11:
                    # Final phase: move disabled
                    self.get_logger().info('Phase 11: Move disabled (final)')
            else:
                # All commands completed, mark as done
                self.get_logger().info('Translation test completed. Exiting...')
                self.test_completed = True
                self.timer.cancel()
                return
        
        # Set position with current Z offset
        position = [
            self.initial_pos[0],
            self.initial_pos[1], 
            self.initial_pos[2] + self.current_z_offset
        ]
        
        # Identity quaternion (no rotation)
        quaternion = [0.0, 0.0, 0.0, 1.0]
        
        # Control settings
        gripper_open = False
        # Move enabled only during movement commands (1-10), disabled at start (0) and end (11)
        move_enabled = self.command_count >= 1 and self.command_count <= 10
        
        # Create and publish message
        msg = self.create_webxr_message(position, quaternion, gripper_open, move_enabled)
        self.webxr_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = KinovaTranslationTestNode()
    
    try:
        while rclpy.ok() and not node.test_completed:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down translation test node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()