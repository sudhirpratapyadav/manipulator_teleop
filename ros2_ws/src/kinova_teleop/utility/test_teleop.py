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
        
        # Test sequence variables
        self.test_phase = 0
        self.phase_start_time = time.time()
        self.phase_duration = 3.0  # 3 seconds per phase
        self.current_time = 0.0
        
        # Initial position (will be used as reference)
        self.initial_pos = [0.5, 0.0, 0.5]  # x, y, z in meters
        
        self.get_logger().info('Kinova Test Node started - Publishing test messages to /webxr/control')
        self.get_logger().info('Test sequence:')
        self.get_logger().info('Phase 0: Stationary with move disabled')
        self.get_logger().info('Phase 1: Enable move, stationary position')
        self.get_logger().info('Phase 2: Rotation test - Roll (X-axis)')
        self.get_logger().info('Phase 3: Rotation test - Pitch (Y-axis)')
        self.get_logger().info('Phase 4: Rotation test - Yaw (Z-axis)')
        self.get_logger().info('Phase 5: Combined rotation test')
        self.get_logger().info('Phase 6: Move in X direction with gripper closed')
        self.get_logger().info('Phase 7: Move in Y direction with gripper open')
        self.get_logger().info('Phase 8: Move in Z direction with gripper closed')
        self.get_logger().info('Phase 9: Circular motion in XY plane with gripper open/close toggle')
        self.get_logger().info('Phase 10: Return to initial position')

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
        """Publish test messages based on current test phase"""
        current_time = time.time()
        phase_time = current_time - self.phase_start_time
        
        # Check if we need to advance to next phase
        if phase_time >= self.phase_duration:
            self.test_phase += 1
            self.phase_start_time = current_time
            phase_time = 0.0
            
            if self.test_phase > 10:
                self.test_phase = 0  # Loop back to beginning
                self.get_logger().info('Test sequence completed, restarting...')
        
        # Normalized time within current phase (0.0 to 1.0)
        t = phase_time / self.phase_duration
        
        # Default values
        position = self.initial_pos.copy()
        quaternion = [0.0, 0.0, 0.0, 1.0]  # Identity quaternion [x, y, z, w]
        gripper_open = False
        move_enabled = False
        
        if self.test_phase == 0:
            # Phase 0: Stationary with move disabled
            if phase_time < 0.1:  # Log only once at start of phase
                self.get_logger().info('Phase 0: Stationary with move disabled')
            move_enabled = False
            gripper_open = False
            
        elif self.test_phase == 1:
            # Phase 1: Enable move, stationary position
            if phase_time < 0.1:
                self.get_logger().info('Phase 1: Enable move, stationary position (calibration)')
            move_enabled = True
            gripper_open = False
            
        elif self.test_phase == 2:
            # Phase 2: Rotation test - Pitch (Y-axis)
            if phase_time < 0.1:
                self.get_logger().info('Phase 2: Rotation test - Pitch (Y-axis rotation)')
            move_enabled = True
            gripper_open = False  # Test with gripper open
            # Sinusoidal pitch rotation around Y-axis
            pitch_angle = 0.4 * math.sin(2 * math.pi * t)  # ±0.4 radians (±22.9 degrees)
            quaternion = [0.0, math.sin(pitch_angle/2), 0.0, math.cos(pitch_angle/2)]
            
        elif self.test_phase == 3:
            # Phase 3: Rotation test - Roll (X-axis)
            if phase_time < 0.1:
                self.get_logger().info('Phase 3: Rotation test - Roll (X-axis rotation)')
            move_enabled = True
            gripper_open = False
            # Sinusoidal roll rotation around X-axis
            roll_angle = 0.1 * math.sin(2 * math.pi * t)  # ±0.5 radians (±28.6 degrees)
            quaternion = [math.sin(roll_angle/2), 0.0, 0.0, math.cos(roll_angle/2)]
            
        elif self.test_phase == 4:
            # Phase 4: Rotation test - Yaw (Z-axis)
            if phase_time < 0.1:
                self.get_logger().info('Phase 4: Rotation test - Yaw (Z-axis rotation)')
            move_enabled = True
            gripper_open = False
            # Sinusoidal yaw rotation around Z-axis
            yaw_angle = 0.6 * math.sin(2 * math.pi * t)  # ±0.6 radians (±34.4 degrees)
            quaternion = [0.0, 0.0, math.sin(yaw_angle/2), math.cos(yaw_angle/2)]
            
        elif self.test_phase == 5:
            # Phase 5: Combined rotation test
            if phase_time < 0.1:
                self.get_logger().info('Phase 5: Combined rotation test - All axes')
            move_enabled = True
            # Toggle gripper every second
            gripper_open = (int(phase_time) % 2) == 0
            # Combined rotations with different frequencies
            roll_angle = 0.3 * math.sin(2 * math.pi * t)      # Roll at base frequency
            pitch_angle = 0.2 * math.sin(4 * math.pi * t)     # Pitch at 2x frequency
            yaw_angle = 0.4 * math.sin(1 * math.pi * t)       # Yaw at 0.5x frequency
            
            # Convert Euler angles to quaternion (ZYX order)
            cy = math.cos(yaw_angle * 0.5)
            sy = math.sin(yaw_angle * 0.5)
            cp = math.cos(pitch_angle * 0.5)
            sp = math.sin(pitch_angle * 0.5)
            cr = math.cos(roll_angle * 0.5)
            sr = math.sin(roll_angle * 0.5)
            
            quaternion = [
                sr * cp * cy - cr * sp * sy,  # x
                cr * sp * cy + sr * cp * sy,  # y
                cr * cp * sy - sr * sp * cy,  # z
                cr * cp * cy + sr * sp * sy   # w
            ]
        
        elif self.test_phase == 6:
            # Phase 6: Move in X direction with gripper closed
            if phase_time < 0.1:
                self.get_logger().info('Phase 6: Moving in X direction with gripper closed')
            move_enabled = True
            gripper_open = False
            # Sinusoidal motion in X
            position[0] = self.initial_pos[0] + 0.2 * math.sin(2 * math.pi * t)
            
        elif self.test_phase == 7:
            # Phase 7: Move in Y direction with gripper open
            if phase_time < 0.1:
                self.get_logger().info('Phase 7: Moving in Y direction with gripper open')
            move_enabled = True
            gripper_open = True
            # Sinusoidal motion in Y
            position[1] = self.initial_pos[1] + 0.15 * math.sin(2 * math.pi * t)
            
        elif self.test_phase == 8:
            # Phase 8: Move in Z direction with gripper closed
            if phase_time < 0.1:
                self.get_logger().info('Phase 8: Moving in Z direction with gripper closed')
            move_enabled = True
            gripper_open = False
            # Sinusoidal motion in Z
            position[2] = self.initial_pos[2] + 0.1 * math.sin(2 * math.pi * t)
            
        elif self.test_phase == 9:
            # Phase 9: Circular motion in XY plane with gripper toggle
            if phase_time < 0.1:
                self.get_logger().info('Phase 9: Circular motion with gripper toggle')
            move_enabled = True
            # Toggle gripper every half second
            gripper_open = (int(phase_time * 2) % 2) == 0
            # Circular motion
            radius = 0.1
            angle = 2 * math.pi * t
            position[0] = self.initial_pos[0] + radius * math.cos(angle)
            position[1] = self.initial_pos[1] + radius * math.sin(angle)
            # Add some Z rotation to the quaternion
            quat_angle = angle * 0.5  # Rotate slower than the motion
            quaternion = [0.0, 0.0, math.sin(quat_angle/2), math.cos(quat_angle/2)]
            
        elif self.test_phase == 10:
            # Phase 10: Return to initial position
            if phase_time < 0.1:
                self.get_logger().info('Phase 10: Returning to initial position')
            move_enabled = True
            gripper_open = False
            # Smooth transition back to initial position
            # Use easing function for smooth motion
            ease_t = 1 - (1 - t) ** 3  # Ease-out cubic
            # Start from last position of phase 9 and move to initial
            radius = 0.1
            start_pos = [
                self.initial_pos[0] + radius * math.cos(2 * math.pi),
                self.initial_pos[1] + radius * math.sin(2 * math.pi),
                self.initial_pos[2]
            ]
            for i in range(3):
                position[i] = start_pos[i] + (self.initial_pos[i] - start_pos[i]) * ease_t
                
        # Create and publish message
        msg = self.create_webxr_message(position, quaternion, gripper_open, move_enabled)
        self.webxr_pub.publish(msg)
        
        # Log detailed info every second
        if int(phase_time * 10) % 10 == 0:
            self.get_logger().debug(
                f'Phase {self.test_phase}, t={phase_time:.1f}s: '
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