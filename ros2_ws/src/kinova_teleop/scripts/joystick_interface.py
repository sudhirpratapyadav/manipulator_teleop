#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math
from datetime import datetime

class JoystickInterface(Node):
    def __init__(self):
        super().__init__('joystick_interface')
        self.delta_pose = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        }
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.delta_ee_pub = self.create_publisher(
            Twist,
            'delta_ee',
            10
        )
        self.get_logger().info('JoystickInterface node started.')

    def joy_callback(self, msg):
        # Reset delta values to 0
        self.delta_pose = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        }
        # Position mapping (meters)
        if abs(msg.axes[0]) > 0.1:  # Left stick X
            self.delta_pose['position']['y'] = msg.axes[0] * 0.05
        if abs(msg.axes[1]) > 0.1:  # Left stick Y
            self.delta_pose['position']['x'] = msg.axes[1] * 0.05
        if abs(msg.axes[4]) > 0.1:  # Right stick Y
            self.delta_pose['position']['z'] = msg.axes[4] * 0.05
        # Orientation mapping (radians)
        if abs(msg.axes[3]) > 0.1:  # Right stick X
            self.delta_pose['orientation']['yaw'] = msg.axes[3] * 0.1
        if len(msg.buttons) > 2 and msg.buttons[2] == 1:  # X button
            self.delta_pose['orientation']['pitch'] = 0.1
        if len(msg.buttons) > 3 and msg.buttons[3] == 1:  # Y button
            self.delta_pose['orientation']['pitch'] = -0.1
        if len(msg.buttons) > 0 and msg.buttons[0] == 1:  # A button
            self.delta_pose['orientation']['roll'] = 0.1
        if len(msg.buttons) > 1 and msg.buttons[1] == 1:  # B button
            self.delta_pose['orientation']['roll'] = -0.1
        # Publish delta EE as Twist
        twist_msg = Twist()
        twist_msg.linear.x = self.delta_pose['position']['x']
        twist_msg.linear.y = self.delta_pose['position']['y']
        twist_msg.linear.z = self.delta_pose['position']['z']
        twist_msg.angular.x = self.delta_pose['orientation']['roll']
        twist_msg.angular.y = self.delta_pose['orientation']['pitch']
        twist_msg.angular.z = self.delta_pose['orientation']['yaw']
        self.delta_ee_pub.publish(twist_msg)
        # Only print if there's actual movement
        if (abs(self.delta_pose['position']['x']) > 0.001 or 
            abs(self.delta_pose['position']['y']) > 0.001 or 
            abs(self.delta_pose['position']['z']) > 0.001 or
            abs(self.delta_pose['orientation']['roll']) > 0.001 or
            abs(self.delta_pose['orientation']['pitch']) > 0.001 or
            abs(self.delta_pose['orientation']['yaw']) > 0.001):
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            output = [
                f'\n{"="*50}',
                f'Time: {timestamp}',
                f'{"-"*50}',
                'DELTA EE COMMAND:'
            ]
            output.append(f'  X: {self.delta_pose["position"]["x"]:+.3f} m')
            output.append(f'  Y: {self.delta_pose["position"]["y"]:+.3f} m')
            output.append(f'  Z: {self.delta_pose["position"]["z"]:+.3f} m')
            output.append(f'{"-"*50}')
            output.append('DELTA ORIENTATION:')
            output.append(f'  Roll: {self.delta_pose["orientation"]["roll"]:+.3f} rad ({math.degrees(self.delta_pose["orientation"]["roll"]):+.1f}°)')
            output.append(f'  Pitch: {self.delta_pose["orientation"]["pitch"]:+.3f} rad ({math.degrees(self.delta_pose["orientation"]["pitch"]):+.1f}°)')
            output.append(f'  Yaw: {self.delta_pose["orientation"]["yaw"]:+.3f} rad ({math.degrees(self.delta_pose["orientation"]["yaw"]):+.1f}°)')
            output.append(f'{"="*50}')
            self.get_logger().info('\n'.join(output))

def main(args=None):
    rclpy.init(args=args)
    node = JoystickInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
