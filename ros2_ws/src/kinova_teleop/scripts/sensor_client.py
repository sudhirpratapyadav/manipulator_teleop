#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import websocket
import json
import threading
import numpy as np
import time


class AndroidRotationVectorNode(Node):
    def __init__(self):
        super().__init__('android_rotation_vector_node')
        
        # Declare parameters with descriptions
        self.declare_parameter(
            'websocket_url', 
            'ws://172.31.17.36:8080/sensor/connect?type=android.sensor.rotation_vector',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='WebSocket URL for connecting to Android sensor'
            )
        )
        self.declare_parameter(
            'publish_rate', 
            30.0,  # 30 Hz default
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Rate limit for publishing quaternion messages (Hz)'
            )
        )
        self.declare_parameter(
            'reconnect_attempts', 
            5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Number of reconnection attempts on WebSocket failure'
            )
        )
        
        # Get parameters
        self.url = self.get_parameter('websocket_url').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.reconnect_attempts = self.get_parameter('reconnect_attempts').get_parameter_value().integer_value
        
        # Create publisher
        self.publisher_ = self.create_publisher(Quaternion, '/teleop/quat_abs', 10)
        
        # Initialization
        self.current_quat = None
        self.connect_retry_count = 0
        self.ws = None
        self.running = True
        
        # Start WebSocket in a separate thread
        self.ws_thread = threading.Thread(target=self.websocket_manager)
        self.ws_thread.daemon = True
        self.ws_thread.start()
        
        # Start publisher timer
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_quaternion)
        
        self.get_logger().info(f"AndroidRotationVectorNode initialized with WebSocket URL: {self.url}")
        self.get_logger().info(f"Publishing rate: {self.publish_rate} Hz")
        self.get_logger().info(f"Publishing direct rotation vector without relative calculations")

    def websocket_manager(self):
        """Manages WebSocket connection with retry logic."""
        while self.running and self.connect_retry_count < self.reconnect_attempts:
            try:
                self.start_websocket()
                # If we get here, connection closed normally
                if self.running:
                    self.get_logger().warn("WebSocket connection closed, attempting to reconnect...")
                    self.connect_retry_count += 1
                    time.sleep(2.0)  # Wait before reconnecting
            except Exception as e:
                self.get_logger().error(f"WebSocket manager error: {e}")
                self.connect_retry_count += 1
                time.sleep(2.0)  # Wait before reconnecting
                
        if self.connect_retry_count >= self.reconnect_attempts:
            self.get_logger().error(f"Failed to connect after {self.reconnect_attempts} attempts")

    def start_websocket(self):
        """Establish WebSocket connection and set up callbacks."""
        def on_message(ws, message):
            try:
                data = json.loads(message)
                values = data.get('values', [])
                if len(values) < 3:
                    return

                # Store quaternion directly without relative calculations
                self.current_quat = [
                    float(values[0]),
                    float(values[1]),
                    float(values[2]),
                    float(values[3]) if len(values) > 3 else 0.0
                ]

            except json.JSONDecodeError:
                self.get_logger().warn("Received invalid JSON data")
            except Exception as e:
                self.get_logger().error(f"Error processing message: {e}")

        def on_error(ws, error):
            self.get_logger().error(f"WebSocket error: {error}")

        def on_close(ws, close_status_code, close_msg):
            close_info = f"code: {close_status_code}, message: {close_msg}" if close_status_code else "unknown reason"
            self.get_logger().info(f"WebSocket closed: {close_info}")

        def on_open(ws):
            self.get_logger().info("WebSocket connected successfully")
            # Reset retry counter on successful connection
            self.connect_retry_count = 0

        # Create and connect WebSocket
        self.ws = websocket.WebSocketApp(
            self.url,
            on_open=on_open,
            on_message=on_message,
            on_error=on_error,
            on_close=on_close,
        )

        # Run the WebSocket connection
        self.ws.run_forever()

    def publish_quaternion(self):
        """Publishes quaternion at controlled rate."""
        if self.current_quat is None:
            return
            
        quat_msg = Quaternion()
        quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w = self.current_quat

        # Print the quaternion values before publishing (only if values changed significantly)
        if hasattr(self, 'last_published_quat'):
            diff = np.sum(np.abs(np.array(self.current_quat) - np.array(self.last_published_quat)))
            if diff > 0.01:  # Only print if significant change
                print(f"Publishing direct quaternion: x={quat_msg.x:.4f}, y={quat_msg.y:.4f}, z={quat_msg.z:.4f}, w={quat_msg.w:.4f}")
        else:
            print(f"Publishing direct quaternion: x={quat_msg.x:.4f}, y={quat_msg.y:.4f}, z={quat_msg.z:.4f}, w={quat_msg.w:.4f}")
            
        self.last_published_quat = self.current_quat
        self.publisher_.publish(quat_msg)
        self.get_logger().debug(f"Published direct quaternion: {quat_msg}")

    def destroy_node(self):
        """Clean shutdown of node resources."""
        self.running = False
        if self.ws:
            self.ws.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AndroidRotationVectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down AndroidRotationVectorNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
