# WebXR Control ROS Message

This message extends the standard PoseStamped message with additional control signals for teleoperation.

## Message Structure

```
std_msgs/Header header       # Standard ROS header with timestamp
geometry_msgs/Pose pose      # 3D pose (position and orientation)
bool gripper_open            # Gripper state (true = open, false = closed)
bool move_enabled            # Movement control (true = enabled, false = disabled)
```

## Publishing Topic

- `/webxr/control` - Enhanced WebXRControl message with pose and control states

## Usage in Code

### Python
```python
from kinova_teleop.msg import WebXRControl

# Create message
msg = WebXRControl()
msg.header.stamp = node.get_clock().now().to_msg()
msg.header.frame_id = "webxr_frame"
msg.pose.position.x = 0.1
msg.pose.position.y = 0.2
msg.pose.position.z = 0.3
msg.pose.orientation.w = 1.0
msg.gripper_open = True
msg.move_enabled = False

# Publish
publisher.publish(msg)
```

### Subscribing (Python)
```python
from kinova_teleop.msg import WebXRControl

def callback(msg):
    position = msg.pose.position
    orientation = msg.pose.orientation
    gripper_open = msg.gripper_open
    move_enabled = msg.move_enabled
    # Process data...

subscription = node.create_subscription(
    WebXRControl,
    '/webxr/control',
    callback,
    10
)
```

subscription = node.create_subscription(
    WebXRControl,
    '/webxr/control',
    callback,
    10
)
```
