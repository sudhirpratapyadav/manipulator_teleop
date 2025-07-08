# Manip Teleop ROS2 Package

A ROS2 package for robotic manipulator teleoperation with WebXR integration and MuJoCo simulation.

## Overview

This package provides teleoperation capabilities for various robotic manipulators using inverse kinematics, WebXR interfaces, and physics simulation. It supports multiple robot types and provides real-time control through immersive interfaces.

## Package Contents

### Scripts
- `franka_sim.py`: Franka Panda teleoperation with WebXR
- `kinova_sim.py`: Kinova Gen3 with Robotiq 2F85 gripper control
- `omx_sim.py`: Open Manipulator X teleoperation
- `loop_rate_limiters.py`: Rate limiting utilities

### Models
- `franka_emika_panda/`: Franka Panda robot models and scenes
- `kinova_gen3/`: Kinova Gen3 robot models
- `omx/`: Open Manipulator X models
- `omx_urdf/`: Open Manipulator X URDF models
- `robotiq_2f85/`: Robotiq 2F-85 gripper models

### Launch Files
- `teleop_sim.launch.py`: Main launch file for teleoperation simulation

### Custom Messages
- `WebXRControl.msg`: WebXR control input data structure

### Utilities
- `test_load_model_mujoco.py`: Test MuJoCo model loading
- `test_teleop.py`: Test teleoperation without WebXR
- `test_teleop_rotation.py`: Test rotation control

## Configuration

The system uses configurable parameters that can be modified in the respective script files:

### IK Solver Settings
- **Solver Type**: `SOLVER = "daqp"` (default)
- **Position Threshold**: `POS_THRESHOLD = 1e-4`
- **Orientation Threshold**: `ORI_THRESHOLD = 1e-4`
- **Maximum Iterations**: `MAX_ITERS = 20`

### Control Parameters
- **Action Scaling**: `ACTION_SCALE = 0.01`
- **Rate Limiting**: Configurable through `loop_rate_limiters.py`

### Robot Models and Scenes
Robot models are located in the `models/` directory and can be selected by modifying the `_XML` path in the respective scripts:

```python
_XML = _HERE / "franka_emika_panda" / "mjx_scene.xml"  # Franka Panda
_ARM_XML = _HERE / "kinova_gen3" / "scene.xml"         # Kinova Gen3 arm
_HAND_XML = _HERE / "robotiq_2f85" / "2f85.xml"       # Robotiq gripper
_XML = _HERE / "omx" / "scene.xml"                     # Open Manipulator X
```

## Custom Messages

### WebXRControl Message

The `WebXRControl` message provides a standardized interface for WebXR input data:

```
std_msgs/Header header
geometry_msgs/Pose pose
bool gripper_open
bool move_enabled
```

**Fields:**
- `header`: Standard ROS2 header with timestamp and frame information
- `pose`: 6DOF pose data (position and orientation) from WebXR input
- `gripper_open`: Boolean flag for gripper state control
- `move_enabled`: Boolean flag to enable/disable robot movement

**Usage Example:**
```python
from manip_teleop.msg import WebXRControl

# Subscribe to WebXR control data
self.subscription = self.create_subscription(
    WebXRControl,
    '/webxr/control',
    self.webxr_callback,
    10
)
```

## Dependencies

- **ROS2 Packages**: `rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`
- **Python Libraries**: `mujoco`, `numpy`, `mink`
- **Build Tools**: `ament_cmake`, `rosidl_default_generators`

## Building the Package

```bash
cd ros2_ws
colcon build --packages-select manip_teleop
source install/setup.bash
```

## Running Individual Components

### Test MuJoCo Model Loading
```bash
cd utility
python3 test_load_model_mujoco.py
```

### Test Teleoperation
```bash
cd utility
python3 test_teleop.py
```

### Run Specific Robot Teleoperation
```bash
# Franka Panda
ros2 run manip_teleop franka_sim.py

# Kinova with Gripper
ros2 run manip_teleop kinova_sim.py

# Open Manipulator X
ros2 run manip_teleop omx_sim.py
```

## Troubleshooting

### Common Issues

1. **MuJoCo Model Loading Errors**
   - Ensure model files exist in the `models/` directory
   - Check XML file paths in the scripts

2. **IK Solver Issues**
   - Verify Mink installation: `pip install -e mink/`
   - Check solver availability: `pip install daqp`

3. **ROS2 Message Errors**
   - Rebuild the package: `colcon build --packages-select manip_teleop`
   - Source the workspace: `source install/setup.bash`

4. **WebXR Connection Issues**
   - Ensure WebXR API server is running
   - Check network connectivity between devices
   - Verify HTTPS certificates are properly set up

## Future Enhancements

- Support for additional robot models
- Advanced IK solver configurations
- Enhanced rate limiting options
- Multi-robot teleoperation support