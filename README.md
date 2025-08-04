# Manipulator Teleop

A ROS2 package for robotic manipulator teleoperation using WebXR and MuJoCo simulation.

## Overview

This project provides teleoperation capabilities for various robotic manipulators including:
- Franka Emika Panda
- Kinova Gen3
- Open Manipulator X (OMX)

The system uses WebXR for immersive control and MuJoCo for physics simulation, with inverse kinematics powered by the Mink library.

## Package Structure

```
manipulator_teleop/
├── ros2_ws/
│   └── src/
│       └── manip_teleop/           # Main ROS2 package
│           ├── scripts/            # Teleoperation nodes
│           ├── models/             # Robot models (MuJoCo XML files)
│           ├── launch/             # Launch files
│           ├── msg/                # Custom message definitions
│           └── utility/            # Test and utility scripts
├── mink/                           # Inverse kinematics library (submodule)
├── webxr_api_server/               # WebXR API server (submodule)
└── README.md
```

## Features

- **Multi-robot Support**: Supports Franka Panda, Kinova Gen3, and OMX manipulators
- **WebXR Integration**: Immersive VR/AR control interface
- **Physics Simulation**: MuJoCo-based realistic physics
- **Inverse Kinematics**: Real-time IK solving with Mink
- **Rate Limiting**: Configurable control loop rates
- **Gripper Control**: Integrated gripper teleoperation

## Requirements

- ROS2 (Humble/Iron/Rolling)
- Python 3.10+
- MuJoCo
- NumPy
- WebXR-capable browser (Chrome/Firefox with WebXR support)

## Installation

### Option 1: Dev Container (Recommended)

This option provides a pre-configured development environment with all dependencies.

#### Prerequisites
- Docker installed
- VS Code with Dev Containers extension

#### Steps

1. **Clone the repository:**
   ```bash
   git clone --recurse-submodules https://github.com/sudhirpratapyadav/manipulator_teleop.git
   cd manipulator_teleop
   ```

2. **Open in VS Code:**
   ```bash
   code .
   ```

3. **Reopen in Container:**
   - Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
   - Type "Dev Containers: Reopen in Container"
   - Select the option and wait for the container to build

4. **The container will automatically:**
   - Install ROS2 Humble
   - Set up Python environment
   - Install all dependencies (including MuJoCo via Mink)
   - Build the ROS2 workspace
   - Configure WebXR certificates

### Option 2: Manual Installation

#### Prerequisites
- Ubuntu 22.04 or later
- ROS2 installed ([installation guide](https://docs.ros.org/en/humble/Installation.html))
- Python 3.10+ with pip

#### Steps

1. **Clone the repository with submodules:**
   ```bash
   git clone --recurse-submodules https://github.com/sudhirpratapyadav/manipulator_teleop.git
   cd manipulator_teleop
   ```

2. **Install system dependencies:**
   ```bash
   sudo apt update
   sudo apt install -y python3-pip python3-venv build-essential
   ```

3. **Create and activate Python virtual environment:**
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

4. **Install Mink dependencies (includes MuJoCo):**
   ```bash
   cd mink
   pip install -e .
   cd ..
   ```

5. **Install WebXR API server dependencies:**
   ```bash
   pip install -r webxr_api_server/requirements.txt
   ```

6. **Build the ROS2 workspace:**
   ```bash
   cd ros2_ws
   colcon build
   source install/setup.bash
   cd ..
   ```

7. **Setup HTTPS certificates for WebXR:**
   ```bash
   cd webxr_api_server
   chmod +x setup_https.sh
   ./setup_https.sh
   cd ..
   ```

8. **Environment Setup:**
   For each new terminal session, you'll need to source the environment:
   ```bash
   source venv/bin/activate
   source /opt/ros/humble/setup.bash  # Adjust for your ROS2 distribution
   source ros2_ws/install/setup.bash
   ```

## Usage

### Using the WebXR app

1. **Start the WebXR API server:**
   ```bash
   cd webxr_api_server
   python run_server.py
   ```
   The server will automatically detect your IP address and start on `https://<your-ip>:8443`. See the [WebXR API Server README](webxr_api_server/README.md) for detailed information about accessing the web interfaces.

2. **In a new terminal, launch the teleoperation simulation:**
   ```bash
   # Source the environment (if manual installation)
   source venv/bin/activate
   source /opt/ros/humble/setup.bash
   source ros2_ws/install/setup.bash
   
   # Launch the simulation
   ros2 launch manip_teleop teleop_sim.launch.py
   ```

3. **Open your broswer in mobile:**
   - Navigate to `https://<your-ip>:8443/static/index.html` (IP address will be displayed in the server output)
   - Accept the self-signed certificate warning
   - Start the app

### Available Teleoperation Scripts

Run individual scripts for specific robots:

#### Franka Panda
```bash
ros2 run manip_teleop franka_sim.py
```

#### Kinova Gen3 with Gripper
```bash
ros2 run manip_teleop kinova_sim.py
```

#### Open Manipulator X
```bash
ros2 run manip_teleop omx_sim.py
```

### Testing and Utilities

- **Test MuJoCo model loading:**
  ```bash
  cd ros2_ws/src/manip_teleop/utility
  python3 test_load_model_mujoco.py
  ```

- **Test teleoperation without WebXR:**
  ```bash
  cd ros2_ws/src/manip_teleop/utility
  python3 test_teleop.py
  ```

- **Test rotation control:**
  ```bash
  cd ros2_ws/src/manip_teleop/utility
  python3 test_teleop_rotation.py
  ```

### WebXR Interface

The WebXR interface provides:
- **Hand tracking**: Use your hands to control the robot
- **Controller support**: Compatible with VR controllers
- **Desktop mode**: Mouse and keyboard control for testing
- **Real-time visualization**: See the robot's response in the MuJoCo viewer

### Troubleshooting

1. **WebXR not working**: Ensure you're using HTTPS and have a compatible browser
2. **Certificate errors**: Run `./setup_https.sh` in the webxr_api_server directory
3. **ROS2 nodes not found**: Source the workspace: `source ros2_ws/install/setup.bash`
4. **MuJoCo viewer issues**: Ensure you have a display and OpenGL support

## Documentation

For detailed information about the ROS2 package configuration, custom messages, and development details, see the [Manip Teleop Package README](ros2_ws/src/manip_teleop/README.md).


## Claude Code installation
```
sudo apt-get update
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt-get install -y nodejs
npm install -g @anthropic-ai/claude-code --prefix ~/.npm-global
echo 'export PATH="$HOME/.npm-global/bin:$PATH"' >> ~/.bashrc
```