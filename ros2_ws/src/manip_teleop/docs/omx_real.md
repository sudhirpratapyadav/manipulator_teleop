# OmxRealWebXRNode - Architecture Documentation

## Overview

The implementation in `omx_real.py` provides robust manipulation control with WebXR integration for the OMX robot using a **single-threaded main loop** with **event-driven state machine** architecture.

## Core Design Principles

1. **Single-Threaded Main Loop**: All MuJoCo operations and critical logic run in one thread
2. **Event-Driven State Machine**: Clear states with well-defined transitions  
3. **Separation of Concerns**: ROS callbacks only update data, main loop handles logic
4. **Async Action Management**: Non-blocking robot control with proper timeout handling
5. **Robust Error Handling**: Graceful recovery from failures at any stage

## Architecture Overview

```
                    MAIN THREAD (200Hz Loop)                    
                                                                 
     State Machine     MuJoCo Viewer      Action Manager  
     Controller         & Physics          (Async)        
                                                          
            ↓                    ↓                    ↓
                                                       
  ROS Callbacks      Data Storage       Robot Actions  
  (Data Only)        Thread-Safe        Non-Blocking   
```

## Core Components

### 1. SystemState Enum (lines 72-83)

Defines the complete state machine with clear transitions:

```python
class SystemState(Enum):
    STARTING = "starting"                    # Initial state
    WAITING_FOR_JOINTS = "waiting_for_joints"  # Wait for robot joint data
    SYNCING_MUJOCO = "syncing_mujoco"         # Sync simulation with robot
    GOING_TO_HOME = "going_to_home"           # Move robot to home position
    TESTING_GRIPPER_OPEN = "testing_gripper_open"   # Test gripper opening
    TESTING_GRIPPER_CLOSE = "testing_gripper_close" # Test gripper closing
    READY_FOR_WEBXR = "ready_for_webxr"       # Ready for WebXR control
    WEBXR_ACTIVE = "webxr_active"             # Active WebXR teleoperation
    ERROR_RECOVERY = "error_recovery"         # Recovering from errors
    FAILED = "failed"                         # Unrecoverable failure
```

### 2. RobotData Class (lines 85-114)

Thread-safe data container updated only by ROS callbacks:

```python
@dataclass
class RobotData:
    current_joints: Optional[List[float]] = None      # Current robot joint positions
    webxr_pos: Optional[np.ndarray] = None           # WebXR controller position
    webxr_quat: Optional[np.ndarray] = None          # WebXR controller orientation
    gripper_open: bool = False                       # Gripper state from WebXR
    move_enabled: bool = False                       # Movement enable from WebXR
    prev_move_enabled: bool = False                  # Previous movement state
    prev_gripper_open: bool = False                  # Previous gripper state
    last_joint_update: float = 0.0                   # Last joint data timestamp
    last_webxr_update: float = 0.0                   # Last WebXR data timestamp
```

### 3. AsyncActionManager Class (lines 155-327)

Manages robot actions without blocking the main thread:

**Key Features:**
- Non-blocking action client communication
- Timeout handling for all actions
- Automatic cleanup of completed actions
- Separate handling for arm and gripper actions

**Critical Methods:**
- `send_joint_goal_async()` (lines 174-199): Send arm trajectory goals
- `send_gripper_goal_async()` (lines 201-217): Send gripper commands  
- `check_action_status()` (lines 218-229): Check action completion status

### 4. MuJoCoSystem Class (lines 328-413)

Handles all MuJoCo physics simulation operations:

**Key Features:**
- Manages MuJoCo model and data
- Integrates with Mink IK solver
- Synchronizes simulation with real robot state
- Updates visualization based on WebXR input

**Critical Methods:**
- `sync_with_robot()` (lines 348-365): Sync simulation with real robot joints
- `update_from_webxr()` (lines 366-370): Update targets from WebXR input
- `get_ik_solution()` (lines 371-382): Compute inverse kinematics solution

### 5. WebXRSystem Class (lines 414-458)

Handles WebXR processing and coordinate system calibration:

**Key Features:**
- Calibrates WebXR coordinate system to robot workspace
- Processes WebXR input and converts to robot targets
- Handles coordinate transformations and offsets

**Critical Methods:**
- `calibrate()` (lines 424-437): Calibrate WebXR to robot coordinates
- `process_webxr_input()` (lines 438-450): Convert WebXR input to robot targets

### 6. OmxRealController Class (lines 488-755)

Main controller implementing the single-threaded architecture:

**Key Features:**
- Inherits from ROS Node for ROS integration
- Runs 200Hz main loop handling all operations
- Manages state machine transitions
- Coordinates all subsystems

## Main Loop Architecture (lines 559-593)

The heart of the system is the single-threaded main loop:

```python
def run_main_loop(self):
    while rclpy.ok() and self.viewer.is_running():
        # 1. Process ROS messages (non-blocking)
        rclpy.spin_once(self, timeout_sec=0)
        
        # 2. Update state machine
        self._update_state_machine()
        
        # 3. Update MuJoCo based on current state
        self._update_mujoco()
        
        # 4. Process WebXR if ready
        if self.state == SystemState.WEBXR_ACTIVE:
            self._process_webxr()
        
        # 5. Render and sync
        self.viewer.sync()
        rate.sleep()
```

## State Machine Flow

### Initialization Sequence

1. **STARTING** → **WAITING_FOR_JOINTS**
   - Wait for action servers to become available
   - Transition when servers are ready

2. **WAITING_FOR_JOINTS** → **SYNCING_MUJOCO**  
   - Wait for joint state data from real robot
   - Transition when valid joint data received

3. **SYNCING_MUJOCO** → **GOING_TO_HOME**
   - Synchronize MuJoCo simulation with robot state
   - Start motion to home position

4. **GOING_TO_HOME** → **TESTING_GRIPPER_OPEN**
   - Wait for home motion to complete
   - Transition when action succeeds

5. **TESTING_GRIPPER_OPEN** → **TESTING_GRIPPER_CLOSE**
   - Test gripper opening functionality
   - Transition when gripper opens successfully

6. **TESTING_GRIPPER_CLOSE** → **READY_FOR_WEBXR**
   - Test gripper closing functionality  
   - Transition when gripper closes successfully

### Operational States

7. **READY_FOR_WEBXR** ↔ **WEBXR_ACTIVE**
   - **READY_FOR_WEBXR** → **WEBXR_ACTIVE**: When WebXR data available and move enabled
   - **WEBXR_ACTIVE** → **READY_FOR_WEBXR**: When move disabled

### Error Handling

8. **Any State** → **ERROR_RECOVERY** → **WAITING_FOR_JOINTS**
   - Automatic recovery from action failures
   - 5-second delay before retry attempt

9. **Any State** → **FAILED**
   - Unrecoverable failures after max retries

## ROS Integration

### Subscriptions (lines 521-533)

- **Joint States** (`/joint_states`): Robot joint position feedback
- **WebXR Control** (`/webxr/control`): WebXR controller input

### Action Clients (lines 165-167)

- **Arm Controller** (`/arm_controller/follow_joint_trajectory`): Joint trajectory control
- **Gripper Controller** (`/gripper_controller/gripper_cmd`): Gripper position control

### Callback Architecture (lines 535-558)

**Critical Design**: Callbacks are **data-only** - they update `RobotData` but perform no logic:

```python
def _joint_state_callback(self, joint_state: JointState):
    """Handle incoming joint state data - DATA ONLY, NO LOGIC"""
    # Process and store joint data only
    self.robot_data.update_joints(joint_pos_ordered)

def _webxr_control_callback(self, msg: WebXRControl):
    """Handle incoming WebXR control data - DATA ONLY, NO LOGIC"""  
    # Store WebXR data only
    self.robot_data.update_webxr(pos, quat, msg.gripper_open, msg.move_enabled)
```

## WebXR Integration

### Calibration Process (lines 696-707)

When entering WebXR mode, the system calibrates coordinate systems:

1. Get current end-effector pose from MuJoCo
2. Get current WebXR controller pose
3. Calculate position offset and orientation difference
4. Use these offsets to transform future WebXR input

### Real-Time Control (lines 713-755)

During active WebXR control:

1. **Process WebXR Input**: Convert WebXR poses to robot targets using calibration
2. **Update MuJoCo Targets**: Set target position/orientation in simulation
3. **Solve IK**: Compute joint angles to reach targets
4. **Update Visualization**: Show target configuration in MuJoCo viewer
5. **Send Robot Commands**: Send joint goals to real robot at 4Hz

## Configuration System (lines 115-133)

Centralized configuration management:

```python
@dataclass
class SystemConfig:
    loop_frequency: float = 200.0           # Main loop frequency
    action_timeout: float = 10.0            # Action timeout in seconds
    ik_solver: str = "daqp"                # IK solver type
    home_pose: List[float] = [0.0, -60.0, 21.0, 40.3]  # Home joint angles (degrees)
    gripper_open_pos: float = -0.02        # Gripper open position
    gripper_closed_pos: float = -0.01      # Gripper closed position
```

## Error Handling System (lines 459-487)

**ErrorHandler Class** provides centralized error management:

- **Initialization Errors**: Retry mechanism with maximum attempts
- **Runtime Errors**: Automatic recovery procedures  
- **Timeout Handling**: Action timeout detection and cleanup
- **State Recovery**: Return to safe operational state

## Key Architectural Benefits

### 1. **Zero Deadlock Risk**
- No blocking calls in ROS callbacks
- All action clients use async communication
- Main loop never blocks on external operations

### 2. **Clear State Management** 
- Well-defined state machine with explicit transitions
- Easy to understand system behavior
- Predictable error recovery paths

### 3. **Modular Design**
- Separate concerns: data, control, physics, WebXR
- Each component has single responsibility
- Easy to test and maintain individual components

### 4. **Robust Operation**
- Handles all edge cases and error conditions
- Graceful recovery from failures
- Continuous operation even with intermittent issues

### 5. **Real-Time Performance**
- 200Hz main loop for responsive control
- Optimized data flow and processing
- Minimal latency between WebXR input and robot motion

## File Structure Summary

```
omx_real.py (769 lines)
├── Utility Functions (lines 43-71)
├── SystemState Enum (lines 72-83)  
├── RobotData Class (lines 85-114)
├── SystemConfig Class (lines 115-133)
├── SystemLogger Class (lines 134-154)
├── AsyncActionManager Class (lines 155-327)
├── MuJoCoSystem Class (lines 328-413)
├── WebXRSystem Class (lines 414-458)  
├── ErrorHandler Class (lines 459-487)
├── OmxRealController Class (lines 488-755)
└── Main Entry Point (lines 756-769)
```

This architecture ensures reliable, responsive manipulation control with WebXR integration while maintaining thread safety and providing robust error recovery capabilities.