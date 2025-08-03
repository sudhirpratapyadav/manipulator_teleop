# test_omx

## ROS Topics for OpenManipulator

### Published Topics

#### `/arm_controller/controller_state`
- **Type**: `control_msgs/msg/JointTrajectoryControllerState`
- **Interface**:
```
# This message presents current controller state of JTC

# Header timestamp should be update time of controller state
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

string[] joint_names
# The set point, that is, desired state.
trajectory_msgs/JointTrajectoryPoint reference
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# Current value of the process (ie: latest sensor measurement on the controlled value).
trajectory_msgs/JointTrajectoryPoint feedback
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# The error of the controlled value, essentially reference - feedback (for a regular PID implementation).
trajectory_msgs/JointTrajectoryPoint error
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
# Current output of the controller.
trajectory_msgs/JointTrajectoryPoint output
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
```

#### `/arm_controller/state`
- **Type**: `control_msgs/msg/JointTrajectoryControllerState`
- **Interface**: Same as `/arm_controller/controller_state`

#### `/arm_controller/transition_event`
- **Type**: `lifecycle_msgs/msg/TransitionEvent`
- **Interface**:
```
# The time point at which this event occurred.
uint64 timestamp

# The id and label of this transition event.
Transition transition
	uint8 TRANSITION_CREATE = 0
	uint8 TRANSITION_CONFIGURE = 1
	uint8 TRANSITION_CLEANUP = 2
	uint8 TRANSITION_ACTIVATE = 3
	uint8 TRANSITION_DEACTIVATE = 4
	uint8 TRANSITION_UNCONFIGURED_SHUTDOWN  = 5
	uint8 TRANSITION_INACTIVE_SHUTDOWN = 6
	uint8 TRANSITION_ACTIVE_SHUTDOWN = 7
	uint8 TRANSITION_DESTROY = 8
	uint8 TRANSITION_ON_CONFIGURE_SUCCESS = 10
	uint8 TRANSITION_ON_CONFIGURE_FAILURE = 11
	uint8 TRANSITION_ON_CONFIGURE_ERROR = 12
	uint8 TRANSITION_ON_CLEANUP_SUCCESS = 20
	uint8 TRANSITION_ON_CLEANUP_FAILURE = 21
	uint8 TRANSITION_ON_CLEANUP_ERROR = 22
	uint8 TRANSITION_ON_ACTIVATE_SUCCESS = 30
	uint8 TRANSITION_ON_ACTIVATE_FAILURE = 31
	uint8 TRANSITION_ON_ACTIVATE_ERROR = 32
	uint8 TRANSITION_ON_DEACTIVATE_SUCCESS = 40
	uint8 TRANSITION_ON_DEACTIVATE_FAILURE = 41
	uint8 TRANSITION_ON_DEACTIVATE_ERROR = 42
	uint8 TRANSITION_ON_SHUTDOWN_SUCCESS = 50
	uint8 TRANSITION_ON_SHUTDOWN_FAILURE = 51
	uint8 TRANSITION_ON_SHUTDOWN_ERROR = 52
	uint8 TRANSITION_ON_ERROR_SUCCESS = 60
	uint8 TRANSITION_ON_ERROR_FAILURE = 61
	uint8 TRANSITION_ON_ERROR_ERROR = 62
	uint8 TRANSITION_CALLBACK_SUCCESS = 97
	uint8 TRANSITION_CALLBACK_FAILURE = 98
	uint8 TRANSITION_CALLBACK_ERROR = 99
	##
	##
	uint8 id
	string label

# The starting state from which this event transitioned.
State start_state
	uint8 PRIMARY_STATE_UNKNOWN = 0
	uint8 PRIMARY_STATE_UNCONFIGURED = 1
	uint8 PRIMARY_STATE_INACTIVE = 2
	uint8 PRIMARY_STATE_ACTIVE = 3
	uint8 PRIMARY_STATE_FINALIZED = 4
	uint8 TRANSITION_STATE_CONFIGURING = 10
	uint8 TRANSITION_STATE_CLEANINGUP = 11
	uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
	uint8 TRANSITION_STATE_ACTIVATING = 13
	uint8 TRANSITION_STATE_DEACTIVATING = 14
	uint8 TRANSITION_STATE_ERRORPROCESSING = 15
	uint8 id
	string label

# The end state of this transition event.
State goal_state
	uint8 PRIMARY_STATE_UNKNOWN = 0
	uint8 PRIMARY_STATE_UNCONFIGURED = 1
	uint8 PRIMARY_STATE_INACTIVE = 2
	uint8 PRIMARY_STATE_ACTIVE = 3
	uint8 PRIMARY_STATE_FINALIZED = 4
	uint8 TRANSITION_STATE_CONFIGURING = 10
	uint8 TRANSITION_STATE_CLEANINGUP = 11
	uint8 TRANSITION_STATE_SHUTTINGDOWN = 12
	uint8 TRANSITION_STATE_ACTIVATING = 13
	uint8 TRANSITION_STATE_DEACTIVATING = 14
	uint8 TRANSITION_STATE_ERRORPROCESSING = 15
	uint8 id
	string label
```

#### `/dynamic_joint_states`
- **Type**: `control_msgs/msg/DynamicJointState`
- **Interface**:
```
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# List of resource names, e.g. ["arm_joint_1", "arm_joint_2", "gripper_joint"]
string[] joint_names
# Key-value pairs representing interfaces and their corresponding values for each joint listed in `joint_names`
InterfaceValue[] interface_values
	string[] interface_names
	float64[] values
```

#### `/dynamixel_hardware_interface/dxl_state`
- **Type**: `dynamixel_interfaces/msg/DynamixelState`
- **Interface**:
```
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

int32 comm_state
int32[] id
bool[] torque_state
int32[] dxl_hw_state

int32 COMM_STATE_OK = 0
int32 COMM_STATE_CANNOT_FIND_CONTROL_ITEM = -1
int32 COMM_STATE_OPEN_PORT_FAIL = -2
int32 COMM_STATE_INDIRECT_ADDR_FAIL = -3
int32 COMM_STATE_ITEM_WRITE_FAIL = -4
int32 COMM_STATE_ITEM_READ_FAIL = -5
int32 COMM_STATE_SYNC_WRITE_FAIL = -6
int32 COMM_STATE_SYNC_READ_FAIL = -7
int32 COMM_STATE_SET_SYNC_WRITE_FAIL = -8
int32 COMM_STATE_SET_SYNC_READ_FAIL = -9
int32 COMM_STATE_BULK_WRITE_FAIL = -10
int32 COMM_STATE_BULK_READ_FAIL = -11
int32 COMM_STATE_SET_BULK_WRITE_FAIL = -12
int32 COMM_STATE_SET_BULK_READ_FAIL = -13
int32 COMM_STATE_SET_READ_ITEM_FAIL = -14
int32 COMM_STATE_SET_WRITE_ITEM_FAIL = -15
int32 COMM_STATE_DXL_HARDWARE_ERROR = -16
int32 COMM_STATE_DXL_REBOOT_FAIL = -17

# dxl_hw_state = 0: no error
# dxl_hw_state = 1: input voltage error
# dxl_hw_state = 4: overheating
# dxl_hw_state = 8: motor encoder
# dxl_hw_state = 16: electrical shork
# dxl_hw_state = 32: Overload
```

#### `/gripper_controller/transition_event`
- **Type**: `lifecycle_msgs/msg/TransitionEvent`
- **Interface**: Same as `/arm_controller/transition_event`

#### `/joint_state_broadcaster/transition_event`
- **Type**: `lifecycle_msgs/msg/TransitionEvent`
- **Interface**: Same as `/arm_controller/transition_event`

#### `/joint_states`
- **Type**: `sensor_msgs/msg/JointState`
- **Interface**:
```
# This is a message that holds data to describe the state of a set of torque controlled joints.
#
# The state of each joint (revolute or prismatic) is defined by:
#  * the position of the joint (rad or m),
#  * the velocity of the joint (rad/s or m/s) and
#  * the effort that is applied in the joint (Nm or N).
#
# Each joint is uniquely identified by its name
# The header specifies the time at which the joint states were recorded. All the joint states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the joint state.
# The goal is to make each of the fields optional. When e.g. your joints have no
# effort associated with them, you can leave the effort array empty.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.

std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

string[] name
float64[] position
float64[] velocity
float64[] effort
```

#### `/robot_description`
- **Type**: `std_msgs/msg/String`
- **Interface**:
```
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

string data
```



### Subscribed Topics

#### `/arm_controller/joint_trajectory`
- **Type**: `trajectory_msgs/msg/JointTrajectory`
- **Interface**:
```
# The header is used to specify the coordinate frame and the reference time for
# the trajectory durations
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# The names of the active joints in each trajectory point. These names are
# ordered and must correspond to the values in each trajectory point.
string[] joint_names

# Array of trajectory points, which describe the positions, velocities,
# accelerations and/or efforts of the joints at each time point.
JointTrajectoryPoint[] points
	float64[] positions
	float64[] velocities
	float64[] accelerations
	float64[] effort
	builtin_interfaces/Duration time_from_start
		int32 sec
		uint32 nanosec
```