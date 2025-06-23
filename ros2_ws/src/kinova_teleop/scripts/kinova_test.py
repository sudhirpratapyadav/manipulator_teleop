from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter

import mink

_ARM_XML = Path("/home/robot/yash/ros2_ws/src/mujoco_menagerie/kinova_gen3/scene.xml")
_HAND_XML = Path("/home/robot/yash/ros2_ws/src/mujoco_menagerie/robotiq_2f85/2f85.xml")

# Update finger list for Robotiq 2F85 gripper
fingers = ["left_pad", "right_pad"]

# fmt: off
# Updated HOME_QPOS without mobile base components
HOME_QPOS = [
    # Kinova.
    0, 0.26179939, 3.14159265, -2.26892803, 0, 0.95993109, 1.57079633,
    # Robotiq 2F85 gripper (all joints that get created after attachment)
    0, 0, 0, 0, 0, 0, 0, 0
]
# fmt: on

# IK parameters
SOLVER = "daqp"
POS_THRESHOLD = 1e-4
ORI_THRESHOLD = 1e-4
MAX_ITERS = 20


def construct_model() -> mujoco.MjModel:
    arm = mujoco.MjSpec.from_file(_ARM_XML.as_posix())
    hand = mujoco.MjSpec.from_file(_HAND_XML.as_posix())

    # Find the base mount of the Robotiq 2F85 gripper
    base_mount = hand.body("base_mount")
    base_mount.pos = (0, 0, 0.0)  # Adjust position as needed
    base_mount.quat = (0, 0, 0, 1)  # Adjust orientation as needed
    site = arm.site("pinch_site")
    arm.attach(hand, prefix="robotiq_2f85/", site=site)

    for key in arm.keys:
        if key.name in ["home", "retract"]:
            key.delete()
    arm.add_key(name="home", qpos=HOME_QPOS)

    # Add a mocap body for the target
    body = arm.worldbody.add_body(name="target", mocap=True)
    body.add_geom(
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=(0.02,) * 3,
        contype=0,
        conaffinity=0,
        rgba=(0.6, 0.3, 0.3, 0.5),
    )
    
    # Add reference to the gripper's pinch site
    arm.worldbody.add_site(
        name="gripper_pinch_site", 
        pos=(0, 0, 0), 
        quat=(1, 0, 0, 0), 
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=(0.01, 0.01, 0.01),
        rgba=(0, 1, 0, 0.5)
    )

    return arm.compile()


def converge_ik(
    configuration, tasks, dt, solver, pos_threshold, ori_threshold, max_iters
):
    """
    Runs up to 'max_iters' of IK steps. Returns True if position and orientation
    are below thresholds, otherwise False.
    """
    for _ in range(max_iters):
        vel = mink.solve_ik(configuration, tasks, dt, solver, 1e-3)
        configuration.integrate_inplace(vel, dt)

        # Only checking the first FrameTask here (end_effector_task).
        err = tasks[0].compute_error(configuration)
        pos_achieved = np.linalg.norm(err[:3]) <= pos_threshold
        ori_achieved = np.linalg.norm(err[3:]) <= ori_threshold

        if pos_achieved and ori_achieved:
            return True
    return False


def control_gripper(data, position):
    """
    Control the Robotiq 2F85 gripper.
    
    Args:
        data: MjData object containing the simulation state
        position: Normalized position value [0, 1] where:
                 0 = fully open
                 1 = fully closed
    """
    # Clamp input to valid range [0, 1]
    position = max(0.0, min(1.0, position))
    
    # Map from [0, 1] to actuator control range [0, 255]
    # The gripper actuator is defined in the XML with ctrlrange="0 255"
    ctrl_value = position * 255.0
    
    # Find the gripper actuator index (assuming it's the last actuator)
    # For the Robotiq 2F85, the actuator name is "fingers_actuator"
    gripper_actuator_idx = data.ctrl.shape[0] - 1
    
    # Set the control value
    data.ctrl[gripper_actuator_idx] = ctrl_value
    
    return ctrl_value


def main():
    model = construct_model()
    data = mujoco.MjData(model)

    # Create a Mink configuration
    configuration = mink.Configuration(model)

    # Define tasks
    end_effector_task = mink.FrameTask(
        frame_name="robotiq_2f85/pinch",
        frame_type="site",
        position_cost=1.0,
        orientation_cost=1.0,
        lm_damping=1.0,
    )

    # Simple posture task with small cost
    posture_task = mink.PostureTask(model=model, cost=1e-2)
    
    tasks = [end_effector_task, posture_task]

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=False,
        show_right_ui=False,
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Reset to home position
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
        configuration.update(data.qpos)
        posture_task.set_target_from_configuration(configuration)
        mujoco.mj_forward(model, data)

        # Initialize the mocap target at the end-effector site
        mink.move_mocap_to_frame(model, data, "target", "robotiq_2f85/pinch", "site")
        initial_target_position = data.mocap_pos[0].copy()

        # Circular trajectory parameters
        amp = 0.10  # 10cm amplitude
        freq = 0.2  # 0.2Hz frequency

        # Time tracking for smooth trajectory
        local_time = 0.0
        rate = RateLimiter(frequency=200.0, warn=False)
        
        # Initialize gripper state
        gripper_position = 0.0  # Start with open gripper
        gripper_cycle_time = 3.0  # Time in seconds for complete open/close cycle

        while viewer.is_running():
            # Update local time
            dt = rate.dt
            local_time += dt

            # Circular offset for trajectory
            offset = np.array(
                [
                    amp * np.cos(2 * np.pi * freq * local_time),
                    amp * np.sin(2 * np.pi * freq * local_time),
                    0.0,
                ]
            )
            data.mocap_pos[0] = initial_target_position + offset

            # Update the end effector task target from the mocap body
            T_wt = mink.SE3.from_mocap_name(model, data, "target")
            end_effector_task.set_target(T_wt)

            # Attempt to converge IK
            converge_ik(
                configuration,
                tasks,
                dt,
                SOLVER,
                POS_THRESHOLD,
                ORI_THRESHOLD,
                MAX_ITERS,
            )

            # Set robot controls (only the arm joints)
            data.ctrl[:model.nu-1] = configuration.q[:model.nu-1]
            
            # Control the gripper - create a smooth opening and closing pattern
            # Calculate a sinusoidal pattern between 0 and 1 for smooth transition
            gripper_position = 0.5 * (1.0 + np.sin(2 * np.pi * local_time / gripper_cycle_time))
            control_gripper(data, gripper_position)

            # Step simulation
            mujoco.mj_step(model, data)

            # Visualize at fixed FPS
            viewer.sync()
            rate.sleep()


if __name__ == "__main__":
    main()
