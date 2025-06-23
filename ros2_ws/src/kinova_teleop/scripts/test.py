import mujoco
import mujoco.viewer
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(os.path.dirname(os.path.dirname(current_dir)), 'mujoco_menagerie', 'kinova_gen3', 'scene.xml')

model = mujoco.MjModel.from_xml_path(model_path)

# Create data instance for the model
data = mujoco.MjData(model)

# Launch the MuJoCo viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Simulate and render
    for _ in range(10000):  # Run simulation for 10000 steps
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # Break the loop if the viewer is closed
        if not viewer.is_running():
            break
