import time
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("mujoco/robotic_arm.xml")

# Disable gravity for this visual test
model.opt.gravity[:] = 0

data = mujoco.MjData(model)

# Set a known configuration
data.qpos[0] = 0.0    # joint 1
data.qpos[1] = 0.5    # joint 2
data.qpos[2] = -0.8   # joint 3
data.qpos[3] = 0.8    # joint 4

mujoco.mj_forward(model, data)

print("Robot initialized")
print("Joint positions:", data.qpos)

with mujoco.viewer.launch_passive(model, data) as viewer:

    # Give the viewer a reasonable initial camera position
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -20
    viewer.cam.distance = 2.5
    viewer.cam.lookat[:] = data.xpos[1]

    while viewer.is_running():

        mujoco.mj_forward(model, data)

        viewer.sync()
        time.sleep(0.01)

