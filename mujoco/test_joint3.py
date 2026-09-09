import time
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("mujoco/robotic_arm.xml")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:

    # Joint 1
    data.qpos[0] = 0.0

    # Joint 2
    data.qpos[1] = 0.0

    # Joint 3
    data.qpos[2] = 1.0

    mujoco.mj_forward(model, data)

    while viewer.is_running():
        viewer.sync()
        time.sleep(0.01)
