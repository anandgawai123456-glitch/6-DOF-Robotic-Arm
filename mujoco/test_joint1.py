import time
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("mujoco/robotic_arm.xml")
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        # Rotate joint_1 continuously
        data.qpos[0] = 0.8

        mujoco.mj_forward(model, data)

        viewer.sync()
        time.sleep(0.01)
