import mujoco
import mujoco.viewer
import time

MODEL_PATH = "mujoco/robotic_arm.xml"

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

data.qpos[:] = 0.0
mujoco.mj_forward(model, data)

print()
print("============================================================")
print(" ROBOTIC ARM VISUAL CHECK")
print("============================================================")
print(f"Bodies : {model.nbody}")
print(f"Joints : {model.njnt}")
print(f"DOFs   : {model.nv}")
print(f"Meshes : {model.nmesh}")
print()

print("Joint axes at zero position:")
for i in range(model.njnt):

    name = mujoco.mj_id2name(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        i
    )

    body_id = model.jnt_bodyid[i]

    axis_local = model.jnt_axis[i].copy()

    axis_world = data.xmat[body_id].reshape(3, 3) @ axis_local

    print(
        f"{name:10s} "
        f"world axis = "
        f"[{axis_world[0]: .3f}, "
        f"{axis_world[1]: .3f}, "
        f"{axis_world[2]: .3f}]"
    )

print()
print("Launching MuJoCo viewer...")
print("Close the viewer to exit.")
print()

with mujoco.viewer.launch_passive(model, data) as viewer:

    viewer.cam.azimuth = 135
    viewer.cam.elevation = -15
    viewer.cam.distance = 2.4
    viewer.cam.lookat[:] = [0.0, 0.0, 0.8]

    while viewer.is_running():

        mujoco.mj_forward(model, data)

        viewer.sync()

        time.sleep(0.01)
