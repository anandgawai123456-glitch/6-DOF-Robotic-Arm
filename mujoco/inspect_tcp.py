import mujoco
import numpy as np


MODEL_PATH = "mujoco/robotic_arm.xml"

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

mujoco.mj_forward(model, data)


tool_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "tool0"
)

print()
print("============================================================")
print(" TOOL / TCP INSPECTION")
print("============================================================")

print()
print("tool0 body position:")
print(data.xpos[tool_id])

print()
print("tool0 body orientation:")
print(data.xmat[tool_id].reshape(3, 3))

print()
print("Tool mesh information:")

for i in range(model.ngeom):

    body_id = model.geom_bodyid[i]

    if body_id == tool_id:

        print()
        print("Geom ID:", i)
        print("Geom name:", model.geom(i).name)
        print("Geom position:", model.geom_pos[i])
        print("Geom rotation:")
        print(data.geom_xmat[i].reshape(3, 3))
        print("Mesh ID:", model.geom_dataid[i])

print()
print("============================================================")
