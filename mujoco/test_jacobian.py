import mujoco
import numpy as np


MODEL_PATH = "mujoco/robotic_arm.xml"

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)


# Find TCP site
tcp_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "tcp"
)


print()
print("============================================================")
print(" ROBOTIC ARM TCP JACOBIAN TEST")
print("============================================================")

print()
print("TCP site ID:", tcp_id)


# Zero configuration
data.qpos[:] = 0.0

mujoco.mj_forward(model, data)


# TCP world position
tcp_position = data.site_xpos[tcp_id].copy()

print()
print("TCP position at zero configuration:")
print(
    f"x = {tcp_position[0]: .6f}, "
    f"y = {tcp_position[1]: .6f}, "
    f"z = {tcp_position[2]: .6f}"
)


# Calculate Jacobian
jac_pos = np.zeros((3, model.nv))
jac_rot = np.zeros((3, model.nv))

mujoco.mj_jacSite(
    model,
    data,
    jac_pos,
    jac_rot,
    tcp_id
)


print()
print("Position Jacobian Jv:")
print(jac_pos)


print()
print("Rotational Jacobian Jw:")
print(jac_rot)


print()
print("============================================================")
print(" JACOBIAN SHAPE")
print("============================================================")

print("Position Jacobian shape :", jac_pos.shape)
print("Rotation Jacobian shape :", jac_rot.shape)

print()
print("============================================================")
