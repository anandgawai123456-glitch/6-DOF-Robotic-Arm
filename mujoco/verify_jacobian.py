import mujoco
import numpy as np


MODEL_PATH = "mujoco/robotic_arm.xml"

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

tcp_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "tcp"
)


# ------------------------------------------------------------
# Initial configuration
# ------------------------------------------------------------

q = np.array([
    0.3,
    0.4,
    -0.5,
    0.6,
    0.2,
    -0.3
])

data.qpos[:] = q
mujoco.mj_forward(model, data)

p0 = data.site_xpos[tcp_id].copy()


# ------------------------------------------------------------
# Calculate MuJoCo Jacobian
# ------------------------------------------------------------

Jv = np.zeros((3, model.nv))
Jw = np.zeros((3, model.nv))

mujoco.mj_jacSite(
    model,
    data,
    Jv,
    Jw,
    tcp_id
)


print()
print("============================================================")
print(" JACOBIAN FINITE-DIFFERENCE VERIFICATION")
print("============================================================")

print()
print("Initial joint configuration:")
print(q)

print()
print("Initial TCP position:")
print(p0)

print()
print("Position Jacobian:")
print(Jv)


# ------------------------------------------------------------
# Finite difference
# ------------------------------------------------------------

epsilon = 1e-6

print()
print("------------------------------------------------------------")
print("FINITE DIFFERENCE RESULTS")
print("------------------------------------------------------------")

for i in range(6):

    q_test = q.copy()
    q_test[i] += epsilon

    data.qpos[:] = q_test
    mujoco.mj_forward(model, data)

    p1 = data.site_xpos[tcp_id].copy()

    actual_change = p1 - p0

    predicted_change = Jv[:, i] * epsilon

    error = actual_change - predicted_change

    print()
    print(f"Joint {i + 1}")
    print("Actual TCP change    :", actual_change)
    print("Predicted TCP change :", predicted_change)
    print("Error                :", error)
    print("Error norm           :", np.linalg.norm(error))


print()
print("============================================================")
print(" VERIFICATION COMPLETE")
print("============================================================")
