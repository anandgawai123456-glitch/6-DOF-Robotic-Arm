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
# TARGET
# ------------------------------------------------------------
# We generate the target from a known joint configuration.
# This gives us a known-good IK test.
target_q = np.array([
    0.3,
    0.4,
    -0.5,
    0.6,
    0.2,
    -0.3
])

data.qpos[:] = target_q
mujoco.mj_forward(model, data)

target_pos = data.site_xpos[tcp_id].copy()

# ------------------------------------------------------------
# INITIAL GUESS
# ------------------------------------------------------------

q = np.array([
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
])

# ------------------------------------------------------------
# IK PARAMETERS
# ------------------------------------------------------------

max_iterations = 200
tolerance = 1e-5
step_size = 0.5
damping = 0.01

print()
print("============================================================")
print(" NUMERICAL IK TEST")
print("============================================================")

print()
print("Target joint configuration:")
print(target_q)

print()
print("Target TCP position:")
print(target_pos)

print()
print("Initial guess:")
print(q)

# ------------------------------------------------------------
# ITERATIVE IK
# ------------------------------------------------------------

for iteration in range(max_iterations):

    data.qpos[:] = q
    mujoco.mj_forward(model, data)

    current_pos = data.site_xpos[tcp_id].copy()

    error = target_pos - current_pos
    error_norm = np.linalg.norm(error)

    if error_norm < tolerance:
        print()
        print("IK CONVERGED")
        print("Iterations :", iteration)
        break

    Jv = np.zeros((3, model.nv))
    Jw = np.zeros((3, model.nv))

    mujoco.mj_jacSite(
        model,
        data,
        Jv,
        Jw,
        tcp_id
    )

    # Damped least-squares inverse
    A = Jv @ Jv.T + (damping ** 2) * np.eye(3)

    dq = Jv.T @ np.linalg.solve(A, error)

    q += step_size * dq

    # Keep joints within reasonable limits
    for i in range(6):
        q[i] = np.clip(
            q[i],
            model.jnt_range[i, 0],
            model.jnt_range[i, 1]
        )

else:
    print()
    print("IK DID NOT CONVERGE")

# ------------------------------------------------------------
# FINAL RESULT
# ------------------------------------------------------------

data.qpos[:] = q
mujoco.mj_forward(model, data)

final_pos = data.site_xpos[tcp_id].copy()

final_error = target_pos - final_pos

print()
print("------------------------------------------------------------")
print("FINAL IK RESULT")
print("------------------------------------------------------------")

print()
print("Solved joint angles:")
print(q)

print()
print("Target TCP:")
print(target_pos)

print()
print("Final TCP:")
print(final_pos)

print()
print("Position error:")
print(final_error)

print()
print("Position error norm:")
print(np.linalg.norm(final_error))

print()
print("Expected joint configuration:")
print(target_q)

print()
print("Joint error:")
print(q - target_q)

print()
print("============================================================")
