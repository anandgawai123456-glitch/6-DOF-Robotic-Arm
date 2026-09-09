import mujoco
import mujoco.viewer
import numpy as np
import time

MODEL_PATH = "mujoco/robotic_arm.xml"

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

tcp_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "tcp"
)


# ============================================================
# TARGET
# ============================================================

target = np.array([
    0.20,
    -0.20,
    1.40
])


# ============================================================
# IK SOLVER
# ============================================================

def solve_ik(target_position):

    q = np.zeros(6)

    max_iterations = 300
    tolerance = 1e-4

    step_size = 0.5
    damping = 0.02

    for iteration in range(max_iterations):

        data.qpos[:] = q
        mujoco.mj_forward(model, data)

        current = data.site_xpos[tcp_id].copy()

        error = target_position - current

        if np.linalg.norm(error) < tolerance:
            return q, iteration

        Jv = np.zeros((3, model.nv))
        Jw = np.zeros((3, model.nv))

        mujoco.mj_jacSite(
            model,
            data,
            Jv,
            Jw,
            tcp_id
        )

        A = Jv @ Jv.T + damping**2 * np.eye(3)

        dq = Jv.T @ np.linalg.solve(A, error)

        q += step_size * dq

        for i in range(6):
            q[i] = np.clip(
                q[i],
                model.jnt_range[i, 0],
                model.jnt_range[i, 1]
            )

    return q, max_iterations


# ============================================================
# SOLVE
# ============================================================

q_solution, iterations = solve_ik(target)

data.qpos[:] = q_solution
mujoco.mj_forward(model, data)

final_position = data.site_xpos[tcp_id].copy()

print()
print("============================================================")
print(" IK VISUALIZATION")
print("============================================================")

print()
print("Target:")
print(target)

print()
print("Solved joints (rad):")
print(q_solution)

print()
print("Solved joints (deg):")
print(np.degrees(q_solution))

print()
print("Final TCP:")
print(final_position)

print()
print("Position error:")
print(np.linalg.norm(target - final_position))

print()
print("Iterations:")
print(iterations)

print()
print("Opening MuJoCo viewer...")
print("Close the viewer window to exit.")


# ============================================================
# VIEWER
# ============================================================

with mujoco.viewer.launch_passive(model, data) as viewer:

    # Keep target marker visible
    model.site_pos[0] = target

    # Freeze robot at IK solution
    data.qpos[:] = q_solution
    mujoco.mj_forward(model, data)

    viewer.sync()

    while viewer.is_running():

        time.sleep(0.01)
        viewer.sync()
