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


def forward_kinematics(q):
    data.qpos[:] = q
    mujoco.mj_forward(model, data)

    return data.site_xpos[tcp_id].copy()


def solve_ik(target_position, initial_q=None):

    if initial_q is None:
        q = np.zeros(6)
    else:
        q = np.array(initial_q, dtype=float)

    max_iterations = 300
    tolerance = 1e-4

    step_size = 0.5
    damping = 0.02

    for iteration in range(max_iterations):

        data.qpos[:] = q
        mujoco.mj_forward(model, data)

        current_position = data.site_xpos[tcp_id].copy()

        error = target_position - current_position
        error_norm = np.linalg.norm(error)

        if error_norm < tolerance:
            return q, True, iteration, error_norm

        Jv = np.zeros((3, model.nv))
        Jw = np.zeros((3, model.nv))

        mujoco.mj_jacSite(
            model,
            data,
            Jv,
            Jw,
            tcp_id
        )

        # Damped least-squares
        A = Jv @ Jv.T + damping**2 * np.eye(3)

        dq = Jv.T @ np.linalg.solve(A, error)

        q += step_size * dq

        # Joint limits
        for i in range(6):
            q[i] = np.clip(
                q[i],
                model.jnt_range[i, 0],
                model.jnt_range[i, 1]
            )

    return q, False, max_iterations, error_norm


# ============================================================
# TEST
# ============================================================

target = np.array([
    0.20,
    -0.20,
    1.40
])

print()
print("============================================================")
print(" ROBOTIC ARM NUMERICAL IK")
print("============================================================")

print()
print("Target TCP position:")
print(target)

q, success, iterations, error = solve_ik(target)

print()
print("------------------------------------------------------------")

if success:
    print("IK STATUS : SUCCESS")
else:
    print("IK STATUS : FAILED")

print("Iterations:", iterations)

print()
print("Joint angles (rad):")
print(q)

print()
print("Joint angles (deg):")
print(np.degrees(q))

final_position = forward_kinematics(q)

print()
print("Target position:")
print(target)

print()
print("Final TCP position:")
print(final_position)

print()
print("Position error:")
print(target - final_position)

print()
print("Position error norm:")
print(error)

print()
print("============================================================")
