import mujoco
import mujoco.viewer
import numpy as np
import time


MODEL_PATH = "mujoco/robotic_arm.xml"


# ============================================================
# LOAD MODEL
# ============================================================

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)


# Find TCP site by name
tcp_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "tcp"
)


# ============================================================
# NUMERICAL IK
# ============================================================

def solve_ik(target):

    # Start from home position
    q = np.zeros(6)

    max_iterations = 300
    tolerance = 1e-4
    step_size = 0.5
    damping = 0.02

    for _ in range(max_iterations):

        # Set current joint configuration
        data.qpos[:] = q

        # Update forward kinematics
        mujoco.mj_forward(model, data)

        # Current TCP position
        current = data.site_xpos[tcp_id].copy()

        # Position error
        error = target - current

        # Check convergence
        if np.linalg.norm(error) < tolerance:
            break

        # Jacobians
        Jv = np.zeros((3, model.nv))
        Jw = np.zeros((3, model.nv))

        mujoco.mj_jacSite(
            model,
            data,
            Jv,
            Jw,
            tcp_id
        )

        # Damped Least Squares
        A = Jv @ Jv.T + damping**2 * np.eye(3)

        dq = Jv.T @ np.linalg.solve(A, error)

        # Update joints
        q += step_size * dq

        # Respect joint limits
        for i in range(6):
            q[i] = np.clip(
                q[i],
                model.jnt_range[i, 0],
                model.jnt_range[i, 1]
            )

    return q


# ============================================================
# TARGET
# ============================================================

target = np.array([
    0.20,
    -0.20,
    1.40
])


# Home configuration
q_home = np.zeros(6)


# Solve IK
q_target = solve_ik(target)


# ============================================================
# VERIFY IK RESULT
# ============================================================

data.qpos[:] = q_target
mujoco.mj_forward(model, data)

final_tcp = data.site_xpos[tcp_id].copy()
position_error = target - final_tcp
error_norm = np.linalg.norm(position_error)


# ============================================================
# PRINT RESULTS
# ============================================================

print()
print("============================================================")
print(" ROBOTIC ARM IK MOTION DEMO")
print("============================================================")

print()
print("Target TCP:")
print(target)

print()
print("IK solution (rad):")
print(q_target)

print()
print("IK solution (deg):")
print(np.degrees(q_target))

print()
print("Final TCP:")
print(final_tcp)

print()
print("Position error:")
print(position_error)

print()
print("Position error norm:")
print(error_norm)

print()
print("Opening viewer...")


# ============================================================
# VISUALIZATION
# ============================================================

with mujoco.viewer.launch_passive(model, data) as viewer:

    # --------------------------------------------------------
    # TARGET MARKER
    # --------------------------------------------------------

    model.site_pos[0] = target

    # --------------------------------------------------------
    # START AT HOME
    # --------------------------------------------------------

    data.qpos[:] = q_home

    mujoco.mj_forward(model, data)

    viewer.sync()

    time.sleep(2)


    # --------------------------------------------------------
    # MOTION SETTINGS
    # --------------------------------------------------------

    duration = 3.0
    steps = 180


    # ========================================================
    # HOME → TARGET
    # ========================================================

    start_q = q_home.copy()

    for i in range(steps):

        if not viewer.is_running():
            break

        alpha = (i + 1) / steps

        # Smooth cubic interpolation
        smooth = 3 * alpha**2 - 2 * alpha**3

        # Interpolate joint positions
        data.qpos[:] = (
            (1 - smooth) * start_q
            + smooth * q_target
        )

        # Update kinematics
        mujoco.mj_forward(model, data)

        # Update viewer
        viewer.sync()

        time.sleep(duration / steps)


    print()
    print("Reached target.")

    time.sleep(3)


    # ========================================================
    # TARGET → HOME
    # ========================================================

    for i in range(steps):

        if not viewer.is_running():
            break

        alpha = (i + 1) / steps

        # Smooth cubic interpolation
        smooth = 3 * alpha**2 - 2 * alpha**3

        # Interpolate back to home
        data.qpos[:] = (
            (1 - smooth) * q_target
            + smooth * q_home
        )

        # Update kinematics
        mujoco.mj_forward(model, data)

        # Update viewer
        viewer.sync()

        time.sleep(duration / steps)


    print("Returned to home.")

    print()
    print("Demo complete.")
    


    # --------------------------------------------------------
    # KEEP VIEWER ALIVE
    # --------------------------------------------------------

    while viewer.is_running():

        viewer.sync()

        time.sleep(0.05)


