#!/usr/bin/env python3

import time
import numpy as np
import mujoco
import mujoco.viewer


# ============================================================
# EXISTING ROBOT MODEL
# ============================================================

XML_PATH = "mujoco/robotic_arm.xml"
TCP_SITE_NAME = "tcp"

JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]


# ============================================================
# PICK & PLACE SETTINGS
#
# Everything is relative to the REAL HOME TCP position.
#
# Pick = close to robot
# Place = slightly to the RIGHT
# Both are DOWN near the ground.
# ============================================================

PICK_OFFSET = np.array([
    0.00,     # X
    0.08,     # close to robot
    -0.10     # DOWN toward ground
], dtype=float)

PLACE_OFFSET = np.array([
    0.00,     # X
    -0.08,    # RIGHT / opposite side
    -0.10     # DOWN toward ground
], dtype=float)


# Height used to approach the object.
APPROACH_HEIGHT = 0.07

MOVE_TIME = 2.0
PAUSE_TIME = 0.5

IK_MAX_ITERS = 180
IK_TOL = 1e-4
IK_DAMPING = 1e-2


# ============================================================
# MODEL HELPERS
# ============================================================

def get_joint_ids(model):
    return [
        mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            name
        )
        for name in JOINT_NAMES
    ]


def get_qpos_addresses(model, ids):
    return [
        model.jnt_qposadr[j]
        for j in ids
    ]


def get_dof_addresses(model, ids):
    return [
        model.jnt_dofadr[j]
        for j in ids
    ]


def get_joint_limits(model, ids):
    low = []
    high = []

    for jid in ids:

        if model.jnt_limited[jid]:
            low.append(model.jnt_range[jid, 0])
            high.append(model.jnt_range[jid, 1])
        else:
            low.append(-np.inf)
            high.append(np.inf)

    return np.array(low), np.array(high)


def get_q(data, qpos_addr):
    return np.array(
        [data.qpos[a] for a in qpos_addr],
        dtype=float
    )


def set_q(data, qpos_addr, q):
    for addr, value in zip(qpos_addr, q):
        data.qpos[addr] = value


def get_tcp(model, data, tcp):
    mujoco.mj_forward(model, data)
    return data.site_xpos[tcp].copy()


# ============================================================
# IK
# ============================================================

def solve_ik(
    model,
    data,
    tcp,
    qpos_addr,
    dof_addr,
    q_start,
    target
):

    q = q_start.copy()

    low, high = get_joint_limits(
        model,
        JOINT_IDS
    )

    jacp = np.zeros(
        (3, model.nv),
        dtype=float
    )

    jacr = np.zeros(
        (3, model.nv),
        dtype=float
    )

    for _ in range(IK_MAX_ITERS):

        set_q(
            data,
            qpos_addr,
            q
        )

        mujoco.mj_forward(
            model,
            data
        )

        current = data.site_xpos[tcp].copy()

        error = target - current

        if np.linalg.norm(error) < IK_TOL:
            return q, True

        mujoco.mj_jacSite(
            model,
            data,
            jacp,
            jacr,
            tcp
        )

        J = jacp[:, dof_addr]

        A = (
            J @ J.T
            + IK_DAMPING**2 * np.eye(3)
        )

        try:
            dq = J.T @ np.linalg.solve(
                A,
                error
            )
        except np.linalg.LinAlgError:
            return q, False

        # Keep the joint movement conservative.
        step = np.linalg.norm(dq)

        if step > 0.08:
            dq *= 0.08 / step

        q += dq

        q = np.clip(
            q,
            low,
            high
        )

    set_q(
        data,
        qpos_addr,
        q
    )

    mujoco.mj_forward(
        model,
        data
    )

    final_error = np.linalg.norm(
        target - data.site_xpos[tcp]
    )

    return q, final_error < 0.005


# ============================================================
# SMOOTH JOINT MOTION
# ============================================================

def smoothstep5(t):

    t = np.clip(
        t,
        0.0,
        1.0
    )

    return (
        10.0 * t**3
        - 15.0 * t**4
        + 6.0 * t**5
    )


def move_joints(
    model,
    data,
    viewer,
    qpos_addr,
    q_start,
    q_goal
):

    steps = max(
        1,
        int(MOVE_TIME / model.opt.timestep)
    )

    for i in range(steps + 1):

        if not viewer.is_running():
            return False

        t = i / steps

        s = smoothstep5(t)

        q = (
            q_start
            + s * (q_goal - q_start)
        )

        set_q(
            data,
            qpos_addr,
            q
        )

        mujoco.mj_forward(
            model,
            data
        )

        viewer.sync()

        time.sleep(
            model.opt.timestep
        )

    return True


# ============================================================
# TCP MOTION
# ============================================================

def move_tcp(
    model,
    data,
    viewer,
    tcp,
    qpos_addr,
    dof_addr,
    target,
    name
):

    current_q = get_q(
        data,
        qpos_addr
    )

    solution, success = solve_ik(
        model,
        data,
        tcp,
        qpos_addr,
        dof_addr,
        current_q,
        target
    )

    current_tcp = get_tcp(
        model,
        data,
        tcp
    )

    error = np.linalg.norm(
        target - current_tcp
    )

    print()
    print(f"[MOVE] {name}")

    print(
        "       TARGET:",
        np.array2string(
            target,
            precision=4
        )
    )

    if not success:

        print(
            f"       IK WARNING: "
            f"error = {error:.5f} m"
        )

    else:

        print(
            f"       IK OK: "
            f"error = {error:.5f} m"
        )

    return move_joints(
        model,
        data,
        viewer,
        qpos_addr,
        current_q,
        solution
    )


# ============================================================
# PICK & PLACE SEQUENCE
# ============================================================

def run_pick_place(
    model,
    data,
    viewer,
    tcp,
    qpos_addr,
    dof_addr
):

    print()
    print("=" * 75)
    print(" ROBOTIC ARM — CLOSE-GROUND PICK & PLACE")
    print("=" * 75)

    # --------------------------------------------------------
    # HOME
    # --------------------------------------------------------

    home_q = get_q(
        data,
        qpos_addr
    )

    home_tcp = get_tcp(
        model,
        data,
        tcp
    )

    print()
    print("[1] HOME")

    print(
        "    HOME TCP =",
        np.array2string(
            home_tcp,
            precision=4
        )
    )

    time.sleep(
        PAUSE_TIME
    )

    # --------------------------------------------------------
    # CREATE CLOSE PICK / PLACE LOCATIONS
    # --------------------------------------------------------

    pick = home_tcp + PICK_OFFSET

    place = home_tcp + PLACE_OFFSET

    # Approach points are directly above the ground points.
    pick_approach = pick.copy()
    pick_approach[2] += APPROACH_HEIGHT

    place_approach = place.copy()
    place_approach[2] += APPROACH_HEIGHT

    print()
    print("CLOSE-GROUND TARGETS")
    print(
        "    PICK           =",
        np.array2string(
            pick,
            precision=4
        )
    )

    print(
        "    PICK APPROACH  =",
        np.array2string(
            pick_approach,
            precision=4
        )
    )

    print(
        "    PLACE          =",
        np.array2string(
            place,
            precision=4
        )
    )

    print(
        "    PLACE APPROACH =",
        np.array2string(
            place_approach,
            precision=4
        )
    )

    # --------------------------------------------------------
    # 2. MOVE DOWN NEAR ROBOT
    # --------------------------------------------------------

    print()
    print("[2] MOVE TO PICK APPROACH")

    if not move_tcp(
        model,
        data,
        viewer,
        tcp,
        qpos_addr,
        dof_addr,
        pick_approach,
        "PICK APPROACH"
    ):
        return

    time.sleep(
        PAUSE_TIME
    )

    # --------------------------------------------------------
    # 3. DOWN TO GROUND
    # --------------------------------------------------------

    print()
    print("[3] DOWN TO GROUND — PICK")

    if not move_tcp(
        model,
        data,
        viewer,
        tcp,
        qpos_addr,
        dof_addr,
        pick,
        "GROUND PICK"
    ):
        return

    time.sleep(
        PAUSE_TIME
    )

    print()
    print("    [GRIP] OBJECT PICKED")

    time.sleep(
        PAUSE_TIME
    )

    # --------------------------------------------------------
    # 4. LIFT
    # --------------------------------------------------------

    print()
    print("[4] LIFT")

    if not move_tcp(
        model,
        data,
        viewer,
        tcp,
        qpos_addr,
        dof_addr,
        pick_approach,
        "LIFT"
    ):
        return

    time.sleep(
        PAUSE_TIME
    )

    # --------------------------------------------------------
    # 5. TURN RIGHT
    # --------------------------------------------------------

    print()
    print("[5] TURN RIGHT")

    if not move_tcp(
        model,
        data,
        viewer,
        tcp,
        qpos_addr,
        dof_addr,
        place_approach,
        "TURN RIGHT"
    ):
        return

    time.sleep(
        PAUSE_TIME
    )

    # --------------------------------------------------------
    # 6. DOWN TO GROUND
    # --------------------------------------------------------

    print()
    print("[6] DOWN TO GROUND — PLACE")

    if not move_tcp(
        model,
        data,
        viewer,
        tcp,
        qpos_addr,
        dof_addr,
        place,
        "GROUND PLACE"
    ):
        return

    time.sleep(
        PAUSE_TIME
    )

    print()
    print("    [RELEASE] OBJECT PLACED")

    time.sleep(
        PAUSE_TIME
    )

    # --------------------------------------------------------
    # 7. LIFT
    # --------------------------------------------------------

    print()
    print("[7] LIFT AFTER PLACE")

    if not move_tcp(
        model,
        data,
        viewer,
        tcp,
        qpos_addr,
        dof_addr,
        place_approach,
        "LIFT AFTER PLACE"
    ):
        return

    time.sleep(
        PAUSE_TIME
    )

    # --------------------------------------------------------
    # 8. HOME
    # --------------------------------------------------------

    print()
    print("[8] RETURN HOME")

    move_joints(
        model,
        data,
        viewer,
        qpos_addr,
        get_q(
            data,
            qpos_addr
        ),
        home_q
    )

    print()
    print("=" * 75)
    print(" PICK & PLACE COMPLETE — ROBOT AT HOME")
    print("=" * 75)
    print()


# ============================================================
# MAIN
# ============================================================

def main():

    global JOINT_IDS

    print()
    print("Loading:", XML_PATH)

    model = mujoco.MjModel.from_xml_path(
        XML_PATH
    )

    data = mujoco.MjData(
        model
    )

    JOINT_IDS = get_joint_ids(
        model
    )

    if any(j < 0 for j in JOINT_IDS):

        raise RuntimeError(
            "One or more robot joints were not found."
        )

    tcp = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_SITE,
        TCP_SITE_NAME
    )

    if tcp < 0:

        raise RuntimeError(
            f"TCP site '{TCP_SITE_NAME}' not found."
        )

    qpos_addr = get_qpos_addresses(
        model,
        JOINT_IDS
    )

    dof_addr = get_dof_addresses(
        model,
        JOINT_IDS
    )

    mujoco.mj_forward(
        model,
        data
    )

    print()
    print("=" * 75)
    print(" CLOSE-GROUND PICK & PLACE DEMO")
    print("=" * 75)

    print(
        "MuJoCo:",
        mujoco.__version__
    )

    print(
        "HOME TCP:",
        np.array2string(
            data.site_xpos[tcp],
            precision=4
        )
    )

    print()
    print("Sequence:")
    print("  HOME")
    print("   ↓")
    print("  DOWN near robot")
    print("   ↓")
    print("  PICK")
    print("   ↓")
    print("  LIFT")
    print("   ↓")
    print("  TURN RIGHT")
    print("   ↓")
    print("  DOWN")
    print("   ↓")
    print("  PLACE")
    print("   ↓")
    print("  LIFT")
    print("   ↓")
    print("  HOME")

    print()
    print("Starting MuJoCo...")
    print("=" * 75)

    with mujoco.viewer.launch_passive(
        model,
        data
    ) as viewer:

        run_pick_place(
            model,
            data,
            viewer,
            tcp,
            qpos_addr,
            dof_addr
        )

        # Keep the viewer open after the demo.
        while viewer.is_running():

            mujoco.mj_forward(
                model,
                data
            )

            viewer.sync()

            time.sleep(
                model.opt.timestep
            )


if __name__ == "__main__":

    try:
        main()

    except KeyboardInterrupt:

        print()
        print("Demo stopped.")
