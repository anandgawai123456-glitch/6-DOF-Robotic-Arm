import json
import math
import os
import select
import sys
import termios
import time
import tty

import numpy as np
import mujoco
import mujoco.viewer


# ============================================================
# ROBOTIC ARM
# DIRECT KINEMATIC CONTROLLER + MOTION RECORDER
#
# IMPORTANT
#
# NO mj_step()
#
# We directly modify joint qpos and call mj_forward().
#
# Therefore:
#
#     keyboard
#         ↓
#     one command
#         ↓
#     one qpos update
#         ↓
#     mj_forward()
#         ↓
#     viewer
#
# There is NO dynamic simulation.
# There is NO command queue.
# There is NO background trajectory controller.
# ============================================================


MODEL_PATH = "mujoco/robotic_arm.xml"
MOTION_FILE = "mujoco/recorded_motion.json"


# ============================================================
# CONTROL SETTINGS
# ============================================================

LOOP_DT = 0.01

JOINT_STEP = math.radians(5.0)

XYZ_STEP = 0.005

# Prevent terminal key-repeat from generating dozens
# of movements from one held key.
KEY_DEBOUNCE = 0.10


# ============================================================
# IK SETTINGS
# ============================================================

IK_DAMPING = 0.15
IK_STEP = 0.20
IK_ITERATIONS = 12

POSITION_TOLERANCE = 1e-4

MAX_IK_JOINT_STEP = math.radians(2.0)


# ============================================================
# RECORDING
# ============================================================

RECORD_DT = 0.02


# ============================================================
# MODEL
# ============================================================

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
# GLOBAL STATE
# ============================================================

model = None
data = None

running = True

mode = "JOINT"

selected_joint = 0

cartesian_target = None


# ============================================================
# PLAYBACK STATE
# ============================================================

playback = False
playback_motion = []
playback_start_time = 0.0


# ============================================================
# RECORDING STATE
# ============================================================

recording = False
recorded_motion = []

record_start_time = 0.0
last_record_time = 0.0


# ============================================================
# JOINT MAPS
# ============================================================

joint_qpos_adr = {}
joint_dof_adr = {}
joint_limits = {}


# ============================================================
# KEYBOARD
# ============================================================

class Keyboard:

    def __init__(self):

        self.fd = sys.stdin.fileno()

        self.old_settings = termios.tcgetattr(
            self.fd
        )

        self.last_key = None
        self.last_key_time = 0.0

    def start(self):

        tty.setcbreak(self.fd)

    def stop(self):

        termios.tcsetattr(
            self.fd,
            termios.TCSADRAIN,
            self.old_settings
        )

    def get_key(self):

        readable, _, _ = select.select(
            [sys.stdin],
            [],
            [],
            0
        )

        if not readable:
            return None

        key = sys.stdin.read(1)

        now = time.monotonic()

        # ----------------------------------------------------
        # DEBOUNCE
        #
        # Prevent terminal auto-repeat from turning one
        # physical press into many robot movements.
        # ----------------------------------------------------

        if (
            key == self.last_key
            and
            (now - self.last_key_time)
            < KEY_DEBOUNCE
        ):
            return None

        self.last_key = key
        self.last_key_time = now

        return key


# ============================================================
# MODEL / JOINT MAP
# ============================================================

def build_joint_maps():

    joint_qpos_adr.clear()
    joint_dof_adr.clear()
    joint_limits.clear()

    for name in JOINT_NAMES:

        jid = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            name
        )

        if jid < 0:

            raise RuntimeError(
                f"Joint not found: {name}"
            )

        joint_qpos_adr[name] = int(
            model.jnt_qposadr[jid]
        )

        joint_dof_adr[name] = int(
            model.jnt_dofadr[jid]
        )

        if model.jnt_limited[jid]:

            joint_limits[name] = (
                float(model.jnt_range[jid, 0]),
                float(model.jnt_range[jid, 1])
            )

        else:

            joint_limits[name] = (
                -math.inf,
                math.inf
            )


# ============================================================
# JOINT STATE
# ============================================================

def get_joint_positions():

    q = np.zeros(6)

    for i, name in enumerate(JOINT_NAMES):

        q[i] = data.qpos[
            joint_qpos_adr[name]
        ]

    return q


def apply_joint_limits(q):

    q = np.asarray(
        q,
        dtype=float
    ).copy()

    for i, name in enumerate(JOINT_NAMES):

        low, high = joint_limits[name]

        q[i] = np.clip(
            q[i],
            low,
            high
        )

    return q


def set_joint_positions(q):

    q = apply_joint_limits(q)

    for i, name in enumerate(JOINT_NAMES):

        adr = joint_qpos_adr[name]

        data.qpos[adr] = q[i]


# ============================================================
# FORWARD KINEMATICS
# ============================================================

def forward():

    mujoco.mj_forward(
        model,
        data
    )


# ============================================================
# TCP
# ============================================================

def get_tcp_id():

    sid = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_SITE,
        TCP_SITE_NAME
    )

    if sid < 0:

        raise RuntimeError(
            f"TCP site '{TCP_SITE_NAME}' not found"
        )

    return sid


def get_tcp_position():

    sid = get_tcp_id()

    return data.site_xpos[sid].copy()


def get_tcp_rotation():

    sid = get_tcp_id()

    return data.site_xmat[
        sid
    ].reshape(3, 3).copy()


# ============================================================
# CARTESIAN TARGET
# ============================================================

def sync_target_to_tcp():

    global cartesian_target

    forward()

    cartesian_target = (
        get_tcp_position().copy()
    )


# ============================================================
# JACOBIAN
# ============================================================

def get_position_jacobian():

    sid = get_tcp_id()

    jac_pos = np.zeros(
        (3, model.nv)
    )

    jac_rot = np.zeros(
        (3, model.nv)
    )

    mujoco.mj_jacSite(
        model,
        data,
        jac_pos,
        jac_rot,
        sid
    )

    J = np.zeros(
        (3, 6)
    )

    for i, name in enumerate(JOINT_NAMES):

        dof = joint_dof_adr[name]

        J[:, i] = jac_pos[:, dof]

    return J


# ============================================================
# IK
# ============================================================

def solve_ik(target_position):

    target_position = np.asarray(
        target_position,
        dtype=float
    )

    q = get_joint_positions()

    for _ in range(IK_ITERATIONS):

        set_joint_positions(q)

        forward()

        current_position = (
            get_tcp_position()
        )

        error = (
            target_position
            - current_position
        )

        error_norm = np.linalg.norm(
            error
        )

        if error_norm < POSITION_TOLERANCE:
            break

        J = get_position_jacobian()

        damping = (
            IK_DAMPING ** 2
        ) * np.eye(3)

        try:

            dq = (
                J.T
                @ np.linalg.solve(
                    J @ J.T + damping,
                    error
                )
            )

        except np.linalg.LinAlgError:

            dq = (
                np.linalg.pinv(J)
                @ error
            )

        dq *= IK_STEP

        maximum = np.max(
            np.abs(dq)
        )

        if maximum > MAX_IK_JOINT_STEP:

            dq *= (
                MAX_IK_JOINT_STEP
                / maximum
            )

        q += dq

        q = apply_joint_limits(q)

    set_joint_positions(q)

    forward()

    return q


# ============================================================
# JOINT CONTROL
# ============================================================

def move_joint(index, direction):

    global cartesian_target

    q = get_joint_positions()

    old = q[index]

    q[index] += (
        direction
        * JOINT_STEP
    )

    q = apply_joint_limits(q)

    set_joint_positions(q)

    forward()

    # Cartesian target follows the actual robot.
    cartesian_target = (
        get_tcp_position().copy()
    )

    new = q[index]

    print(
        f"J{index + 1}: "
        f"{math.degrees(old):.2f}°"
        f" -> "
        f"{math.degrees(new):.2f}°"
    )


# ============================================================
# WORLD CONTROL
# ============================================================

def move_world(axis, direction):

    global cartesian_target

    if cartesian_target is None:

        sync_target_to_tcp()

    target = (
        cartesian_target.copy()
    )

    axis_index = {
        "x": 0,
        "y": 1,
        "z": 2
    }[axis]

    target[axis_index] += (
        direction
        * XYZ_STEP
    )

    solve_ik(target)

    forward()

    actual = get_tcp_position()

    # IMPORTANT:
    # Use actual achieved TCP position.
    cartesian_target = actual.copy()

    print(
        f"WORLD {axis.upper()}"
        f"{'+' if direction > 0 else '-'}"
        f"  TCP = "
        f"[{actual[0]:.4f}, "
        f"{actual[1]:.4f}, "
        f"{actual[2]:.4f}]"
    )


# ============================================================
# TOOL CONTROL
# ============================================================

def move_tool(axis, direction):

    global cartesian_target

    if cartesian_target is None:

        sync_target_to_tcp()

    R = get_tcp_rotation()

    local_move = np.zeros(3)

    axis_index = {
        "x": 0,
        "y": 1,
        "z": 2
    }[axis]

    local_move[axis_index] = (
        direction
        * XYZ_STEP
    )

    world_move = (
        R @ local_move
    )

    target = (
        cartesian_target
        + world_move
    )

    solve_ik(target)

    forward()

    actual = get_tcp_position()

    cartesian_target = actual.copy()

    print(
        f"TOOL {axis.upper()}"
        f"{'+' if direction > 0 else '-'}"
        f"  TCP = "
        f"[{actual[0]:.4f}, "
        f"{actual[1]:.4f}, "
        f"{actual[2]:.4f}]"
    )


# ============================================================
# HOME
# ============================================================

def go_home():

    global cartesian_target

    # Stop anything that could overwrite HOME.
    stop_playback_internal()

    q = np.zeros(6)

    set_joint_positions(q)

    forward()

    cartesian_target = (
        get_tcp_position().copy()
    )

    print("HOME")


# ============================================================
# PLAYBACK CONTROL
# ============================================================

def stop_playback_internal():

    global playback

    playback = False


def stop_motion():

    global cartesian_target

    # REAL STOP.
    stop_playback_internal()

    forward()

    cartesian_target = (
        get_tcp_position().copy()
    )

    print("STOP")


# ============================================================
# RECORDING
# ============================================================

def start_recording():

    global recording
    global recorded_motion
    global record_start_time
    global last_record_time

    recorded_motion = []

    recording = True

    record_start_time = time.monotonic()

    last_record_time = 0.0

    print("RECORDING STARTED")


def stop_recording():

    global recording

    recording = False

    print(
        f"RECORDING STOPPED "
        f"({len(recorded_motion)} samples)"
    )


def toggle_recording():

    if recording:

        stop_recording()

    else:

        start_recording()


def update_recording():

    global last_record_time

    if not recording:
        return

    now = time.monotonic()

    if (
        last_record_time != 0.0
        and
        now - last_record_time < RECORD_DT
    ):
        return

    last_record_time = now

    q = get_joint_positions()

    elapsed = (
        now
        - record_start_time
    )

    recorded_motion.append(
        {
            "time": float(elapsed),
            "q": q.tolist()
        }
    )


# ============================================================
# SAVE
# ============================================================

def save_recording():

    if not recorded_motion:

        print(
            "Nothing recorded."
        )

        return

    directory = os.path.dirname(
        MOTION_FILE
    )

    if directory:

        os.makedirs(
            directory,
            exist_ok=True
        )

    timestamp = time.strftime("%Y%m%d_%H%M%S")

    save_file = os.path.join(
        directory,
        f"recording_{timestamp}.json"
    )

    counter = 1

    while os.path.exists(save_file):

        save_file = os.path.join(
            directory,
            f"recording_{timestamp}_{counter:02d}.json"
        )

        counter += 1

    with open(
        save_file,
        "w"
    ) as f:

        json.dump(
            recorded_motion,
            f,
            indent=2
        )

    print(
        f"Saved {len(recorded_motion)} samples to:"
    )

    print(
        save_file
    )


# ============================================================
# PLAYBACK LOAD
# ============================================================

def load_playback():

    global playback_motion

    import glob

    # Prefer the newest timestamped recording.
    recording_files = glob.glob(
        os.path.join(
            os.path.dirname(MOTION_FILE),
            "recording_*.json"
        )
    )

    if recording_files:

        filename = max(
            recording_files,
            key=os.path.getmtime
        )

    else:

        filename = MOTION_FILE

    if not os.path.exists(filename):

        print(
            f"File not found: {filename}"
        )

        return False

    try:

        with open(filename, "r") as f:

            data = json.load(f)

        if isinstance(data, dict):

            if "frames" in data:

                data = data["frames"]

            elif "motion" in data:

                data = data["motion"]

            elif "samples" in data:

                data = data["samples"]

        if not isinstance(data, list):

            print(
                "Invalid playback file format."
            )

            return False

        playback_motion = data

        print(
            f"Playback loaded: {len(playback_motion)} samples"
        )

        if playback_motion:

            duration = playback_motion[-1].get(
                "time",
                0.0
            )

            print(
                f"Duration: {float(duration):.3f} s"
            )

        print(
            f"File: {filename}"
        )

        return True

    except Exception as e:

        print(
            f"Playback load error: {e}"
        )

        return False


# ============================================================
# START PLAYBACK
# ============================================================

def start_playback():

    global playback
    global playback_start_time

    if recording:

        print(
            "Stop recording before playback."
        )

        return

    if not load_playback():

        return

    playback = True

    playback_start_time = (
        time.monotonic()
    )

    print(
        "PLAYBACK STARTED"
    )


# ============================================================
# UPDATE PLAYBACK
# ============================================================

def update_playback():

    global playback
    global cartesian_target

    if not playback:

        return

    elapsed = (
        time.monotonic()
        - playback_start_time
    )

    first = playback_motion[0]

    last = playback_motion[-1]

    first_t = float(
        first["time"]
    )

    last_t = float(
        last["time"]
    )

    # --------------------------------------------------------
    # BEFORE FIRST SAMPLE
    # --------------------------------------------------------

    if elapsed <= first_t:

        q = np.asarray(
            first["q"],
            dtype=float
        )

    # --------------------------------------------------------
    # AFTER LAST SAMPLE
    # --------------------------------------------------------

    elif elapsed >= last_t:

        q = np.asarray(
            last["q"],
            dtype=float
        )

        set_joint_positions(q)

        forward()

        cartesian_target = (
            get_tcp_position().copy()
        )

        playback = False

        print(
            "PLAYBACK FINISHED"
        )

        return

    # --------------------------------------------------------
    # BETWEEN SAMPLES
    # --------------------------------------------------------

    else:

        index = 0

        for i in range(
            len(playback_motion) - 1
        ):

            t0 = float(
                playback_motion[i]["time"]
            )

            t1 = float(
                playback_motion[i + 1]["time"]
            )

            if t0 <= elapsed <= t1:

                index = i

                break

        p0 = playback_motion[index]

        p1 = playback_motion[index + 1]

        t0 = float(
            p0["time"]
        )

        t1 = float(
            p1["time"]
        )

        q0 = np.asarray(
            p0["q"],
            dtype=float
        )

        q1 = np.asarray(
            p1["q"],
            dtype=float
        )

        if t1 <= t0:

            alpha = 0.0

        else:

            alpha = (
                elapsed - t0
            ) / (
                t1 - t0
            )

        alpha = np.clip(
            alpha,
            0.0,
            1.0
        )

        q = (
            q0
            +
            alpha
            *
            (
                q1 - q0
            )
        )

    set_joint_positions(q)

    forward()

    cartesian_target = (
        get_tcp_position().copy()
    )


# ============================================================
# MODE
# ============================================================

def set_mode(new_mode):

    global mode

    # Mode changes never move the robot.
    mode = new_mode

    if mode in (
        "WORLD",
        "TOOL"
    ):

        sync_target_to_tcp()

    print(
        f"MODE: {mode}"
    )


# ============================================================
# STATUS
# ============================================================

def print_status():

    q = get_joint_positions()

    tcp = get_tcp_position()

    print()
    print("=" * 70)
    print("ROBOT STATUS")
    print("=" * 70)

    print(
        f"Mode           : {mode}"
    )

    print(
        f"Selected joint : J{selected_joint + 1}"
    )

    print(
        f"Recording      : "
        f"{'ON' if recording else 'OFF'}"
    )

    print(
        f"Playback       : "
        f"{'ON' if playback else 'OFF'}"
    )

    print()

    for i in range(6):

        print(
            f"J{i + 1}: "
            f"{math.degrees(q[i]): .3f}°"
        )

    print()

    print(
        f"TCP X: {tcp[0]: .6f}"
    )

    print(
        f"TCP Y: {tcp[1]: .6f}"
    )

    print(
        f"TCP Z: {tcp[2]: .6f}"
    )

    print("=" * 70)


# ============================================================
# HELP
# ============================================================

def print_help():

    print()
    print("=" * 70)
    print("ROBOTIC ARM CONTROL")
    print("=" * 70)

    print()
    print("MODES")
    print("  j       JOINT mode")
    print("  w       WORLD mode")
    print("  t       TOOL mode")

    print()
    print("JOINT MODE")
    print("  1-6     Select J1-J6")
    print("  + / =   Selected joint +5°")
    print("  -       Selected joint -5°")

    print()
    print("WORLD MODE")
    print("  x       X +5 mm")
    print("  X       X -5 mm")
    print("  y       Y +5 mm")
    print("  Y       Y -5 mm")
    print("  z       Z +5 mm")
    print("  Z       Z -5 mm")

    print()
    print("TOOL MODE")
    print("  x       Tool X +5 mm")
    print("  X       Tool X -5 mm")
    print("  y       Tool Y +5 mm")
    print("  Y       Tool Y -5 mm")
    print("  z       Tool Z +5 mm")
    print("  Z       Tool Z -5 mm")

    print()
    print("OTHER")
    print("  h       HOME")
    print("  s       STOP")
    print("  r       Record ON/OFF")
    print("  v       Save recording")
    print("  p       Playback")
    print("  i       Status")
    print("  ?       Help")
    print("  q       Quit")

    print()
    print("IMPORTANT:")
    print("  One key action = one robot command")
    print("  No dynamic physics")
    print("  No mj_step()")
    print("  No background command queue")

    print("=" * 70)


# ============================================================
# KEY PROCESSING
# ============================================================

def process_key(key):

    global running
    global selected_joint

    if key is None:

        return

    # ========================================================
    # QUIT
    # ========================================================

    if key == "q":

        stop_playback_internal()

        running = False

        return

    # ========================================================
    # HELP
    # ========================================================

    if key == "?":

        print_help()

        return

    # ========================================================
    # STATUS
    # ========================================================

    if key == "i":

        print_status()

        return

    # ========================================================
    # HOME
    # ========================================================

    if key == "h":

        go_home()

        return

    # ========================================================
    # STOP
    # ========================================================

    if key == "s":

        stop_motion()

        return

    # ========================================================
    # RECORD
    # ========================================================

    if key == "r":

        # Don't record while playback is active.
        if playback:

            print(
                "Stop playback before recording."
            )

            return

        toggle_recording()

        return

    # ========================================================
    # SAVE
    # ========================================================

    if key == "v":

        save_recording()

        return

    # ========================================================
    # PLAYBACK
    # ========================================================

    if key == "p":

        if playback:

            stop_playback_internal()

            print(
                "PLAYBACK STOPPED"
            )

        else:

            start_playback()

        return

    # ========================================================
    # MODE
    # ========================================================

    if key == "j":

        set_mode("JOINT")

        return

    if key == "w":

        set_mode("WORLD")

        return

    if key == "t":

        set_mode("TOOL")

        return

    # ========================================================
    # MANUAL COMMANDS ALWAYS STOP PLAYBACK
    # ========================================================

    if playback:

        stop_playback_internal()

        print(
            "PLAYBACK INTERRUPTED"
        )

    # ========================================================
    # JOINT SELECTION
    # ========================================================

    if key in "123456":

        selected_joint = (
            int(key) - 1
        )

        print(
            f"Selected joint: "
            f"J{selected_joint + 1}"
        )

        return

    # ========================================================
    # JOINT MODE
    # ========================================================

    if mode == "JOINT":

        if key in (
            "+",
            "="
        ):

            move_joint(
                selected_joint,
                +1
            )

            return

        if key == "-":

            move_joint(
                selected_joint,
                -1
            )

            return

    # ========================================================
    # WORLD MODE
    # ========================================================

    if mode == "WORLD":

        if key == "x":

            move_world(
                "x",
                +1
            )

            return

        if key == "X":

            move_world(
                "x",
                -1
            )

            return

        if key == "y":

            move_world(
                "y",
                +1
            )

            return

        if key == "Y":

            move_world(
                "y",
                -1
            )

            return

        if key == "z":

            move_world(
                "z",
                +1
            )

            return

        if key == "Z":

            move_world(
                "z",
                -1
            )

            return

    # ========================================================
    # TOOL MODE
    # ========================================================

    if mode == "TOOL":

        if key == "x":

            move_tool(
                "x",
                +1
            )

            return

        if key == "X":

            move_tool(
                "x",
                -1
            )

            return

        if key == "y":

            move_tool(
                "y",
                +1
            )

            return

        if key == "Y":

            move_tool(
                "y",
                -1
            )

            return

        if key == "z":

            move_tool(
                "z",
                +1
            )

            return

        if key == "Z":

            move_tool(
                "z",
                -1
            )

            return


# ============================================================
# MAIN
# ============================================================

def main():

    global model
    global data
    global running

    print()
    print("=" * 80)
    print(" ROBOTIC ARM — DIRECT KINEMATIC CONTROLLER")
    print("=" * 80)

    print(
        f"MuJoCo version : "
        f"{mujoco.__version__}"
    )

    print(
        f"Model          : "
        f"{MODEL_PATH}"
    )

    print()
    print(
        "Dynamic physics: DISABLED"
    )

    print(
        "Control        : qpos + mj_forward()"
    )

    print(
        "mj_step()      : NEVER USED"
    )

    print()

    # ========================================================
    # LOAD MODEL
    # ========================================================

    if not os.path.exists(
        MODEL_PATH
    ):

        raise FileNotFoundError(
            MODEL_PATH
        )

    model = (
        mujoco.MjModel.from_xml_path(
            MODEL_PATH
        )
    )

    data = mujoco.MjData(
        model
    )

    # ========================================================
    # BUILD MAPS
    # ========================================================

    build_joint_maps()

    # ========================================================
    # INITIAL FK
    # ========================================================

    forward()

    sync_target_to_tcp()

    # ========================================================
    # SHOW HELP
    # ========================================================

    print_help()

    # ========================================================
    # KEYBOARD
    # ========================================================

    keyboard = Keyboard()

    keyboard.start()

    try:

        with mujoco.viewer.launch_passive(
            model,
            data
        ) as viewer:

            while running:

                # ------------------------------------------------
                # ONE KEY
                # ------------------------------------------------

                key = keyboard.get_key()

                if key is not None:

                    process_key(key)

                # ------------------------------------------------
                # PLAYBACK
                # ------------------------------------------------

                if playback:

                    update_playback()

                # ------------------------------------------------
                # RECORD
                # ------------------------------------------------

                update_recording()

                # ------------------------------------------------
                # FORWARD KINEMATICS ONLY
                # ------------------------------------------------

                forward()

                # ------------------------------------------------
                # VIEWER
                # ------------------------------------------------

                viewer.sync()

                time.sleep(
                    LOOP_DT
                )

    finally:

        keyboard.stop()

        running = False

        print()
        print(
            "ROBOTIC ARM CONTROLLER EXITED"
        )


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":

    main()

