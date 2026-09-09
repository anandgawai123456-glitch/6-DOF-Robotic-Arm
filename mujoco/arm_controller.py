import time
import sys
import select
import termios
import tty

import numpy as np
import mujoco
import mujoco.viewer


# ============================================================
# CUSTOM 6-DOF ROBOTIC ARM
# INTERACTIVE KINEMATIC CONTROLLER
#
# MODES
#
#   j = JOINT MODE
#   w = WORLD XYZ MODE
#   t = TOOL XYZ MODE
#
#
# JOINT MODE
#
#   1-6   = select joint
#   +     = positive joint movement
#   -     = negative joint movement
#
#
# WORLD MODE
#
#   x = +X
#   X = -X
#   y = +Y
#   Y = -Y
#   z = +Z
#   Z = -Z
#
#
# TOOL MODE
#
#   x = tool +X
#   X = tool -X
#   y = tool +Y
#   Y = tool -Y
#   z = tool +Z
#   Z = tool -Z
#
#
# OTHER
#
#   h = HOME
#   q = EXIT
#
#
# Cartesian control:
#
#   Cartesian target
#          ↓
#   Position error
#          ↓
#   MuJoCo Jacobian
#          ↓
#   Damped Least Squares IK
#          ↓
#   Bounded joint update
#
# IMPORTANT:
#
# This is a KINEMATIC controller.
# It directly changes qpos.
#
# It is NOT a torque/actuator controller.
# ============================================================


# ============================================================
# CONFIGURATION
# ============================================================

MODEL_PATH = "mujoco/robotic_arm.xml"

# Main controller loop
DT = 0.02


# ------------------------------------------------------------
# Cartesian movement
#
# 5 mm per keyboard command
# ------------------------------------------------------------

XYZ_STEP = 0.005


# ------------------------------------------------------------
# Joint movement
#
# 5 degrees per keyboard command
# ------------------------------------------------------------

JOINT_STEP = np.deg2rad(5.0)


# ------------------------------------------------------------
# IK parameters
# ------------------------------------------------------------

DAMPING = 0.15

# Fraction of the calculated IK update applied
IK_STEP = 0.15

# Number of internal IK iterations per update
IK_ITERATIONS = 12

# Maximum movement of ANY joint during one controller update
MAX_JOINT_STEP = np.deg2rad(2.0)


# ------------------------------------------------------------
# Position convergence
# ------------------------------------------------------------

POSITION_TOLERANCE = 1e-4


# ------------------------------------------------------------
# Joint names
# ------------------------------------------------------------

JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]


# ============================================================
# KEYBOARD
# ============================================================

class Keyboard:

    def __init__(self):

        self.fd = sys.stdin.fileno()

        self.old_settings = termios.tcgetattr(
            self.fd
        )

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

        if readable:

            return sys.stdin.read(1)

        return None


# ============================================================
# ARM CONTROLLER
# ============================================================

class ArmController:

    def __init__(self, model, data):

        self.model = model
        self.data = data

        # ----------------------------------------------------
        # Find TCP site
        # ----------------------------------------------------

        self.tcp_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            "tcp"
        )

        if self.tcp_id < 0:

            raise RuntimeError(
                "TCP site 'tcp' was not found."
            )

        # ----------------------------------------------------
        # Mode
        # ----------------------------------------------------

        self.mode = "WORLD"

        # ----------------------------------------------------
        # Selected joint
        # ----------------------------------------------------

        self.selected_joint = 0

        # ----------------------------------------------------
        # Home position
        # ----------------------------------------------------

        self.home = np.zeros(6)

        # ----------------------------------------------------
        # Initial FK
        # ----------------------------------------------------

        mujoco.mj_forward(
            self.model,
            self.data
        )

        # ----------------------------------------------------
        # Cartesian target
        #
        # IMPORTANT:
        #
        # Target always starts exactly at the
        # current TCP.
        # ----------------------------------------------------

        self.target_position = (
            self.data.site_xpos[
                self.tcp_id
            ].copy()
        )

        # ----------------------------------------------------
        # Previous target
        # ----------------------------------------------------

        self.last_target_position = (
            self.target_position.copy()
        )

    # ========================================================
    # TCP POSITION
    # ========================================================

    def get_tcp_position(self):

        mujoco.mj_forward(
            self.model,
            self.data
        )

        return self.data.site_xpos[
            self.tcp_id
        ].copy()

    # ========================================================
    # TCP ROTATION
    # ========================================================

    def get_tcp_rotation(self):

        mujoco.mj_forward(
            self.model,
            self.data
        )

        return self.data.site_xmat[
            self.tcp_id
        ].reshape(3, 3).copy()

    # ========================================================
    # POSITION JACOBIAN
    # ========================================================

    def get_position_jacobian(self):

        jac_pos = np.zeros(
            (3, self.model.nv)
        )

        jac_rot = np.zeros(
            (3, self.model.nv)
        )

        mujoco.mj_jacSite(
            self.model,
            self.data,
            jac_pos,
            jac_rot,
            self.tcp_id
        )

        # Only the first six DOFs belong to our arm.
        return jac_pos[:, :6]

    # ========================================================
    # JOINT LIMITS
    # ========================================================

    def apply_joint_limits(self):

        for i in range(6):

            low = self.model.jnt_range[
                i, 0
            ]

            high = self.model.jnt_range[
                i, 1
            ]

            self.data.qpos[i] = np.clip(
                self.data.qpos[i],
                low,
                high
            )

    # ========================================================
    # NUMERICAL IK
    # ========================================================

    def solve_ik(self, target):

        target = np.asarray(
            target,
            dtype=float
        )

        for _ in range(IK_ITERATIONS):

            # ------------------------------------------------
            # Forward kinematics
            # ------------------------------------------------

            mujoco.mj_forward(
                self.model,
                self.data
            )

            # ------------------------------------------------
            # Current TCP
            # ------------------------------------------------

            current = (
                self.data.site_xpos[
                    self.tcp_id
                ].copy()
            )

            # ------------------------------------------------
            # Cartesian error
            # ------------------------------------------------

            error = target - current

            error_norm = np.linalg.norm(
                error
            )

            # ------------------------------------------------
            # Already close enough
            # ------------------------------------------------

            if error_norm < POSITION_TOLERANCE:

                break

            # ------------------------------------------------
            # Jacobian
            # ------------------------------------------------

            J = self.get_position_jacobian()

            # ------------------------------------------------
            # Damped Least Squares
            #
            # dq =
            #
            # J^T (J J^T + λ²I)^-1 e
            # ------------------------------------------------

            A = (
                J @ J.T
                +
                DAMPING**2
                *
                np.eye(3)
            )

            try:

                dq = (
                    J.T
                    @
                    np.linalg.solve(
                        A,
                        error
                    )
                )

            except np.linalg.LinAlgError:

                print(
                    "\nIK matrix solve failed."
                )

                break

            # ------------------------------------------------
            # Scale IK update
            # ------------------------------------------------

            dq *= IK_STEP

            # ------------------------------------------------
            # CRITICAL SAFETY LIMIT
            #
            # Never allow one joint to move more than
            # MAX_JOINT_STEP in one update.
            # ------------------------------------------------

            largest_step = np.max(
                np.abs(dq)
            )

            if largest_step > MAX_JOINT_STEP:

                dq *= (
                    MAX_JOINT_STEP
                    /
                    largest_step
                )

            # ------------------------------------------------
            # Apply joint update
            # ------------------------------------------------

            self.data.qpos[:6] += dq

            # ------------------------------------------------
            # Apply joint limits
            # ------------------------------------------------

            self.apply_joint_limits()

            # ------------------------------------------------
            # Update FK
            # ------------------------------------------------

            mujoco.mj_forward(
                self.model,
                self.data
            )

        return self.data.qpos[:6].copy()

    # ========================================================
    # MOVE WORLD
    # ========================================================

    def move_world(
        self,
        axis,
        direction
    ):

        self.target_position[axis] += (
            direction * XYZ_STEP
        )

    # ========================================================
    # MOVE TOOL
    # ========================================================

    def move_tool(
        self,
        axis,
        direction
    ):

        # Current TCP orientation
        R = self.get_tcp_rotation()

        # Movement in TCP local coordinates
        local_delta = np.zeros(3)

        local_delta[axis] = (
            direction * XYZ_STEP
        )

        # Convert local movement to world frame
        world_delta = (
            R @ local_delta
        )

        self.target_position += (
            world_delta
        )

    # ========================================================
    # JOINT CONTROL
    # ========================================================

    def move_joint(self, direction):

        i = self.selected_joint

        # ----------------------------------------------------
        # Direct joint movement
        # ----------------------------------------------------

        self.data.qpos[i] += (
            direction * JOINT_STEP
        )

        # ----------------------------------------------------
        # Joint limits
        # ----------------------------------------------------

        self.apply_joint_limits()

        # ----------------------------------------------------
        # FK
        # ----------------------------------------------------

        mujoco.mj_forward(
            self.model,
            self.data
        )

        # ----------------------------------------------------
        # IMPORTANT:
        #
        # After manual joint movement, reset the Cartesian
        # target to the NEW TCP.
        #
        # Otherwise WORLD mode would try to chase an old
        # target and the robot could jump.
        # ----------------------------------------------------

        self.sync_target_to_tcp()

    # ========================================================
    # SYNCHRONIZE CARTESIAN TARGET
    # ========================================================

    def sync_target_to_tcp(self):

        mujoco.mj_forward(
            self.model,
            self.data
        )

        self.target_position = (
            self.data.site_xpos[
                self.tcp_id
            ].copy()
        )

        self.last_target_position = (
            self.target_position.copy()
        )

    # ========================================================
    # HOME
    # ========================================================

    def go_home(self):

        self.data.qpos[:6] = (
            self.home
        )

        self.apply_joint_limits()

        mujoco.mj_forward(
            self.model,
            self.data
        )

        self.sync_target_to_tcp()

        print(
            "\nHOME position."
        )

    # ========================================================
    # CHANGE MODE
    # ========================================================

    def set_mode(self, mode):

        self.mode = mode

        # ----------------------------------------------------
        # Always synchronize target when entering Cartesian
        # control.
        # ----------------------------------------------------

        if mode in [
            "WORLD",
            "TOOL"
        ]:

            self.sync_target_to_tcp()

        print(
            f"\nMODE: {self.mode}"
        )

    # ========================================================
    # PROCESS KEY
    # ========================================================

    def process_key(self, key):

        # ----------------------------------------------------
        # EXIT
        # ----------------------------------------------------

        if key == "q":

            return False

        # ----------------------------------------------------
        # MODE
        # ----------------------------------------------------

        if key == "j":

            self.set_mode(
                "JOINT"
            )

            return True

        if key == "w":

            self.set_mode(
                "WORLD"
            )

            return True

        if key == "t":

            self.set_mode(
                "TOOL"
            )

            return True

        # ----------------------------------------------------
        # HOME
        # ----------------------------------------------------

        if key == "h":

            self.go_home()

            return True

        # ----------------------------------------------------
        # JOINT MODE
        # ----------------------------------------------------

        if self.mode == "JOINT":

            # Select joint
            if key in "123456":

                self.selected_joint = (
                    int(key) - 1
                )

                print(
                    "\nSelected joint:",
                    JOINT_NAMES[
                        self.selected_joint
                    ]
                )

            # Positive
            elif key == "+":

                self.move_joint(
                    +1
                )

            # Negative
            elif key == "-":

                self.move_joint(
                    -1
                )

        # ----------------------------------------------------
        # WORLD MODE
        # ----------------------------------------------------

        elif self.mode == "WORLD":

            if key == "x":

                self.move_world(
                    0,
                    +1
                )

            elif key == "X":

                self.move_world(
                    0,
                    -1
                )

            elif key == "y":

                self.move_world(
                    1,
                    +1
                )

            elif key == "Y":

                self.move_world(
                    1,
                    -1
                )

            elif key == "z":

                self.move_world(
                    2,
                    +1
                )

            elif key == "Z":

                self.move_world(
                    2,
                    -1
                )

        # ----------------------------------------------------
        # TOOL MODE
        # ----------------------------------------------------

        elif self.mode == "TOOL":

            if key == "x":

                self.move_tool(
                    0,
                    +1
                )

            elif key == "X":

                self.move_tool(
                    0,
                    -1
                )

            elif key == "y":

                self.move_tool(
                    1,
                    +1
                )

            elif key == "Y":

                self.move_tool(
                    1,
                    -1
                )

            elif key == "z":

                self.move_tool(
                    2,
                    +1
                )

            elif key == "Z":

                self.move_tool(
                    2,
                    -1
                )

        return True

    # ========================================================
    # STATUS
    # ========================================================

    def print_status(self):

        tcp = self.get_tcp_position()

        target_error = (
            self.target_position
            -
            tcp
        )

        error_mm = (
            np.linalg.norm(
                target_error
            )
            * 1000.0
        )

        print(
            f"\r"
            f"Mode: {self.mode:5s} | "
            f"Joint: {self.selected_joint + 1} | "
            f"TCP: "
            f"X={tcp[0]:+.3f} "
            f"Y={tcp[1]:+.3f} "
            f"Z={tcp[2]:+.3f} | "
            f"Err={error_mm:6.2f} mm",
            end="",
            flush=True
        )


# ============================================================
# MAIN
# ============================================================

def main():

    print(
        "=" * 80
    )

    print(
        " CUSTOM 6-DOF ROBOTIC ARM"
    )

    print(
        " STABLE KINEMATIC CONTROLLER"
    )

    print(
        "=" * 80
    )

    # --------------------------------------------------------
    # Load MuJoCo model
    # --------------------------------------------------------

    model = mujoco.MjModel.from_xml_path(
        MODEL_PATH
    )

    data = mujoco.MjData(
        model
    )

    print()
    print(
        "Model loaded."
    )

    print(
        "Bodies :",
        model.nbody
    )

    print(
        "Joints:",
        model.njnt
    )

    print(
        "DOFs   :",
        model.nv
    )

    # --------------------------------------------------------
    # Controller
    # --------------------------------------------------------

    controller = ArmController(
        model,
        data
    )

    # --------------------------------------------------------
    # Controls
    # --------------------------------------------------------

    print()
    print(
        "CONTROLS"
    )

    print(
        "-" * 80
    )

    print(
        "j : JOINT MODE"
    )

    print(
        "w : WORLD XYZ MODE"
    )

    print(
        "t : TOOL XYZ MODE"
    )

    print()

    print(
        "JOINT MODE"
    )

    print(
        "1-6   : select joint"
    )

    print(
        "+ / - : move selected joint"
    )

    print()

    print(
        "WORLD / TOOL MODE"
    )

    print(
        "x / X : +X / -X"
    )

    print(
        "y / Y : +Y / -Y"
    )

    print(
        "z / Z : +Z / -Z"
    )

    print()

    print(
        "h : HOME"
    )

    print(
        "q : EXIT"
    )

    print()

    print(
        "Cartesian step:",
        XYZ_STEP * 1000,
        "mm"
    )

    print(
        "Maximum IK joint step:",
        np.rad2deg(MAX_JOINT_STEP),
        "deg"
    )

    print()

    print(
        "Starting MuJoCo viewer..."
    )

    # --------------------------------------------------------
    # Keyboard
    # --------------------------------------------------------

    keyboard = Keyboard()

    # --------------------------------------------------------
    # Viewer
    # --------------------------------------------------------

    with mujoco.viewer.launch_passive(
        model,
        data
    ) as viewer:

        keyboard.start()

        try:

            while viewer.is_running():

                # ------------------------------------------------
                # Keyboard
                # ------------------------------------------------

                key = keyboard.get_key()

                if key is not None:

                    running = (
                        controller.process_key(
                            key
                        )
                    )

                    if not running:

                        break

                # ------------------------------------------------
                # Cartesian IK
                #
                # Only run when target differs from TCP.
                # ------------------------------------------------

                if controller.mode in [
                    "WORLD",
                    "TOOL"
                ]:

                    tcp = (
                        controller.get_tcp_position()
                    )

                    target_error = (
                        controller.target_position
                        -
                        tcp
                    )

                    # Only solve IK when there is an
                    # actual Cartesian target error.
                    if np.linalg.norm(
                        target_error
                    ) > POSITION_TOLERANCE:

                        controller.solve_ik(
                            controller.target_position
                        )

                # ------------------------------------------------
                # Status
                # ------------------------------------------------

                controller.print_status()

                # ------------------------------------------------
                # Viewer
                # ------------------------------------------------

                viewer.sync()

                # ------------------------------------------------
                # Controller rate
                # ------------------------------------------------

                time.sleep(
                    DT
                )

        finally:

            keyboard.stop()

    print()
    print()

    print(
        "Controller stopped."
    )


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":

    main()
