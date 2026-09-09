import mujoco
import numpy as np


MODEL_PATH = "mujoco/robotic_arm.xml"


model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)


def print_fk(joints):
    data.qpos[:] = joints

    mujoco.mj_forward(model, data)

    tool_id = 17

    position = data.xpos[tool_id].copy()
    rotation = data.xmat[tool_id].reshape(3, 3).copy()

    print()
    print("Joint angles:")
    print(np.array(joints))

    print()
    print("Tool position:")
    print(
        f"x = {position[0]: .6f}, "
        f"y = {position[1]: .6f}, "
        f"z = {position[2]: .6f}"
    )

    print()
    print("Tool orientation:")
    print(rotation)

    print()
    print("----------------------------------------")


print()
print("============================================================")
print(" ROBOTIC ARM FORWARD KINEMATICS TEST")
print("============================================================")


# Test 1: Zero configuration
print("TEST 1: ZERO CONFIGURATION")
print_fk([
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
])


# Test 2: Joint 1 = +90 degrees
print("TEST 2: JOINT 1 = +90 DEGREES")
print_fk([
    np.pi / 2,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
])


# Test 3: Joint 2 = +90 degrees
print("TEST 3: JOINT 2 = +90 DEGREES")
print_fk([
    0.0,
    np.pi / 2,
    0.0,
    0.0,
    0.0,
    0.0
])


# Test 4: A combined configuration
print("TEST 4: COMBINED CONFIGURATION")
print_fk([
    0.3,
    0.4,
    -0.5,
    0.6,
    0.2,
    -0.3
])
