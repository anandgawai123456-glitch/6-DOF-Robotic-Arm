import mujoco

print("MuJoCo version:", mujoco.__version__)

model = mujoco.MjModel.from_xml_string("""
<mujoco>
    <worldbody>

        <body name="box" pos="0 0 1">

            <freejoint/>

            <geom
                type="box"
                size="0.2 0.2 0.2"
                mass="1"/>

        </body>

        <geom
            type="plane"
            size="5 5 0.1"/>

    </worldbody>
</mujoco>
""")

data = mujoco.MjData(model)

print("Number of bodies:", model.nbody)
print("Number of joints:", model.njnt)
print("Number of actuators:", model.nu)

for i in range(100):
    mujoco.mj_step(model, data)

print("Simulation time:", data.time)
