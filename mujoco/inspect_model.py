import mujoco

model = mujoco.MjModel.from_xml_string("""
<mujoco>

    <worldbody>

        <body name="box" pos="0 0 2">

            <freejoint/>

            <geom
                type="box"
                size="0.5 0.5 0.5"
                mass="1"/>

        </body>

        <geom
            type="plane"
            size="5 5 0.1"/>

    </worldbody>

</mujoco>
""")

data = mujoco.MjData(model)

print("Bodies    :", model.nbody)
print("Joints    :", model.njnt)
print("DOFs      :", model.nv)
print("qpos size :", model.nq)
print("Actuators :", model.nu)

print("\nInitial qpos:")
print(data.qpos)

print("\nInitial qvel:")
print(data.qvel)
