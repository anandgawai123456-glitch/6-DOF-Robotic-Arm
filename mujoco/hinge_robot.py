import mujoco

model = mujoco.MjModel.from_xml_string("""
<mujoco>

    <option gravity="0 0 -9.81"/>

    <worldbody>

        <body name="base" pos="0 0 1">

            <geom
                type="box"
                size="0.3 0.3 0.1"
                mass="1"/>

            <body name="link" pos="0 0 -0.5">

                <joint
                    name="hinge"
                    type="hinge"
                    axis="0 1 0"/>

                <geom
                    type="capsule"
                    fromto="0 0 0 0 0 -1"
                    size="0.1"
                    mass="1"/>

            </body>

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

print("\nqpos:")
print(data.qpos)

print("\nqvel:")
print(data.qvel)
