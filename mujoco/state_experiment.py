import mujoco

model = mujoco.MjModel.from_xml_string("""
<mujoco>

    <option gravity="0 0 -9.81"/>

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

print("START")
print("qpos:", data.qpos)
print("qvel:", data.qvel)

for i in range(100):

    mujoco.mj_step(model, data)

print("\nAFTER 100 STEPS")
print("time:", data.time)
print("qpos:", data.qpos)
print("qvel:", data.qvel)
