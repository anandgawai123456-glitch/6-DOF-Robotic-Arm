import mujoco
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_string("""
<mujoco>

    <option gravity="0 0 -9.81"/>

    <visual>
        <global azimuth="90" elevation="-20"/>
    </visual>

    <worldbody>

        <!-- BIG RED BOX -->
        <body name="box" pos="0 0 2">

            <freejoint/>

            <geom
                name="box_geom"
                type="box"
                size="0.5 0.5 0.5"
                mass="1"/>

        </body>

        <!-- GROUND -->
        <geom
            name="ground"
            type="plane"
            size="5 5 0.1"/>

    </worldbody>

</mujoco>
""")

data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        mujoco.mj_step(model, data)

        viewer.sync()

        time.sleep(0.002)
