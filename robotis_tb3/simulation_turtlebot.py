import mujoco
import mujoco.viewer
from pathlib import Path

model_path = Path(__file__).parent / "sceneforturtle.xml"

model = mujoco.MjModel.from_xml_path(str(model_path))
data = mujoco.MjData(model)
data.ctrl[:] = [1.0,1.0]  # left wheel
step = 0
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        step += 1
        if step % 100 == 0:
            print(data.sensordata[0])  # enc_left
            print(data.sensordata[1])  # enc_right
            print(data.sensordata[2])  # vel_left
            print(data.sensordata[3])  # vel_right
        viewer.sync()