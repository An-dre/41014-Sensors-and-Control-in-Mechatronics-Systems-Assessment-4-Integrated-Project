import mujoco
import mujoco.viewer
from pathlib import Path
import numpy as np

model_path = Path(__file__).parent / "sceneforturtle.xml"

model = mujoco.MjModel.from_xml_path(str(model_path))
data = mujoco.MjData(model)
data.ctrl[:] = [1.0,1.0]  # both wheels rad/s
step = 0
N = 36

scan =  data.sensordata[4:4+N]
angles = np.arange(N) * 2 * np.pi / N
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        step += 1
        if step % 100 == 0:
            print(f"Encoder Left: {round(data.sensordata[0], 3)}, Encoder Right: {round(data.sensordata[1], 3)}, Velocity Left: {round(data.sensordata[2], 3)}, Velocity Right: {round(data.sensordata[3], 3)}")
            print(f"front", round(scan[0],2), "left", round(scan[9],2),"back", round(scan[18],2), "right", round(scan[27],2))
        viewer.sync()
        