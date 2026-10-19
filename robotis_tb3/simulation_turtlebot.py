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
noiselidar = 0.015 
scan =  data.sensordata[4:4+N]
angles = np.arange(N) * 2 * np.pi / N
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = True
    while viewer.is_running():
        mujoco.mj_step(model, data)
        step += 1
        
        scan_noisy = scan + np.random.normal(0, noiselidar, N)
        scan_noisy[scan < 0] = -1  
        
        if step % 100 == 0:
            print(f"Encoder Left: {round(data.sensordata[0], 3)}, Encoder Right: {round(data.sensordata[1], 3)}, Velocity Left: {round(data.sensordata[2], 3)}, Velocity Right: {round(data.sensordata[3], 3)}")
            print(f"front", round(scan[0],2), "left", round(scan[9],2),"back", round(scan[18],2), "right", round(scan[27],2))
            print("noisy front", round(scan_noisy[0],2), "left", round(scan_noisy[9],2), "back", round(scan_noisy[18],2), "right", round(scan_noisy[27],2))
        viewer.sync()
        