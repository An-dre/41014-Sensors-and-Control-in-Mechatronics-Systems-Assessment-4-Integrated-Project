import mujoco
import numpy as np
from pathlib import Path
import model as tb3

scene = Path(__file__).parent.parent / "robotis_tb3" / "sceneforturtle.xml"
mj = mujoco.MjModel.from_xml_path(str(scene))
data = mujoco.MjData(mj)
dt = mj.opt.timestep

print("loaded, timestep = ", mj.opt.timestep)
data.ctrl[:] = [2.0, 4.0] #set both wheel to 3 rad/s 

pose_pred = np.zeros(3)
prev_left = 0.0
prev_right = 0.0

for step in range(5000):
    mujoco.mj_step(mj, data) #moves simulation forward one tick (0.002)
    
    left_angle = data.sensordata[0]
    right_angle = data.sensordata[1]
    w_left = (left_angle - prev_left) / dt
    w_right = (right_angle - prev_right) /dt
    prev_left = left_angle
    prev_right = right_angle
    
    v, omega = tb3.wheels_to_body(w_left, w_right)
    pose_pred = tb3.f(pose_pred, np.array([v, omega]), dt)
    
w, qx, qy, qz = data.qpos[3:7] #rotation stored as a quaternion 
yaw_true = np.arctan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)) #turns quaternion into heading angle 
pose_true = np.array([data.qpos[0], data.qpos[1], yaw_true]) #packs all together

print("true pose after 2 s:", pose_true)
print("model pose after 2 s", pose_pred)
print("error (model - true)", pose_pred - pose_true)
