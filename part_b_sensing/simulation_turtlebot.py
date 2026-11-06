"""Early Part B sensor demo: watch the robot drive in the MuJoCo viewer and print its sensors.

Both wheels turn at 1 rad/s. Every 0.2 s it prints the encoder odometry next to the true
pose (for checking only), four LiDAR beams, and the marker range and bearing from the
camera. It needs a display, because it opens the MuJoCo viewer. Close the window to stop.

    python -m part_b_sensing.simulation_turtlebot

The encoder, odometry and range/bearing models follow the Week 2 tutorial. The pipeline
uses the Part B functions in sensing.py, not this script.
"""

import sys
from pathlib import Path

import cv2
import mujoco
import mujoco.viewer
import numpy as np

if __package__ in (None, ""):  # run as a file, not with -m: let Python find the part_* packages
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from part_e_integration import sim

wheel_r = 0.033   # from the waffle xml
wheel_sep = 0.288


# encoder from week 2 tut (cell 7), dynamixel is 4096 cpr
class WheelEncoder:
    def __init__(self, cpr=1024, noise_ticks=0.5, seed=0):
        self.cpr = cpr
        self.noise_ticks = noise_ticks
        self.rng = np.random.default_rng(seed)
        self.phi = 0.0
        self.count = 0

    @property
    def resolution(self):
        return 2 * np.pi / self.cpr

    def read(self, dphi_true):
        self.phi += dphi_true
        total = int(round(self.phi / self.resolution
                          + self.noise_ticks * self.rng.standard_normal()))
        dticks = total - self.count
        self.count = total
        return dticks * self.resolution


# odometry, week 2 cell 14. uses the midpoint heading (see B.0 in the tut)
def diff_drive_step(x, dphi_L, dphi_R, r, W):
    d = 0.5 * r * (dphi_L + dphi_R)
    dpsi = r * (dphi_R - dphi_L) / W
    return x + np.array([d * np.cos(x[2] + 0.5 * dpsi),
                         d * np.sin(x[2] + 0.5 * dpsi),
                         dpsi])


# range + bearing to a landmark, week 2 cell 31: sensor pose = robot pose composed with
# the sensor's mounting offset (x forward along the robot), then the landmark in that frame
def h_sensor(x, offset_x, p_W):
    th = x[2]
    sensor = np.array([x[0] + offset_x * np.cos(th), x[1] + offset_x * np.sin(th)])
    dx, dy = p_W - sensor
    p_S = np.array([np.cos(th) * dx + np.sin(th) * dy, -np.sin(th) * dx + np.cos(th) * dy])
    return np.array([np.hypot(*p_S), np.arctan2(p_S[1], p_S[0])])


CAM_OFFSET = 0.07                    # camera is 7 cm in front of the base origin
MARKER_POS = np.array([2.888, 0.0])  # front face of the dock plate

# K from fovy, no distortion in sim so this is exact
FOVY = 60
F = (480 / 2) / np.tan(np.radians(FOVY) / 2)
K = np.array([[F, 0, 320],
              [0, F, 240],
              [0, 0, 1]])

# plate is 0.2 m but the png has a white border, black square is only 200/250 of it
MARKER_SIDE = 0.2 * 200 / 250
_hs = MARKER_SIDE / 2
MARKER_CORNERS_3D = np.array([[-_hs, _hs, 0],
                              [_hs, _hs, 0],
                              [_hs, -_hs, 0],
                              [-_hs, -_hs, 0]], dtype=np.float32)
N_BEAMS = 36
LIDAR_NOISE = 0.015   # lds-01 datasheet says +-15 mm


def main():
    model, data = sim.load()      # the scene with the floor-contact correction, so odometry holds
    data.ctrl[:] = [1.0, 1.0]     # both wheels rad/s
    enc_left = WheelEncoder(cpr=4096, noise_ticks=0.5, seed=1)
    enc_right = WheelEncoder(cpr=4096, noise_ticks=0.5, seed=2)
    scan_address = model.sensor("scan0").adr[0]
    renderer = mujoco.Renderer(model, 480, 640)
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                       cv2.aruco.DetectorParameters())
    pose_odo = np.zeros(3)   # x y theta from encoders only, relative to the start
    start = data.qpos[:2].copy()
    prev_left, prev_right = data.sensordata[0], data.sensordata[1]
    step = 0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = True
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            step += 1

            # encoders -> odometry
            left_angle, right_angle = data.sensordata[0], data.sensordata[1]
            dleft = enc_left.read(left_angle - prev_left)
            dright = enc_right.read(right_angle - prev_right)
            prev_left, prev_right = left_angle, right_angle
            pose_odo = diff_drive_step(pose_odo, dleft, dright, wheel_r, wheel_sep)

            if step % 100:
                continue
            scan = data.sensordata[scan_address:scan_address + N_BEAMS]
            scan_noisy = scan + np.random.normal(0, LIDAR_NOISE, N_BEAMS)
            scan_noisy[scan < 0] = -1
            t = step * model.opt.timestep
            # ground truth, only for checking - nothing here feeds back into the robot
            w, qx, qy, qz = data.qpos[3:7]   # mujoco quat is w x y z
            yaw_true = np.arctan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            pose_true = np.array([data.qpos[0], data.qpos[1], yaw_true])

            print(f"\n--- t = {t:5.1f} s ---")
            print(f"pose   odom  x {start[0] + pose_odo[0]:6.3f}  y {start[1] + pose_odo[1]:6.3f}"
                  f"  th {np.degrees(pose_odo[2]):6.1f} deg")
            print(f"       true  x {pose_true[0]:6.3f}  y {pose_true[1]:6.3f}  th {np.degrees(pose_true[2]):6.1f} deg")
            print(f"lidar  front {scan_noisy[0]:5.2f}  left {scan_noisy[9]:5.2f}  back {scan_noisy[18]:5.2f}"
                  f"  right {scan_noisy[27]:5.2f}  (m, -1 = nothing)")

            renderer.update_scene(data, camera="rgb")
            gray = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is not None:
                ok, rvec, tvec = cv2.solvePnP(MARKER_CORNERS_3D, corners[0].reshape(4, 2), K, None)
                tx, ty, tz = tvec.ravel()
                marker_range = np.hypot(tz, tx)
                marker_bearing = np.arctan2(-tx, tz)   # cam x is right, robot y is left
                expected = h_sensor(pose_true, CAM_OFFSET, MARKER_POS)
                print(f"marker {np.ravel(ids)[0]}: range {marker_range:.3f} (true {expected[0]:.3f})"
                      f" bearing {np.degrees(marker_bearing):.1f} (true {np.degrees(expected[1]):.1f})")
            else:
                print("no marker")
    renderer.close()


if __name__ == "__main__":
    main()
