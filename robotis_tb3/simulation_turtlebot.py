import mujoco
import mujoco.viewer
from pathlib import Path
import numpy as np
import cv2
from spatialmath import SE2


model_path = Path(__file__).parent / "sceneforturtle.xml"
model = mujoco.MjModel.from_xml_path(str(model_path))
data = mujoco.MjData(model)
data.ctrl[:] = [1.0, 1.0]  # both wheels rad/s
step = 0

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

enc_left = WheelEncoder(cpr=4096, noise_ticks=0.5, seed=1)
enc_right = WheelEncoder(cpr=4096, noise_ticks=0.5, seed=2)

# odometry, week 2 cell 14. uses the midpoint heading (see B.0 in the tut)
def diff_drive_step(x, dphi_L, dphi_R, r, W):
    d = 0.5 * r * (dphi_L + dphi_R)
    dpsi = r * (dphi_R - dphi_L) / W
    return x + np.array([d * np.cos(x[2] + 0.5 * dpsi),
                         d * np.sin(x[2] + 0.5 * dpsi),
                         dpsi])

pose_odo = np.zeros(3)   # x y theta from encoders only
prev_left = 0.0
prev_right = 0.0

# range + bearing to a landmark, week 2 cell 31
def h_lidar(x, T_BS, p_W, sigma_rho=0.02, sigma_alpha=np.deg2rad(0.25), rng=None):
    T_WS = SE2(x[0], x[1], x[2]) * T_BS
    p_S = (T_WS.inv() * p_W.reshape(2, 1)).flatten()
    rho, alpha = np.hypot(*p_S), np.arctan2(p_S[1], p_S[0])
    if rng is not None:
        rho += sigma_rho * rng.standard_normal()
        alpha += sigma_alpha * rng.standard_normal()
    return np.array([rho, alpha])

cam_offset = SE2(0.07, 0.0, 0.0)     # camera is 7cm in front of base
lidar_offset = SE2(-0.049, 0.0, 0.0)
marker_pos = np.array([2.888, 0.0])  # front face of the dock plate

# lidar
N = 36
noiselidar = 0.015   # lds-01 datasheet says +-15mm
scan = data.sensordata[4:4 + N]
angles = np.arange(N) * 2 * np.pi / N

# camera
renderer = mujoco.Renderer(model, 480, 640)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

# K from fovy, no distortion in sim so this is exact
fovy = 60
f = (480 / 2) / np.tan(np.radians(fovy) / 2)
K = np.array([[f, 0, 320],
              [0, f, 240],
              [0, 0, 1]])

# plate is 0.2m but the png has a white border, black square is only 200/250 of it
marker_side = 0.2 * 200 / 250
hs = marker_side / 2
marker_corners_3d = np.array([[-hs, hs, 0],
                              [hs, hs, 0],
                              [hs, -hs, 0],
                              [-hs, -hs, 0]], dtype=np.float32)

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = True
    while viewer.is_running():
        mujoco.mj_step(model, data)
        step += 1

        # encoders -> odometry
        left_angle = data.sensordata[0]
        right_angle = data.sensordata[1]
        dleft = enc_left.read(left_angle - prev_left)
        dright = enc_right.read(right_angle - prev_right)
        prev_left = left_angle
        prev_right = right_angle
        pose_odo = diff_drive_step(pose_odo, dleft, dright, wheel_r, wheel_sep)

        scan_noisy = scan + np.random.normal(0, noiselidar, N)
        scan_noisy[scan < 0] = -1

        if step % 100 == 0:
            t = step * model.opt.timestep
            # ground truth, only for checking - controller never sees this
            w, qx, qy, qz = data.qpos[3:7]   # mujoco quat is w x y z
            yaw_true = np.arctan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            pose_true = np.array([data.qpos[0], data.qpos[1], yaw_true])

            print(f"\n--- t = {t:5.1f} s ---")
            print(f"pose   odom  x {pose_odo[0]:6.3f}  y {pose_odo[1]:6.3f}  th {np.degrees(pose_odo[2]):6.1f} deg")
            print(f"       true  x {pose_true[0]:6.3f}  y {pose_true[1]:6.3f}  th {np.degrees(pose_true[2]):6.1f} deg")
            print(f"lidar  front {scan_noisy[0]:5.2f}  left {scan_noisy[9]:5.2f}  back {scan_noisy[18]:5.2f}  right {scan_noisy[27]:5.2f}  (m, -1 = nothing)")
            # print(dleft, dright)

            renderer.update_scene(data, camera="rgb")
            img = renderer.render()
            cv2.imwrite(str(Path(__file__).parent / "frame.png"), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            if ids is not None:
                ok, rvec, tvec = cv2.solvePnP(marker_corners_3d, corners[0][0], K, None)
                tx, ty, tz = tvec.ravel()
                marker_range = np.hypot(tz, tx)
                marker_bearing = np.arctan2(-tx, tz)   # cam x is right, robot y is left

                expected = h_lidar(pose_true, cam_offset, marker_pos)
                print(f"marker {ids.flatten()[0]}: range {marker_range:.3f} (true {expected[0]:.3f})"
                      f" bearing {np.degrees(marker_bearing):.1f} (true {np.degrees(expected[1]):.1f})")
            else:
                print("no marker")

        viewer.sync()