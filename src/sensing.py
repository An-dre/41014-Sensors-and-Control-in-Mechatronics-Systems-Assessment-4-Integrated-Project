import model
import numpy as np
import cv2
import sim
import sys
import mujoco
from pathlib import Path

# B1: encoder noise 
# A real encoder gives integer ticks. The TB3's Dynamixel gives 4096 ticks per turn.
# Steps: angle -> ticks (round to an integer) -> back to an angle, plus a small
# Gaussian noise (slip, backlash). Decide the noise size and WRITE WHY on the page.

TICKS_PER_REV = 4096
TICK = 2 * np.pi / TICKS_PER_REV
ENC_NOISE_RAD = TICK

# noisy_encoders takes two inputs: the left and right wheel angles from the sim, in radians
def noisy_encoders(enc_left, enc_right): 
    # Converts the left wheel angle into a whole number of ticks
    left_ticks = np.round(enc_left / TICK)
    # Converts back to radians and adds a small random error
    # np.random.normal picks a random number from a bell curve. 0 means it’s centred on zero,
    # no bias, equally likely to be + or −
    left = left_ticks * TICK + np.random.normal(0, ENC_NOISE_RAD)

    right_ticks = np.round(enc_right / TICK)
    right = right_ticks * TICK + np.random.normal(0, ENC_NOISE_RAD)

    return left, right

# wheel angle change -> wheel speeds -> (v, omega) using model.wheels_to_body.
# This (v, omega) is the EKF's input u, so it carries the encoder noise with it.

# odometry takes 3 inputs: the current and previous encoder readings (in radians) and the time step dt (0.02s).
# enc_now: the wheel angles this loop, as a pair [left (0), right (1)] -> [1.08, 1.12]
# enc_prev: the wheel angles last loop -> [1.00, 1.00]
def odometry(enc_now, enc_prev, dt):
    # Works out how fast the left wheel is spinning
    w_left = (enc_now[0] - enc_prev[0]) / dt
    # Works out how fast the right wheel is spinning
    w_right = (enc_now[1] - enc_prev[1]) / dt
    # Converts the two wheel speeds into the robot’s speed. This uses P4’s function from A1
    v, omega = model.wheels_to_body(w_left, w_right)
    # The EKF needs this in a NumPy array because it’s the input vector u for the prediction step
    return np.array([v, omega])
   

# B2: lidar noise 
# The LDS-01 datasheet quotes +-15 mm accuracy for 0.12-0.5 m and +-5 % for 0.5-3.5 m.
# Turn that into a std that grows with range, and say how you converted it.
# Also replace "-1" (no return) with the max range so the planner treats it as free space.

# real limits of the TurtleBot3’s lidar
LIDAR_MIN = 0.12
LIDAR_MAX = 3.5
 
def noisy_lidar(ranges):
    # Copy the 360 beam distances into a NumPy array so we can do maths on all of them at once
    r = np.array(ranges, dtype=float)
    # Remember which beams got no return (-1) BEFORE adding noise
    no_return = r < 0
    # Noise size per beam: 7.5 mm for short beams, 2.5% of the distance for long beams
    sigma = np.where(r < 0.5, 0.015 / 2, 0.05 * r / 2)
    # Add Gaussian noise: a standard bell-curve number per beam, scaled by that beam's sigma
    r = r + np.random.normal(0, 1, r.shape) * sigma
    # Keep every reading inside the sensor's real range (0.12-3.5 m)
    r = np.clip(r, LIDAR_MIN, LIDAR_MAX)
    # Beams with no return become max range, so the planner treats them as free space
    r[no_return] = LIDAR_MAX
    return r

# B3: camera model
# The sim camera is a pinhole with a vertical field of view fovy. From the image
# geometry, focal length in pixels is
#     f = (H / 2) / tan(fovy / 2)
# and the principal point is the image centre. This is the "true" K. Your
# calibration (B4) should land close to it, and the difference is a result to report.
def camera_matrix():
    H = sim.CAM_H
    W = sim.CAM_W
    f = (H / 2) / np.tan(np.radians(sim.CAM_FOVY_DEG) / 2)
    K = np.array([[f, 0, W / 2],
                  [0, f, H / 2],
                  [0, 0, 1]])
    return K

# B4: calibration
# sceneforturtle.xml has no chessboard, so B4 uses calib_scene.xml: the same robot and
# camera plus an 8 x 6 board of 0.1 m squares (7 x 5 inner corners) at (2.0, 0, 0.35).
# K belongs to the camera, not the room, so the result is valid for the real scene.
# Run:  python sensing.py calib   -> prints calibrated vs true K, saves calib.npz

CHESS_PATTERN = (7, 5)                                     # inner corners (8 x 6 squares)
CHESS_SQUARE = 0.1                                         # square size in metres
CALIB_SCENE = Path(__file__).resolve().parent.parent / "robotis_tb3" / "calib_scene.xml"    # robot + chessboard    # robot + chessboard
BOARD_CENTRE = np.array([2.0, 0.0])                        # board centre on the floor plane


def collect_calibration_images():
    # Park the robot at 27 spots facing the board: 3 distances x 3 sideways x 3 turns
    m = mujoco.MjModel.from_xml_path(str(CALIB_SCENE))
    d = mujoco.MjData(m)
    renderer, opt = sim.make_renderer(m)
    images = []
    for dist_ in (1.0, 1.25, 1.5):                 # metres from the board
        for side in (-0.35, 0.0, 0.35):            # left / centre / right of the board
            for dth in (-0.25, 0.0, 0.25):         # turn a little left / straight / right (rad)
                x, y = BOARD_CENTRE[0] - dist_, BOARD_CENTRE[1] + side
                theta = np.arctan2(BOARD_CENTRE[1] - y, BOARD_CENTRE[0] - x) + dth
                sim.set_pose(m, d, x, y, theta)
                images.append(sim.read_camera(renderer, opt, d))
    return images


def calibrate(images):
    # 3D corner positions on the board itself (z = 0, flat), in metres: (0,0), (0.1,0), ...
    objp = np.zeros((CHESS_PATTERN[0] * CHESS_PATTERN[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESS_PATTERN[0], 0:CHESS_PATTERN[1]].T.reshape(-1, 2) * CHESS_SQUARE

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    obj_points, img_points = [], []
    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        ok, corners = cv2.findChessboardCorners(gray, CHESS_PATTERN)
        if not ok:
            continue                                   # board not fully visible in this photo
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)   # refine to sub-pixel
        obj_points.append(objp)
        img_points.append(corners)

    if len(obj_points) < 3:
        print("calibrate: only", len(obj_points), "usable photos - check the board is in view")
        return None

    H, W = images[0].shape[:2]
    rms, K, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, (W, H), None, None)
    return K, dist, rms, len(obj_points)


def run_calibration():
    out = calibrate(collect_calibration_images())
    if out is None:
        return
    K_cal, dist, rms, n = out
    print(f"used {n} photos, reprojection error {rms:.3f} px")
    print("true K:\n", camera_matrix())
    print("calibrated K:\n", np.round(K_cal, 2))
    np.savez(Path(__file__).parent / "calib.npz", K=K_cal, dist=dist)


# B5: ArUco detection and pose
# Finds marker id 0 (DICT_4X4_50) and uses solvePnP (IPPE_SQUARE) to get its
# position in the OpenCV camera frame (x right, y down, z forward).
# Rejects wrong id, tiny markers (< MIN_SIDE_PX), and impossible ranges.
MARKER_SIZE = 0.2
MARKER_ID = 0
MIN_SIDE_PX = 30      # reject markers smaller than this in the image (~2.2 m away for the 0.16 m marker)
MAX_RANGE = 3.5       # reject poses further than this (impossible given MIN_SIDE_PX)
 
# The 4 marker corners in the MARKER frame (centre at 0, flat so z = 0), in the order
# OpenCV gives them: top-left, top-right, bottom-right, bottom-left
_h = MARKER_SIZE / 2
MARKER_CORNERS_3D = np.array([[-_h,  _h, 0],
                              [ _h,  _h, 0],
                              [ _h, -_h, 0],
                              [-_h, -_h, 0]], dtype=np.float32)
 
# Make the detector once, not every frame
_DETECTOR = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
 
# Count why detections were thrown away (report these on the page)
REJECTS = {"wrong_id": 0, "too_small": 0, "pnp_failed": 0, "bad_range": 0}
 
 
def load_calibration():
    # Use the calibrated K from B4 if it exists, otherwise the true K from B3
    try:
        c = np.load(Path(__file__).parent / "calib.npz")
        return c["K"], c["dist"]
    except FileNotFoundError:
        return camera_matrix(), None
 
 
def detect_marker(img, K, dist=None):
    # returns (seen, tvec, rvec). seen = False when the marker is not in view.
    if dist is None:
        dist = np.zeros(5)                     # sim camera has no lens distortion
 
    # 1. colour photo -> greyscale (the detector only needs light/dark)
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
 
    # 2. find any ArUco markers: corners = pixel corners of each, ids = their numbers
    corners, ids, _ = _DETECTOR.detectMarkers(gray)
    if ids is None:
        return False, None, None               # nothing in view
 
    # 3. keep only marker id 0 (np.ravel handles OpenCV 4 vs 5 shapes)
    ids = np.ravel(ids)
    matches = np.where(ids == MARKER_ID)[0]
    if len(matches) == 0:
        REJECTS["wrong_id"] += 1
        return False, None, None
    c = corners[matches[0]].reshape(4, 2).astype(np.float32)
 
    # 4. size check: shortest side in pixels. Tiny markers give garbage poses.
    sides = np.linalg.norm(c - np.roll(c, 1, axis=0), axis=1)
    if sides.min() < MIN_SIDE_PX:
        REJECTS["too_small"] += 1
        return False, None, None
 
    # 5. pose: where must the 0.2 m square be so its corners land on these pixels?
    ok, rvec, tvec = cv2.solvePnP(MARKER_CORNERS_3D, c, K, dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        REJECTS["pnp_failed"] += 1
        return False, None, None
    tvec = tvec.ravel()
    rvec = rvec.ravel()
 
    # 6. range check: must be in front of the camera (z > 0) and not impossibly far
    if not np.all(np.isfinite(tvec)) or tvec[2] <= 0 or np.linalg.norm(tvec) > MAX_RANGE:
        REJECTS["bad_range"] += 1
        return False, None, None
 
    return True, tvec, rvec

# B6: frames: camera -> robot base, then range and bearing 
# OpenCV camera (x right, y down, z forward) -> robot base (x fwd, y left, z up).
# p_base = R_BC @ p_cam + CAM_POS_IN_BASE
CAM_POS_IN_BASE = np.array([0.086, 0.0, 0.107])

# Rotation from the OpenCV camera frame to the robot base frame.
# Each COLUMN says where one camera axis points in the base frame:
#   column 0: cam x (right)   -> base -y   = [ 0, -1,  0]
#   column 1: cam y (down)    -> base -z   = [ 0,  0, -1]
#   column 2: cam z (forward) -> base +x   = [ 1,  0,  0]
R_BC = np.array([[ 0,  0,  1],
                 [-1,  0,  0],
                 [ 0, -1,  0]], dtype=float)

def cam_to_base(tvec):
    # rotate into the base axes, then shift by where the camera is mounted
    p_cam = np.asarray(tvec, dtype=float).ravel()
    p_base = R_BC @ p_cam + CAM_POS_IN_BASE
    return p_base
 
 
def range_bearing(p_base):
    # range = distance in the floor plane, bearing = angle of the marker from the robot's nose
    x, y = p_base[0], p_base[1]          # ignore height (z): the EKF works on the floor plane
    rng = np.hypot(x, y)                 # sqrt(x^2 + y^2)
    bearing = np.arctan2(y, x)           # + = marker to the left, - = to the right (radians)
    return rng, bearing

# B7: Experiment - measure the noise you will put in R 
# Run it with:  python sensing.py noise

# Park the robot at 1, 1.5, 2 m, take 150 photos each, measure range/bearing
# error vs truth. std -> R in the EKF, mean -> bias.  Run: python sensing.py noise
# The sim repeats exactly, so we add pixel noise + tiny pose jitter.
# Scene is sceneforturtle.xml: B7 applies the same size/camera correction as
# sensor_adapter.py. Max 2 m because MIN_SIDE_PX rejects the 0.16 m marker past ~2.2 m.

MARKER_WORLD = np.array([2.888, 0.0])          # marker centre on the floor plane (sceneforturtle.xml)
SCENE_MARKER_SIZE = 0.16                       # real black square: 0.2 m plate x 200/250 (sim.py line 82)
SCENE_CAM_POS = np.array([0.07, 0.0, 0.107])   # real camera mount in this scene
PIX_NOISE = 2.0                         # grey levels out of 255
POS_JITTER = 0.002                      # m
ANG_JITTER = np.radians(0.2)            # rad


def measure_noise(distances=(1.0, 1.5, 2.0), n=150):
    mj_model, data = sim.load()
    renderer, opt = sim.make_renderer(mj_model)
    K, dist = load_calibration()
    rand = np.random.default_rng(0)      # fixed seed so the table is the same every run
    rows = []

    for d in distances:
        rng_err, brg_err = [], []
        for _ in range(n):
            # 1. park the robot d metres in front of the marker, with a tiny random jitter
            x = MARKER_WORLD[0] - d + rand.normal(0, POS_JITTER)
            y = MARKER_WORLD[1] + rand.normal(0, POS_JITTER)
            th = rand.normal(0, ANG_JITTER)
            sim.set_pose(mj_model, data, x, y, th)

            # 2. photo + pixel noise
            img = sim.read_camera(renderer, opt, data).astype(float)
            img = img + rand.normal(0, PIX_NOISE, img.shape)
            img = np.clip(img, 0, 255).astype(np.uint8)

            # 3. measured range and bearing with our B5 + B6
            seen, tvec, _ = detect_marker(img, K, dist)
            if not seen:
                continue
            # same correction as sensor_adapter.py: real marker size and camera mount
            tvec = tvec * SCENE_MARKER_SIZE / MARKER_SIZE
            p = cam_to_base(tvec) + SCENE_CAM_POS - CAM_POS_IN_BASE
            r_meas, b_meas = range_bearing(p)

            # 4. true range and bearing: marker position rotated into the robot's frame
            dx, dy = MARKER_WORLD[0] - x, MARKER_WORLD[1] - y
            bx = np.cos(th) * dx + np.sin(th) * dy
            by = -np.sin(th) * dx + np.cos(th) * dy
            r_true, b_true = np.hypot(bx, by), np.arctan2(by, bx)

            # 5. store the errors (bearing wrapped so 359 deg reads as -1 deg)
            rng_err.append(r_meas - r_true)
            brg_err.append(np.arctan2(np.sin(b_meas - b_true), np.cos(b_meas - b_true)))

        rng_err, brg_err = np.array(rng_err), np.array(brg_err)
        if len(rng_err) < 2:
            print(f"{d:.2f} m: marker seen in {len(rng_err)}/{n} photos - not enough")
            continue
        rows.append((d, len(rng_err), n, rng_err.mean(), rng_err.std(),
                     brg_err.mean(), brg_err.std()))

    # 6. the table for the page
    print("\n dist | seen    | range bias | range std | bearing bias | bearing std")
    print("------+---------+------------+-----------+--------------+------------")
    for d, k, n_tot, rb, rs, bb, bs in rows:
        print(f" {d:4.2f} | {k:3d}/{n_tot:<3d} | {rb*1000:7.1f} mm | {rs*1000:6.1f} mm "
              f"| {np.degrees(bb):9.3f} deg | {np.degrees(bs):7.3f} deg")
    print("\nrejections:", REJECTS)
    print("R diagonal = (range std^2, bearing std^2):")
    for d, _, _, _, rs, _, bs in rows:
        print(f"  {d:.2f} m: [{rs**2:.2e}, {bs**2:.2e}]")

    if not rows:
        return rows

    # save one fixed R for the EKF (ekf.noise_matrices loads the key "R")
    # worst case over the distances, with the bias folded in since the EKF doesn't remove it
    var_r = max(rb**2 + rs**2 for _, _, _, rb, rs, _, _ in rows)
    var_b = max(bb**2 + bs**2 for _, _, _, _, _, bb, bs in rows)
    R = np.diag([var_r, var_b])
    # named after the K it was measured with, so one run never overwrites the other
    calibrated = (Path(__file__).parent / "calib.npz").exists()
    out = Path(__file__).parent / ("camera_R_calibrated.npz" if calibrated else "camera_R_nominal.npz")
    np.savez(out, R=R)
    print("\nsaved", out)
    print("R =\n", R)

    return rows

if __name__ == "__main__":
    # B7 noise experiment:  python sensing.py noise
    if len(sys.argv) > 1 and sys.argv[1] == "noise":
        measure_noise()
        sys.exit()
        
    # B4 calibration:  python sensing.py calib
    if len(sys.argv) > 1 and sys.argv[1] == "calib":
        run_calibration()
        sys.exit()

    # self-check: python sensing.py   (needs the TODOs in B5/B6 done)
    # named mj_model (not model) so it doesn't overwrite "import model" at the top
    mj_model, data = sim.load()
    sim.set_pose(mj_model, data, 1.0, 0.0, 0.0)          # 1.888 m from the marker, facing it
    renderer, opt = sim.make_renderer(mj_model)
    img = sim.read_camera(renderer, opt, data)
    seen, tvec, rvec = detect_marker(img, camera_matrix())
    print("seen:", seen, " tvec:", tvec)
    if seen:
        p = cam_to_base(tvec)
        # raw B5/B6 use the scaffold marker size and camera offset, so in sceneforturtle.xml
        # the raw range reads ~2.46 m; sensor_adapter.py corrects it to ~1.89 m in the pipeline.
        print("marker in base frame (raw):", p)
        print("range, bearing (raw):", range_bearing(p), " expect about (2.46, 0)")
        p_fixed = cam_to_base(tvec * SCENE_MARKER_SIZE / MARKER_SIZE) + SCENE_CAM_POS - CAM_POS_IN_BASE
        print("range, bearing (corrected):", range_bearing(p_fixed), " expect about (1.888, 0)")