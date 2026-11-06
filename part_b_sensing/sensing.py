"""Part B: sensor models, camera calibration and marker measurements (Dinel).

B1 encoder ticks and noise -> odometry u = [v, omega] for the EKF predict step
B2 LiDAR noise from the LDS-01 datasheet
B3 ideal pinhole camera matrix from the field of view
B4 chessboard calibration in simulation        python -m part_b_sensing.sensing calib
B5 ArUco detection and pose (solvePnP)
B6 camera -> robot base frame -> range and bearing z = [rho, beta] for the EKF update
B7 measured camera noise and bias -> R files     python -m part_b_sensing.sensing noise
   corner refinement check (why B5 refines)    python -m part_b_sensing.sensing refine
Self-check of B5 and B6 at one known pose:      python -m part_b_sensing.sensing

Outputs (calibration, R files and result tables) go to part_b_sensing/results_part_b/.
"""

import sys
from pathlib import Path

import cv2
import mujoco
import numpy as np

if __package__ in (None, ""):  # run as a file, not with -m: let Python find the part_* packages
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from part_a_modelling import model
from part_e_integration import sim

RESULTS = Path(__file__).resolve().parent / "results_part_b"
CALIBRATION_FILE = RESULTS / "calib.npz"              # B4 output: K and dist
R_CALIBRATED_FILE = RESULTS / "camera_R_calibrated.npz"   # B7 output, measured with the calibrated K
R_NOMINAL_FILE = RESULTS / "camera_R_nominal.npz"         # B7 output, measured with the ideal K

# B1: encoder noise
# A real encoder gives integer ticks. The TB3's Dynamixel gives 4096 ticks per turn.
# Steps: angle -> ticks (round to an integer) -> back to an angle, plus a small
# Gaussian noise of one tick, standing in for backlash and small slip of a similar size.

TICKS_PER_REV = 4096
TICK = 2 * np.pi / TICKS_PER_REV
ENC_NOISE_RAD = TICK


# noisy_encoders takes the left and right wheel angles from the sim, in radians.
# noise_scale multiplies the noise for robustness tests (1 = the B1 model).
def noisy_encoders(enc_left, enc_right, noise_scale=1.0):
    # Converts the left wheel angle into a whole number of ticks
    left_ticks = np.round(enc_left / TICK)
    # Converts back to radians and adds a small random error
    # np.random.normal picks a random number from a bell curve. 0 means it's centred on zero,
    # no bias, equally likely to be + or -
    left = left_ticks * TICK + np.random.normal(0, ENC_NOISE_RAD * noise_scale)

    right_ticks = np.round(enc_right / TICK)
    right = right_ticks * TICK + np.random.normal(0, ENC_NOISE_RAD * noise_scale)

    return left, right


# wheel angle change -> wheel speeds -> (v, omega) using model.wheels_to_body.
# This (v, omega) is the EKF's input u, so it carries the encoder noise with it.

# odometry takes 3 inputs: the current and previous encoder readings (in radians) and the time step dt (0.02 s).
# enc_now: the wheel angles this loop, as a pair [left (0), right (1)] -> [1.08, 1.12]
# enc_prev: the wheel angles last loop -> [1.00, 1.00]
def odometry(enc_now, enc_prev, dt):
    # Works out how fast the left wheel is spinning
    w_left = (enc_now[0] - enc_prev[0]) / dt
    # Works out how fast the right wheel is spinning
    w_right = (enc_now[1] - enc_prev[1]) / dt
    # Converts the two wheel speeds into the robot's speed with Part A's wheels_to_body (A1)
    v, omega = model.wheels_to_body(w_left, w_right)
    # The EKF needs this in a NumPy array because it's the input vector u for the prediction step
    return np.array([v, omega])


# B2: lidar noise
# The LDS-01 datasheet quotes +-15 mm accuracy for 0.12-0.5 m and +-5 % for 0.5-3.5 m.
# We read "+-" as a 2-sigma bound, so sigma = 7.5 mm below 0.5 m and 2.5 % of the range above.
# "-1" (no return) becomes the max range so the planner treats it as free space.

# real limits of the TurtleBot3's lidar
LIDAR_MIN = 0.12
LIDAR_MAX = 3.5


def noisy_lidar(ranges, noise_scale=1.0):
    # Copy the beam distances into a NumPy array so we can do maths on all of them at once
    r = np.array(ranges, dtype=float)
    # Remember which beams got no return (-1) BEFORE adding noise
    no_return = r < 0
    # Noise size per beam: 7.5 mm for short beams, 2.5% of the distance for long beams
    # (noise_scale multiplies it for robustness tests; 1 = the datasheet model)
    sigma = np.where(r < 0.5, 0.015 / 2, 0.05 * r / 2) * noise_scale
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
# and the principal point is the image centre. This is the "true" K that the
# calibration (B4) is compared against.
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
# Run:  python -m part_b_sensing.sensing calib   -> prints calibrated vs true K, saves calib.npz

CHESS_PATTERN = (7, 5)                                     # inner corners (8 x 6 squares)
CHESS_SQUARE = 0.1                                         # square size in metres
CALIB_SCENE = Path(__file__).resolve().parents[1] / "robotis_tb3" / "calib_scene.xml"   # robot + chessboard
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
    renderer.close()
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
    images = collect_calibration_images()
    out = calibrate(images)
    if out is None:
        return
    K_cal, dist, rms, n = out
    K_true = camera_matrix()
    print(f"used {n} of {len(images)} photos, reprojection error {rms:.3f} px")
    print("true K:\n", np.round(K_true, 2))
    print("calibrated K:\n", np.round(K_cal, 2))
    print("distortion (k1, k2, p1, p2, k3):", np.round(dist.ravel(), 4))
    RESULTS.mkdir(exist_ok=True)
    np.savez(CALIBRATION_FILE, K=K_cal, dist=dist)
    rows = [("fx", K_true[0, 0], K_cal[0, 0]), ("fy", K_true[1, 1], K_cal[1, 1]),
            ("cx", K_true[0, 2], K_cal[0, 2]), ("cy", K_true[1, 2], K_cal[1, 2])]
    lines = ["# B4 calibration result", "",
             f"Made by `python -m part_b_sensing.sensing calib`: {n} of {len(images)} chessboard views used, "
             f"reprojection error {rms:.3f} px.", "",
             "| Parameter | True K (B3) | Calibrated (B4) | Difference |", "|---|---|---|---|"]
    lines += [f"| {name} | {true:.2f} | {cal:.2f} | {cal - true:+.2f} px |" for name, true, cal in rows]
    lines += ["", "Distortion (k1, k2, p1, p2, k3): " + ", ".join(f"{v:.4f}" for v in dist.ravel()), ""]
    (RESULTS / "calibration.md").write_text("\n".join(lines), encoding="utf-8")
    print("saved", CALIBRATION_FILE.relative_to(Path.cwd()) if CALIBRATION_FILE.is_relative_to(Path.cwd())
          else CALIBRATION_FILE)


# B5: ArUco detection and pose
# Finds marker id 0 (DICT_4X4_50) and uses solvePnP (IPPE_SQUARE) to get its
# position in the OpenCV camera frame (x right, y down, z forward).
# Rejects wrong id, tiny markers (< MIN_SIDE_PX), failed or impossible poses.
MARKER_SIZE = 0.16    # black square (m): markermake.py draws 200 px of black on a 250 px plate of 0.2 m
MARKER_ID = 0
MIN_SIDE_PX = 30      # reject markers smaller than this in the image (about 2.2 m away for the 0.16 m marker)
MAX_RANGE = 3.5       # reject poses further than this (impossible given MIN_SIDE_PX)

# The 4 marker corners in the MARKER frame (centre at 0, flat so z = 0), in the order
# OpenCV gives them: top-left, top-right, bottom-right, bottom-left
_h = MARKER_SIZE / 2
MARKER_CORNERS_3D = np.array([[-_h,  _h, 0],
                              [ _h,  _h, 0],
                              [ _h, -_h, 0],
                              [-_h, -_h, 0]], dtype=np.float32)


def make_detector(refine=cv2.aruco.CORNER_REFINE_SUBPIX):
    # Sub-pixel corner refinement: without it the corners snap to whole pixels and the
    # marker can look up to about 1.5 px too small, which reads as up to 8.5 cm too far at 2 m.
    # `python -m part_b_sensing.sensing refine` measures the difference.
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = refine
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)


# Make the detector once, not every frame
_DETECTOR = make_detector()

# Count why detections were thrown away (reported in the trial summaries)
REJECTS = {"wrong_id": 0, "too_small": 0, "pnp_failed": 0, "bad_range": 0}


def load_calibration():
    # Use the calibrated K from B4 if it exists, otherwise the true K from B3
    try:
        c = np.load(CALIBRATION_FILE)
        return c["K"], c["dist"]
    except FileNotFoundError:
        return camera_matrix(), None


def detect_marker(img, K, dist=None, detector=None):
    # returns (seen, tvec, rvec). seen = False when the marker is not in view.
    if dist is None:
        dist = np.zeros(5)                     # sim camera has no lens distortion
    detector = _DETECTOR if detector is None else detector

    # 1. colour photo -> greyscale (the detector only needs light/dark)
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    # 2. find any ArUco markers: corners = pixel corners of each, ids = their numbers
    corners, ids, _ = detector.detectMarkers(gray)
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

    # 5. pose: where must the 0.16 m square be so its corners land on these pixels?
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
CAM_POS_IN_BASE = np.array([0.07, 0.0, 0.15])   # camera mount in robotis_tb3/turtlebot3_waffle_pi.xml

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
    rng = float(np.hypot(x, y))          # sqrt(x^2 + y^2)
    bearing = float(np.arctan2(y, x))    # + = marker to the left, - = to the right (radians)
    return rng, bearing


# B7: Experiment - measure the noise that goes into R
# Park the robot at 1, 1.5, 2 m, take 150 photos each, measure range/bearing
# error against the known pose: mean -> bias, std -> noise.
# The sim repeats exactly, so we add pixel noise + tiny pose jitter.
# Max 2 m because MIN_SIDE_PX rejects the 0.16 m marker past about 2.2 m.
# Run:  python -m part_b_sensing.sensing noise   (both the ideal and the calibrated K)

MARKER_WORLD = np.array([2.888, 0.0])   # marker centre on the floor plane (sceneforturtle.xml)
PIX_NOISE = 2.0                         # grey levels out of 255
POS_JITTER = 0.002                      # m
ANG_JITTER = np.radians(0.2)            # rad


def true_range_bearing(x, y, th):
    # marker position rotated into the robot's frame (used to score B7, never in the loop)
    dx, dy = MARKER_WORLD[0] - x, MARKER_WORLD[1] - y
    bx = np.cos(th) * dx + np.sin(th) * dy
    by = -np.sin(th) * dx + np.cos(th) * dy
    return np.hypot(bx, by), np.arctan2(by, bx)


def measure_noise(K, dist, distances=(1.0, 1.5, 2.0), n=150):
    mj_model, data = sim.load()
    renderer, opt = sim.make_renderer(mj_model)
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
            r_meas, b_meas = range_bearing(cam_to_base(tvec))

            # 4. true range and bearing from the pose we set
            r_true, b_true = true_range_bearing(x, y, th)

            # 5. store the errors (bearing wrapped so 359 deg reads as -1 deg)
            rng_err.append(r_meas - r_true)
            brg_err.append(np.arctan2(np.sin(b_meas - b_true), np.cos(b_meas - b_true)))

        rng_err, brg_err = np.array(rng_err), np.array(brg_err)
        if len(rng_err) < 2:
            print(f"{d:.2f} m: marker seen in {len(rng_err)}/{n} photos - not enough")
            continue
        rows.append((d, len(rng_err), n, rng_err.mean(), rng_err.std(),
                     brg_err.mean(), brg_err.std()))
    renderer.close()
    return rows


def noise_table(rows):
    lines = ["| Distance | Seen | Range bias | Range σ | Bearing bias | Bearing σ |", "|---|---|---|---|---|---|"]
    for d, k, n_tot, rb, rs, bb, bs in rows:
        lines.append(f"| {d:.1f} m | {k}/{n_tot} | {rb * 1000:.1f} mm | {rs * 1000:.1f} mm "
                     f"| {np.degrees(bb):.3f}° | {np.degrees(bs):.3f}° |")
    return lines


def r_from_rows(rows):
    # one fixed R for the EKF: worst case over the distances, with the bias folded in
    # as extra variance because the EKF does not remove it
    var_r = max(rb**2 + rs**2 for _, _, _, rb, rs, _, _ in rows)
    var_b = max(bb**2 + bs**2 for _, _, _, _, _, bb, bs in rows)
    return np.diag([var_r, var_b])


def run_noise_experiment():
    if not CALIBRATION_FILE.is_file():
        print("No calibration yet: run  python -m part_b_sensing.sensing calib  first")
        return
    RESULTS.mkdir(exist_ok=True)
    cal = np.load(CALIBRATION_FILE)
    lines = ["# B7 measured camera noise", "",
             "Made by `python -m part_b_sensing.sensing noise`. 150 photos per distance, head-on, with pixel "
             f"noise σ = {PIX_NOISE:g} grey levels and pose jitter σ = {POS_JITTER * 1000:g} mm, "
             f"{np.degrees(ANG_JITTER):g}°. Error = measured − true.", ""]
    for label, K, dist, out in (("Ideal K (B3)", camera_matrix(), None, R_NOMINAL_FILE),
                                ("Calibrated K (B4)", cal["K"], cal["dist"], R_CALIBRATED_FILE)):
        for reason in REJECTS:
            REJECTS[reason] = 0
        rows = measure_noise(K, dist)
        R = r_from_rows(rows)
        np.savez(out, R=R)
        print(f"\n{label}")
        print("\n".join(noise_table(rows)))
        print(f"rejections: {REJECTS}")
        print(f"R = diag({R[0, 0]:.3e}, {R[1, 1]:.3e}) -> sigma range {np.sqrt(R[0, 0]) * 1000:.1f} mm, "
              f"sigma bearing {np.degrees(np.sqrt(R[1, 1])):.3f} deg, saved {out.name}")
        lines += [f"## {label}", ""] + noise_table(rows)
        lines += ["", f"Saved `{out.name}`: R = diag({R[0, 0]:.3e} m², {R[1, 1]:.3e} rad²), that is "
                      f"σ range {np.sqrt(R[0, 0]) * 1000:.1f} mm and σ bearing "
                      f"{np.degrees(np.sqrt(R[1, 1])):.3f}° (worst distance, bias folded in).", ""]
    (RESULTS / "noise.md").write_text("\n".join(lines), encoding="utf-8")


def refinement_check(distances=(0.6, 0.9, 1.0, 1.25, 1.5, 1.75, 1.888, 2.0, 2.2)):
    # Range error with and without sub-pixel corner refinement, robot head-on, no noise.
    mj_model, data = sim.load()
    renderer, opt = sim.make_renderer(mj_model)
    detectors = {"none": make_detector(cv2.aruco.CORNER_REFINE_NONE),
                 "subpixel": make_detector(cv2.aruco.CORNER_REFINE_SUBPIX)}
    lines = ["# B5 corner refinement check", "",
             "Made by `python -m part_b_sensing.sensing refine`: robot head-on to the marker, ideal K, no noise. "
             "Range error = measured − true.", "",
             "| Distance | No refinement | Sub-pixel refinement |", "|---|---|---|"]
    for d in distances:
        x = MARKER_WORLD[0] - d
        sim.set_pose(mj_model, data, x, 0.0, 0.0)
        img = sim.read_camera(renderer, opt, data)
        cells = []
        for detector in detectors.values():
            seen, tvec, _ = detect_marker(img, camera_matrix(), detector=detector)
            cells.append(f"{(range_bearing(cam_to_base(tvec))[0] - d) * 1000:+.1f} mm" if seen else "not seen")
        lines.append(f"| {d:.3f} m | {cells[0]} | {cells[1]} |")
    renderer.close()
    print("\n".join(lines))
    RESULTS.mkdir(exist_ok=True)
    (RESULTS / "corner_refinement.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def self_check():
    # B5 + B6 at one known pose: robot base 1.888 m from the marker, facing it
    mj_model, data = sim.load()
    sim.set_pose(mj_model, data, 1.0, 0.0, 0.0)
    renderer, opt = sim.make_renderer(mj_model)
    img = sim.read_camera(renderer, opt, data)
    renderer.close()
    seen, tvec, _ = detect_marker(img, camera_matrix())
    print("marker seen:", seen)
    if seen:
        p = cam_to_base(tvec)
        rng, brg = range_bearing(p)
        print("marker in camera frame (m):", np.round(tvec, 4))
        print("marker in base frame (m):  ", np.round(p, 4))
        print(f"range {rng:.3f} m, bearing {np.degrees(brg):+.2f} deg   (true: 1.888 m, +0.00 deg;"
              f" results_part_b/corner_refinement.md has the range error at each distance)")


if __name__ == "__main__":
    task = sys.argv[1] if len(sys.argv) > 1 else "check"
    actions = {"calib": run_calibration, "noise": run_noise_experiment,
               "refine": refinement_check, "check": self_check}
    if task not in actions:
        sys.exit("usage: python -m part_b_sensing.sensing [calib | noise | refine]")
    actions[task]()
