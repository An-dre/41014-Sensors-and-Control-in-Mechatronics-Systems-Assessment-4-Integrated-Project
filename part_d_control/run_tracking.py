# run_tracking.py - PART D results. Owner: Andre.
# Closed-loop test bench for the LQR tracker in controller.py:
#     wheel encoders -> odometry -> ekf.predict
#     camera sees the ArUco marker -> ekf.update
#     EKF estimate -> controller.LQRTracker -> wheel speeds -> robot
#
# Ground truth: each plant has truth(), the pose the simulator says the robot really has.
# Only the referee calls it: the scoring log, the stop-if-it-hits-something check, and
# --check-sim. The fake camera (--camera synthetic) also needs to know where the robot is,
# to work out what a camera would see, but it reads the plant's own state (_pose) the way
# MuJoCo's renderer does. Nothing in the control path (encoders, odometry, EKF, controller)
# is handed either. --leak-check proves it: it re-runs with truth() itself swapped for one
# that only returns NaN, and checks every wheel command comes out bit-for-bit the same.
#
# Run from the outer project folder:
#   python part_d_control\run_tracking.py                     one run of each route in MuJoCo
#   python part_d_control\run_tracking.py --trials 10         10 random trials per route
#   python part_d_control\run_tracking.py --compare           final design vs the alternatives
#   python part_d_control\run_tracking.py --sweep             effort vs tracking error (R sweep)
#   python part_d_control\run_tracking.py --leak-check        proves the truth never steers it
#   python part_d_control\run_tracking.py --check-sim         check wheels and marker reading first
#   python part_d_control\run_tracking.py --viewer            watch one run in the MuJoCo viewer
#   python part_d_control\run_tracking.py --wheel-gain 0.9 1  left motor 10 % weak (disturbance)
#   python part_d_control\run_tracking.py --report            all of the above that the write-up uses
# Add --camera synthetic to fake the marker reading from the pose instead of rendering the
# camera (about 40 times faster), --as-is to load the robot exactly as the xml says (it
# slips, see fix_floor_contacts), or --plant kinematic to use the fast stand-in model (no
# MuJoCo needed). Plots and tables go to part_d_control/results_part_d/.

import argparse
import csv
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# so the imports below can find the files in the part_* folders
sys.path[:0] = [str(p) for p in sorted(Path(__file__).resolve().parent.parent.glob("part_*"))]

import controller as ctl
import ekf
import model
from model import r, L

HERE = Path(__file__).resolve().parent
SCENE = HERE.parent / "robotis_tb3" / "sceneforturtle.xml"
OUT = HERE / "results_part_d"
DT = ctl.DT


# --- the room, read from the same scene file MuJoCo loads --------------------------
def load_room(scene=SCENE):
    # Every box geom becomes a 2D rectangle (centre, half sizes); they're all axis
    # aligned in sceneforturtle.xml. The marker is the "dock_marker" geom, and its
    # normal points from the dock towards the marker, i.e. out into the room.
    root = ET.parse(scene).getroot()
    boxes, centres = {}, {}
    for g in root.iter("geom"):
        if g.get("type") != "box":
            continue
        pos = np.array([float(v) for v in g.get("pos").split()])
        size = np.array([float(v) for v in g.get("size").split()])
        centres[g.get("name")] = pos[:2]
        boxes[g.get("name")] = (pos[:2], size[:2])
    marker = centres["dock_marker"]
    normal = ctl._unit(marker - centres["dock"])
    return boxes, marker, normal


BOXES, MARKER, NORMAL = load_room()
P_PRE, P_DOCK, TH_DOCK = ctl.dock_targets(MARKER, NORMAL)
DOCK_POSE = np.array([P_DOCK[0], P_DOCK[1], TH_DOCK])

# Test routes: start pose, then corner points up to somewhere near the dock;
# docking_reference() adds the straight final leg. These stand in for the planner's
# waypoints. Before the final leg, every path keeps its centre line at least
# ROUTE_CLEARANCE from every box, which test_controller.py checks.
# I first used 0.30 m (robot radius 0.25 m + 5 cm). 2 of 10 slalom trials then clipped
# box1: the camera can't see the dock from that far, so the robot was on odometry alone,
# and a 3 deg start heading error had already put it 15 cm off the path. The controller
# was holding the estimate to under 1 mm the whole time. 0.40 m leaves room for that drift.
ROUTE_CLEARANCE = 0.40
ROUTES = {
    "slalom": ((-2.3, 0.0, 0.0),
               [(-2.3, 0.0), (-1.4, 0.0), (-0.6, 0.30), (0.0, 0.30), (0.9, -0.35)]),
    "north": ((-2.2, 2.0, -np.pi / 2),
              [(-2.2, 2.0), (-2.2, 1.2), (1.6, 1.2)]),
    "south": ((-1.0, -2.2, 0.0),           # starts facing east but the path goes north
              [(-1.0, -2.2), (-1.0, -1.3), (1.0, -1.3)]),
}


def route_reference(name, **kwargs):
    _, route = ROUTES[name]
    return ctl.docking_reference(route, MARKER, NORMAL, **kwargs)


# --- robot footprint and clearance (scoring only) ----------------------------------
# From the robot xml and the base mesh: chassis x -0.197..0.077 with the camera bracket
# out to 0.086, and the tyres out to y = +-0.153.
FOOT = np.array([[-0.197, -0.153], [0.086, -0.153], [0.086, 0.153], [-0.197, 0.153]])


def footprint(pose):
    c, s = np.cos(pose[2]), np.sin(pose[2])
    return FOOT @ np.array([[c, s], [-s, c]]) + pose[:2]


BOX_C = np.array([c for c, _ in BOXES.values()])          # (n, 2) centres
BOX_H = np.array([h for _, h in BOXES.values()])          # (n, 2) half sizes
BOX_CORNERS = np.concatenate([BOX_C + BOX_H * np.array(s) for s in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
FOOT_C = FOOT.mean(axis=0)
FOOT_H = (FOOT.max(axis=0) - FOOT.min(axis=0)) / 2


def _to_box(points, centres, halves):
    # distance from each point to each axis-aligned box (0 inside), shape (points, boxes)
    d = np.abs(points[:, None, :] - centres[None, :, :]) - halves[None, :, :]
    return np.linalg.norm(np.maximum(d, 0.0), axis=2)


def clearance(pose):
    # Gap between the robot's footprint and the nearest box (walls, boxes, dock).
    # For two rectangles the closest pair always includes a corner of one of them, so I
    # check robot corners against the boxes, and box corners against the robot (in the
    # robot's own frame, where its footprint is axis aligned).
    c, s = np.cos(pose[2]), np.sin(pose[2])
    corners = footprint(pose)
    gap = _to_box(corners, BOX_C, BOX_H).min()
    local = (BOX_CORNERS - pose[:2]) @ np.array([[c, -s], [s, c]])
    gap = min(gap, _to_box(local, FOOT_C[None], FOOT_H[None]).min())
    return float(gap)


# --- sensor stand-ins --------------------------------------------------------------
# These do what sensing.py does (B1 encoders, B5/B6 marker). sensing.py can't be imported
# yet because the sim module it uses isn't in the repo, and model.wheels_to_body (A1) is
# still a TODO. At integration these get swapped for the sensing.py functions.
TICK = 2 * np.pi / 4096
CAM_POS = 0.07                       # camera "rgb" sits 7 cm ahead of the base origin
CAM_W, CAM_H, CAM_FOVY = 640, 480, 60.0
FOCAL = (CAM_H / 2) / np.tan(np.radians(CAM_FOVY) / 2)
MARKER_SIDE = 0.16                   # black square only: 0.2 m plate x 200/250 px (markermake.py)
MIN_SIDE_PX = 30                     # same "too small" rule as sensing.py
CAM_EVERY = 5                        # a camera frame every 5th control step = 10 Hz
K_CAM = np.array([[FOCAL, 0, CAM_W / 2], [0, FOCAL, CAM_H / 2], [0, 0, 1]])


def read_encoders(angles, rng):
    # whole ticks plus a little noise, like sensing.noisy_encoders
    return np.round(np.asarray(angles) / TICK) * TICK + rng.normal(0.0, TICK, 2)


def odometry(enc_now, enc_prev, dt=DT):
    w_left, w_right = (enc_now - enc_prev) / dt
    return np.array([r * (w_right + w_left) / 2, r * (w_right - w_left) / L])


def marker_visible(pose):
    # could the camera see the whole marker from here, big enough to pass MIN_SIDE_PX?
    c, s = np.cos(pose[2]), np.sin(pose[2])
    cam = pose[:2] + CAM_POS * np.array([c, s])
    to_m = MARKER - cam
    d = np.linalg.norm(to_m)
    facing = -(to_m / d) @ NORMAL                      # 1 = looking straight at it
    bearing = ctl.wrap(np.arctan2(to_m[1], to_m[0]) - pose[2])
    fits_side = abs(bearing) + np.arctan(MARKER_SIDE / 2 / d) < np.arctan((CAM_W / 2) / FOCAL)
    fits_up = d * np.tan(np.radians(CAM_FOVY) / 2) > MARKER_SIDE / 2
    big_enough = FOCAL * MARKER_SIDE * facing / d >= MIN_SIDE_PX
    return facing > np.cos(np.radians(60)) and fits_side and fits_up and big_enough


def synthetic_marker(pose_true, rng):
    # The simulator standing in for the camera: if the marker is in view, report range
    # and bearing from the base origin with noise like sensing.py measured (about 1 cm
    # at 1 m, 3 cm at 3 m). Only used with --camera synthetic or the kinematic stand-in.
    if not marker_visible(pose_true):
        return None
    z = ekf.h(pose_true, MARKER)
    return z + np.array([rng.normal(0.0, 0.002 + 0.01 * z[0]), rng.normal(0.0, np.radians(0.5))])


_DETECTOR = None


def rendered_marker(image):
    # Real camera path for MuJoCo: find marker 0 in the rendered image, solvePnP for where
    # it is, move that into the base frame, then range and bearing. Same steps as
    # sensing.py B5/B6, but with this scene's camera (7 cm ahead) and marker size.
    # Two things I found testing this on synthetic images (worth copying into sensing.py):
    #   * without subpixel corner refinement the corners sit ~0.3 px inside the edge, so the
    #     marker looks small and range reads about 2 % long at 1 m. SUBPIX brings it to ~0.3 %.
    #   * SOLVEPNP_IPPE_SQUARE can say ok = True and still hand back NaN. I check for that and
    #     fall back to the default solver.
    global _DETECTOR
    import cv2
    if _DETECTOR is None:
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        _DETECTOR = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = _DETECTOR.detectMarkers(gray)
    if ids is None:
        return None
    hits = np.where(np.ravel(ids) == 0)[0]
    if len(hits) == 0:
        return None
    c = corners[hits[0]].reshape(4, 2).astype(np.float32)
    if np.linalg.norm(c - np.roll(c, 1, axis=0), axis=1).min() < MIN_SIDE_PX:
        return None
    h = MARKER_SIDE / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)
    ok, _, tvec = cv2.solvePnP(obj, c, K_CAM, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or not np.all(np.isfinite(tvec)):
        ok, _, tvec = cv2.solvePnP(obj, c, K_CAM, None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or not np.all(np.isfinite(tvec)) or tvec[2, 0] <= 0:
        return None
    tx, _, tz = tvec.ravel()
    forward, left = tz + CAM_POS, -tx            # OpenCV: x right, z forward -> base frame
    rng_ = np.hypot(forward, left)
    if rng_ > 3.5:
        return None
    return np.array([rng_, np.arctan2(left, forward)])


# --- plants ------------------------------------------------------------------------


def fix_floor_contacts(model, mujoco):
    # Keep the old test bench API, using the same correction as the pipeline.
    from sim import fix_floor_contacts as correct_contacts
    correct_contacts(model, mujoco)


class MujocoTB3:
    # The group's Waffle Pi scene. Wheels take speed commands (rad/s) through data.ctrl.
    def __init__(self, camera="render", wheel_gain=(1.0, 1.0), as_is=False):
        try:
            import mujoco
        except ImportError:
            raise SystemExit("MuJoCo isn't installed here (pip install mujoco), "
                             "or add --plant kinematic to use the stand-in model")
        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(SCENE))
        if not as_is:
            fix_floor_contacts(self.model, mujoco)
        self.data = mujoco.MjData(self.model)
        self.substeps = int(round(DT / self.model.opt.timestep))
        self.camera = camera
        self.renderer = mujoco.Renderer(self.model, CAM_H, CAM_W) if camera == "render" else None
        self.wheel_gain = np.array(wheel_gain)

    def reset(self, pose):
        self.mj.mj_resetData(self.model, self.data)
        self.data.qpos[0:3] = [pose[0], pose[1], 0.0]
        self.data.qpos[3:7] = [np.cos(pose[2] / 2), 0.0, 0.0, np.sin(pose[2] / 2)]
        self.mj.mj_forward(self.model, self.data)

    def wheel_angles(self):
        return np.array(self.data.sensordata[0:2])

    def drive(self, w_left, w_right):
        self.data.ctrl[:] = np.array([w_left, w_right]) * self.wheel_gain
        for _ in range(self.substeps):
            self.mj.mj_step(self.model, self.data)

    def marker(self, rng):
        if self.camera == "render":
            self.renderer.update_scene(self.data, camera="rgb")
            return rendered_marker(self.renderer.render())
        return synthetic_marker(self._pose(), rng)

    def _pose(self):
        # the simulator's own state. Only the fake camera uses this directly.
        w, qx, qy, qz = self.data.qpos[3:7]
        yaw = np.arctan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        return np.array([self.data.qpos[0], self.data.qpos[1], yaw])

    def truth(self):
        # REFEREE ONLY: scoring, the collision check and --check-sim. Never control.
        return self._pose()


class KinematicTB3:
    # Fast stand-in for tuning when MuJoCo isn't available: unicycle kinematics plus the
    # wheel servo from the robot xml (velocity actuator kv = 5, armature 0.01, frictionloss
    # 0.1), stepped at MuJoCo's 2 ms. No contact physics and no rendering, so the camera
    # reading is synthetic. wheel_gain / radius_scale are for robustness runs.
    def __init__(self, wheel_gain=(1.0, 1.0), radius_scale=(1.0, 1.0), **_):
        self.substeps = 10
        self.h = DT / self.substeps
        self.tau = 0.011 / 5.0          # wheel inertia (armature + share of the robot) / kv
        self.offset = 0.1 / 5.0         # friction torque / kv = steady speed shortfall
        self.wheel_gain = np.array(wheel_gain)
        self.radius = r * np.array(radius_scale)

    def reset(self, pose):
        self.pose = np.array(pose, dtype=float)
        self.wheel = np.zeros(2)
        self.angle = np.zeros(2)

    def wheel_angles(self):
        return self.angle.copy()

    def drive(self, w_left, w_right):
        cmd = np.clip(np.array([w_left, w_right]) * self.wheel_gain, -7.88, 7.88)
        a = np.exp(-self.h / self.tau)
        for _ in range(self.substeps):
            target = cmd - self.offset * np.sign(cmd)
            self.wheel = target + (self.wheel - target) * a
            self.angle += self.wheel * self.h
            v = (self.radius[1] * self.wheel[1] + self.radius[0] * self.wheel[0]) / 2
            w = (self.radius[1] * self.wheel[1] - self.radius[0] * self.wheel[0]) / L
            th = self.pose[2]
            if abs(w) < 1e-9:
                self.pose[:2] += v * self.h * np.array([np.cos(th), np.sin(th)])
            else:
                self.pose[0] += v / w * (np.sin(th + w * self.h) - np.sin(th))
                self.pose[1] += v / w * (np.cos(th) - np.cos(th + w * self.h))
            self.pose[2] = ctl.wrap(th + w * self.h)

    def marker(self, rng):
        return synthetic_marker(self._pose(), rng)

    def _pose(self):
        # the stand-in's own state. Only the fake camera uses this directly.
        return self.pose.copy()

    def truth(self):
        # REFEREE ONLY: scoring and the collision check. Never control.
        return self._pose()


# --- one closed-loop run -----------------------------------------------------------
START_SD = np.array([0.02, 0.02, np.radians(1.5)])   # how accurately it's put down at the start


def run(route_name, plant, seed=0, start_error=True, scheduled=True, feedforward=True,
        R_scale=1.0, t_max=150.0, viewer=None):
    rng = np.random.default_rng(seed)
    start = np.array(ROUTES[route_name][0], dtype=float)
    ref = route_reference(route_name)
    tracker = ctl.LQRTracker(ref, scheduled=scheduled, feedforward=feedforward, R=ctl.R_LQR * R_scale)

    plant.reset(start + (rng.normal(0.0, START_SD) if start_error else 0.0))

    # The EKF only knows where the robot was meant to be put down, and how sure that is.
    x_hat, P = start.copy(), np.diag(START_SD**2)
    ekf.reset_update_counts()
    enc_prev = read_encoders(plant.wheel_angles(), rng)

    keys = ("t", "true", "est", "P", "pose_ref", "e", "mode", "v", "w", "v_ref", "w_ref",
            "wl", "wr", "scale", "fix", "gap")
    log = {k: [] for k in keys}
    collided = False
    for k in range(int(t_max / DT)):
        t = k * DT
        # 1. odometry -> predict. u is the motion the encoders measured, not my command.
        enc = read_encoders(plant.wheel_angles(), rng)
        x_hat, P = ekf.predict(x_hat, P, odometry(enc, enc_prev), DT)
        enc_prev = enc
        # 2. camera -> update, on frames where the marker is found
        fix = False
        if k % CAM_EVERY == 0:
            z = plant.marker(rng)
            if z is not None:
                before = ekf.updates_accepted
                x_hat, P = ekf.update(x_hat, P, z, MARKER)
                fix = ekf.updates_accepted > before
        # 3. controller, on the estimate only
        v, w = tracker.step(x_hat, t)
        w_left, w_right, _ = ctl.wheel_speeds(v, w)

        # 4. referee: the only place in the loop the true pose is read. It's logged for
        #    scoring, and ends the run if the real robot touches a box.
        truth = plant.truth()
        gap = clearance(truth)
        info = tracker.last
        if tracker.mode == "track":
            pose_ref = ref.sample(t - tracker.t0)[0]
        elif tracker.mode == "align":
            pose_ref = np.array([ref.x[0], ref.y[0], ref.th[0]])
        else:
            pose_ref = ref.end_pose()
        row = (t, truth, x_hat.copy(), np.diag(P).copy(), pose_ref, info["e"], info["mode"], v, w,
               info["v_ref"], info["w_ref"], w_left, w_right, info["scale"], fix, gap)
        for key, val in zip(keys, row):
            log[key].append(val)
        if gap <= 0.0:
            collided = True
            break
        if tracker.docked or tracker.mode == "timeout":
            break

        # 5. send the wheel speeds and step the physics
        plant.drive(w_left, w_right)
        if viewer is not None:
            viewer.sync()
            time.sleep(DT)

    res = {k: np.array(v, dtype=object if k == "mode" else float) for k, v in log.items()}
    res["fix"] = res["fix"].astype(bool)
    res.update(route=route_name, seed=seed, reference=ref, docked=tracker.docked,
               collided=collided, accepted=ekf.updates_accepted, rejected=ekf.updates_rejected)
    return res


def leak_check(route_name, plant, seed=1):
    # The same run twice. The second time the simulator's truth() is swapped for one that
    # only returns NaN. If the encoders, EKF or controller used the true pose anywhere, the
    # wheel commands would change (or turn into NaN). With the rendered camera nothing in
    # the loop needs the pose at all, so I blank _pose() as well. The fake camera does need
    # it, so with that one only truth() is blanked.
    nan = lambda: np.full(3, np.nan)
    a = run(route_name, plant, seed=seed)
    plant.truth = nan                    # instance attribute, hides the method for now
    if getattr(plant, "camera", None) == "render":
        plant._pose = nan
    try:
        b = run(route_name, plant, seed=seed)
    finally:
        del plant.truth
        if "_pose" in vars(plant):
            del plant._pose
    # If the real robot hit a box, the referee stopped run a there. Run b can't see the
    # collision (its referee is blind), so it carries on: compare up to that point.
    n = len(a["wl"])
    same = ((len(b["wl"]) == n or a["collided"]) and np.array_equal(a["wl"], b["wl"][:n])
            and np.array_equal(a["wr"], b["wr"][:n]))
    return same, n, bool(np.all(np.isnan(b["true"])))


# --- scoring -----------------------------------------------------------------------
def _rms(a):
    return float(np.sqrt(np.mean(np.square(a)))) if len(a) else float("nan")


def score(res):
    track = res["mode"] == "track"
    true_e = np.array([ctl.tracking_error(p, q) for p, q in zip(res["true"], res["pose_ref"])])
    est_e = res["e"]
    dock_err = ctl.tracking_error(res["true"][-1], DOCK_POSE)   # in the robot's own frame
    est_err = res["est"] - res["true"]
    dw_fb = (res["w"] - res["w_ref"])[track]
    acc = np.diff(np.c_[res["wl"], res["wr"]], axis=0) / DT
    success = (res["docked"] and not res["collided"] and abs(dock_err[0]) < 0.03
               and abs(dock_err[1]) < 0.02 and abs(dock_err[2]) < np.radians(3))
    return dict(
        route=res["route"], seed=res["seed"], success=bool(success), docked=bool(res["docked"]),
        collided=bool(res["collided"]), time=float(res["t"][-1]),
        rms_ey_true=_rms(true_e[track, 1]), max_ey_true=float(np.max(np.abs(true_e[track, 1]))),
        rms_ey_est=_rms(est_e[track, 1]), max_ey_est=float(np.max(np.abs(est_e[track, 1]))),
        rms_ex_est=_rms(est_e[track, 0]), rms_eth_est_deg=float(np.degrees(_rms(est_e[track, 2]))),
        rms_pos_est_err=_rms(np.linalg.norm(est_err[:, :2], axis=1)),
        dock_along=float(dock_err[0]), dock_side=float(dock_err[1]),
        dock_head_deg=float(np.degrees(dock_err[2])),
        rms_dw_fb=_rms(dw_fb), rms_wheel_acc=_rms(acc.ravel()),
        sat_frac=float(np.mean(res["scale"] < 1.0)), min_gap=float(np.nanmin(res["gap"])),
        fixes=int(np.sum(res["fix"])), rejected=int(res["rejected"]),
    )


# --- plots -------------------------------------------------------------------------
def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def draw_room(ax):
    from matplotlib.patches import Rectangle
    for name, (c, h) in BOXES.items():
        ax.add_patch(Rectangle(c - h, 2 * h[0], 2 * h[1], color="0.8" if "wall" in name else "0.45", lw=0))
    ax.plot(*MARKER, "s", color="k", ms=4)
    ax.set_aspect("equal")
    ax.set_xlim(-3.05, 3.05)
    ax.set_ylim(-3.05, 3.05)


def _shade(ax, t, fix):
    # light grey wherever the camera has the marker
    seen = np.convolve(fix.astype(float), np.ones(2 * CAM_EVERY), "same") > 0
    lo, hi = ax.get_ylim()
    ax.fill_between(t, lo, hi, where=seen, color="0.5", alpha=0.10, lw=0)
    ax.set_ylim(lo, hi)


def plot_run(res, path, title):
    from matplotlib.patches import Ellipse
    plt = _plt()
    t, true, est, Pd, ref = res["t"], res["true"], res["est"], res["P"], res["reference"]
    true_e = np.array([ctl.tracking_error(p, q) for p, q in zip(true, res["pose_ref"])])
    fig = plt.figure(figsize=(13, 9))

    ax = fig.add_subplot(2, 2, 1)
    draw_room(ax)
    ax.plot(ref.x, ref.y, color="0.15", ls=":", lw=1.2, label="reference")
    ax.plot(true[:, 0], true[:, 1], color="C0", lw=1.8, label="true path")
    ax.plot(est[:, 0], est[:, 1], color="C3", ls="--", lw=1.1, label="EKF estimate")
    fix = res["fix"]
    ax.plot(est[fix, 0], est[fix, 1], ".", color="C2", ms=3, label="camera fix")
    for k in range(0, len(t), int(5 / DT)):
        sd = np.sqrt(Pd[k, :2])
        ax.add_patch(Ellipse(est[k, :2], 4 * sd[0], 4 * sd[1], fill=False, color="C3", lw=0.8))
        ax.plot(*footprint(true[k])[[0, 1, 2, 3, 0]].T, color="C0", lw=0.5, alpha=0.5)
    ax.plot(*footprint(true[-1])[[0, 1, 2, 3, 0]].T, color="C0", lw=1.0)
    ax.set_title(title, fontsize=10)
    ax.legend(fontsize=7, loc="lower right")

    ax = fig.add_subplot(2, 2, 2)
    ax.plot(t, 100 * true_e[:, 1], color="C0", lw=1.2, label="true")
    ax.plot(t, 100 * res["e"][:, 1], color="C3", lw=1.0, ls="--", label="what the controller sees")
    ax.axhline(0, color="k", lw=0.5)
    ax.set_ylabel("cm")
    ax.set_title("cross-track error (grey = camera has the marker)", fontsize=10)
    _shade(ax, t, fix)
    ax.legend(fontsize=7)

    ax = fig.add_subplot(2, 2, 3)
    ax.plot(t, res["v"], color="C0", lw=1.2, label="v command (m/s)")
    ax.plot(t, res["v_ref"], color="C0", ls=":", lw=1.0, label="v plan")
    ax.plot(t, res["w"], color="C1", lw=1.2, label="w command (rad/s)")
    ax.plot(t, res["w_ref"], color="C1", ls=":", lw=1.0, label="w plan")
    ax.set_xlabel("time (s)")
    ax.set_title("commands vs plan", fontsize=10)
    _shade(ax, t, fix)
    ax.legend(fontsize=7)

    ax = fig.add_subplot(2, 2, 4)
    ax.plot(t, res["wl"], lw=1.0, label="left wheel")
    ax.plot(t, res["wr"], lw=1.0, label="right wheel")
    for s in (1, -1):
        ax.axhline(s * ctl.WHEEL_LIMIT, color="k", ls="--", lw=0.8)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("rad/s")
    ax.set_title("wheel speed commands (dashed = limit)", fontsize=10)
    _shade(ax, t, fix)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# --- experiments -------------------------------------------------------------------
def make_plant(args, **kwargs):
    if args.plant == "mujoco":
        return MujocoTB3(camera=args.camera, as_is=args.as_is, **kwargs)
    return KinematicTB3(**kwargs)


def make_tag(args):
    # name tag for the result files, so different set-ups never overwrite each other
    tag = args.plant
    if args.plant == "mujoco":
        tag += f"_{args.camera}" + ("_asis" if args.as_is else "")
    if tuple(args.wheel_gain) != (1.0, 1.0):
        tag += f"_gain{args.wheel_gain[0]:g}-{args.wheel_gain[1]:g}"
    if args.trials > 1 and not (args.compare or args.sweep):
        tag += f"_x{args.trials}"
    return tag


SUMMARY_KEYS = ["rms_ey_true", "max_ey_true", "rms_ey_est", "rms_pos_est_err", "dock_along",
                "dock_side", "dock_head_deg", "rms_dw_fb", "rms_wheel_acc", "sat_frac", "min_gap", "time"]


def summarise(rows, title):
    lines = [f"### {title}", "",
             f"{len(rows)} runs. Docked within tolerance: {sum(r_['success'] for r_ in rows)}/{len(rows)}."
             f" Collisions: {sum(r_['collided'] for r_ in rows)}.", "",
             "| metric | mean | sd | worst |", "|---|---|---|---|"]
    for k in SUMMARY_KEYS:
        a = np.array([r_[k] for r_ in rows], dtype=float)
        worst = np.nanmin(a) if k == "min_gap" else a[np.nanargmax(np.abs(a))]
        lines.append(f"| {k} | {np.nanmean(a):.4f} | {np.nanstd(a):.4f} | {worst:.4f} |")
    return "\n".join(lines) + "\n"


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def trials(args, plant, routes, tag):
    rows = []
    for name in routes:
        worst = None
        for trial in range(args.trials):
            seed = args.seed + trial
            t0 = time.time()
            res = run(name, plant, seed=seed, start_error=(args.trials > 1 or seed > 0))
            row = score(res)
            rows.append(row)
            print(f"{name:7s} seed {seed:4d}: docked ok {row['success']!s:5s} rms e_y true "
                  f"{100 * row['rms_ey_true']:.1f} cm, seen {100 * row['rms_ey_est']:.1f} cm |"
                  f" dock side {100 * row['dock_side']:+.1f} cm, along {100 * row['dock_along']:+.1f} cm,"
                  f" head {row['dock_head_deg']:+.1f} deg | min gap {100 * row['min_gap']:.0f} cm"
                  f"  ({time.time() - t0:.1f} s)")
            if trial == 0:
                plot_run(res, OUT / f"run_{tag}_{name}.png", f"{name} route ({tag}, seed {seed})")
            if args.trials > 1 and (worst is None or row["max_ey_true"] > worst[1]["max_ey_true"]):
                worst = (res, row)
        if worst is not None:
            plot_run(worst[0], OUT / f"worst_{tag}_{name}.png",
                     f"{name}: worst of {args.trials} (seed {worst[1]['seed']})")
    write_csv(rows, OUT / f"trials_{tag}.csv")
    text = summarise(rows, f"{tag}, {args.trials} trial(s) per route")
    (OUT / f"summary_{tag}.md").write_text(text)
    print(text)


def compare(args, plant, routes, tag):
    variants = {
        "final: scheduled LQR + feedforward": dict(),
        "one fixed gain (straight line, 0.15 m/s)": dict(scheduled=False),
        "no turn-rate feedforward": dict(feedforward=False),
    }
    lines = [f"### Design comparison ({tag}, {args.trials} trial(s) per route)", "",
             "| variant | docked ok | rms e_y true (cm) | max e_y true (cm) | rms e_y seen (cm) |"
             " max e_y seen (cm) | rms feedback w (rad/s) | wheel limit hit (% of steps) |",
             "|---|---|---|---|---|---|---|---|"]
    for label, kw in variants.items():
        rows = [score(run(name, plant, seed=args.seed + trial, **kw))
                for name in routes for trial in range(args.trials)]
        mean = lambda k: np.mean([r_[k] for r_ in rows])
        worst = lambda k: np.max([r_[k] for r_ in rows])
        lines.append(f"| {label} | {sum(r_['success'] for r_ in rows)}/{len(rows)} |"
                     f" {100 * mean('rms_ey_true'):.2f} | {100 * worst('max_ey_true'):.2f} |"
                     f" {100 * mean('rms_ey_est'):.2f} | {100 * worst('max_ey_est'):.2f} |"
                     f" {mean('rms_dw_fb'):.3f} | {100 * mean('sat_frac'):.1f} |")
        print(lines[-1])
    (OUT / f"compare_{tag}.md").write_text("\n".join(lines) + "\n")


def sweep(args, plant, routes, tag):
    plt = _plt()
    scales = [0.01, 0.03, 0.1, 0.3, 1.0, 3.0, 10.0, 30.0]
    stats = []
    for rs in scales:
        rows = [score(run(name, plant, seed=args.seed + trial, R_scale=rs))
                for name in routes for trial in range(args.trials)]
        m = {k: float(np.mean([r_[k] for r_ in rows])) for k in
             ("rms_ey_true", "rms_ey_est", "rms_dw_fb", "rms_wheel_acc", "sat_frac")}
        m["max_ey_true"] = float(np.max([r_["max_ey_true"] for r_ in rows]))
        m["docked_ok"] = sum(r_["success"] for r_ in rows) / len(rows)
        m["R_scale"] = rs
        stats.append(m)
        print(f"R x {rs:6.2f}: rms e_y true {100 * m['rms_ey_true']:.2f} cm, seen {100 * m['rms_ey_est']:.2f} cm,"
              f" feedback w {m['rms_dw_fb']:.3f} rad/s, wheel acc {m['rms_wheel_acc']:.1f} rad/s^2,"
              f" limit hit {100 * m['sat_frac']:.1f} %, docked ok {100 * m['docked_ok']:.0f} %")
    write_csv(stats, OUT / f"sweep_{tag}.csv")
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    effort = [s["rms_wheel_acc"] for s in stats]
    ax[0].plot(effort, [100 * s["rms_ey_est"] for s in stats], "s--", color="C3", label="what the controller sees")
    ax[0].plot(effort, [100 * s["rms_ey_true"] for s in stats], "o-", color="C0", label="true")
    for s in stats:
        ax[0].annotate(f"R x{s['R_scale']:g}", (s["rms_wheel_acc"], 100 * s["rms_ey_true"]), fontsize=7,
                       xytext=(3, 3), textcoords="offset points")
    ax[0].set_xscale("log")
    ax[0].set_xlabel("RMS wheel acceleration (rad/s^2), i.e. control effort")
    ax[0].set_ylabel("RMS cross-track error (cm)")
    ax[0].set_title("effort vs tracking error, Q fixed, R scaled", fontsize=10)
    ax[0].legend(fontsize=8)
    ax[1].semilogx(scales, [100 * s["sat_frac"] for s in stats], "o-")
    ax[1].axvline(1.0, color="k", ls=":", lw=0.8)
    ax[1].set_xlabel("R scale (1 = Bryson design)")
    ax[1].set_ylabel("% of steps a wheel hit the limit")
    ax[1].set_title("wheel saturation", fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / f"sweep_{tag}.png", dpi=130)
    plt.close(fig)


def report(args):
    # The batch results PART_D_CONTROL.md quotes, with the same seeds, so anyone can re-make
    # them. The rendered-camera trials are a separate command (listed there) because each
    # run takes about 100 s. With scipy this takes about 8 min on the kinematic stand-in,
    # and about 20 min in MuJoCo with --camera synthetic.
    t0 = time.time()
    plant = make_plant(args)

    def job(label, **overrides):
        sub = argparse.Namespace(**{**vars(args), **overrides})
        tag = make_tag(sub)
        print(f"\n== {label}  ({(time.time() - t0) / 60:.1f} min in)")
        same_plant = tuple(sub.wheel_gain) == (1.0, 1.0) and sub.as_is == args.as_is
        p = plant if same_plant else make_plant(sub, wheel_gain=sub.wheel_gain)
        if sub.sweep:
            sweep(sub, p, list(ROUTES), tag)
        elif sub.compare:
            compare(sub, p, list(ROUTES), tag)
        else:
            trials(sub, p, list(ROUTES), tag)

    for name in ROUTES:
        same, steps, _ = leak_check(name, plant)
        print(f"leak check, {name}: wheel commands identical with truth() returning NaN: {same} ({steps} steps)")
    job("nominal run of each route", trials=1, seed=0)
    job("30 random trials per route", trials=30, seed=1000)
    job("design comparison", trials=10, seed=2000, compare=True)
    job("R sweep", trials=5, seed=3000, sweep=True)
    job("both motors fine, same seeds as the weak-motor runs", trials=10, seed=4000)
    job("left motor at 90 %", trials=10, seed=4000, wheel_gain=(0.9, 1.0))
    job("left motor at 80 %", trials=10, seed=4000, wheel_gain=(0.8, 1.0))
    if args.plant == "mujoco" and not args.as_is:
        job("robot exactly as the xml says (it slips)", trials=30, seed=1000, as_is=True)
    print(f"\nall done in {(time.time() - t0) / 60:.1f} min, results in {OUT}")


def check_sim(args):
    # Pre-flight checks for the MuJoCo scene. Run this once before trusting the closed loop.
    # 1. wheel signs and slip: drive straight, then spin, and compare odometry with MuJoCo
    plant = MujocoTB3(camera="render", as_is=args.as_is)
    print("robot model:", "exactly as the xml says" if args.as_is else "with fix_floor_contacts()")
    rng = np.random.default_rng(0)
    for label, wl, wr, steps in (("straight 4 s", 4.0, 4.0, 200), ("spin 3 s", -3.0, 3.0, 150)):
        plant.reset(np.zeros(3))
        x_odo = np.zeros(3)
        enc_prev = read_encoders(plant.wheel_angles(), rng)
        for _ in range(steps):
            plant.drive(wl, wr)
            enc = read_encoders(plant.wheel_angles(), rng)
            x_odo = model.f(x_odo, odometry(enc, enc_prev), DT)
            enc_prev = enc
        true = plant.truth()
        print(f"{label:12s} odometry x {x_odo[0]:+.3f} y {x_odo[1]:+.3f} th {np.degrees(x_odo[2]):+7.2f} deg |"
              f" MuJoCo x {true[0]:+.3f} y {true[1]:+.3f} th {np.degrees(true[2]):+7.2f} deg")
    print("  (should agree to a few mm / a degree or two; a big gap means wheel slip or a sign error)\n")
    # 2. the marker reading at known spots in front of the dock, against the geometry
    #    (every spot is clear of the boxes, with a clear line of sight to the marker)
    print("   x      y   th(deg) |  measured range, bearing |  expected range, bearing")
    for x, y, th in [(0.8, -0.3, 5.0), (1.0, 0.0, 0.0), (1.8, 0.0, 0.0), (1.8, 0.4, -10.0),
                     (1.8, -0.4, 15.0), (2.4, 0.0, 0.0), (2.6, 0.05, 3.0), (2.7, 0.0, 0.0)]:
        pose = np.array([x, y, np.radians(th)])
        plant.reset(pose)
        z = plant.marker(rng)
        want = ekf.h(plant.truth(), MARKER)
        got = "not seen" if z is None else f"{z[0]:.3f} m, {np.degrees(z[1]):+6.2f} deg"
        print(f"{x:5.2f} {y:6.2f} {th:7.1f} | {got:>24s} | {want[0]:.3f} m, {np.degrees(want[1]):+6.2f} deg"
              f"   (expect seen: {marker_visible(pose)})")


def main():
    p = argparse.ArgumentParser(description="Part D closed-loop tests")
    p.add_argument("--plant", choices=["mujoco", "kinematic"], default="mujoco")
    p.add_argument("--camera", choices=["render", "synthetic"], default="render",
                   help="MuJoCo only: find the marker in the rendered image, or fake the reading")
    p.add_argument("--route", choices=list(ROUTES) + ["all"], default="all")
    p.add_argument("--trials", type=int, default=1)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--compare", action="store_true")
    p.add_argument("--sweep", action="store_true")
    p.add_argument("--leak-check", action="store_true")
    p.add_argument("--report", action="store_true", help="re-make the batch results PART_D_CONTROL.md quotes")
    p.add_argument("--check-sim", action="store_true")
    p.add_argument("--viewer", action="store_true")
    p.add_argument("--wheel-gain", type=float, nargs=2, default=(1.0, 1.0), metavar=("LEFT", "RIGHT"),
                   help="motor mismatch, e.g. 0.9 1.0 = left wheel only gives 90 %% of what it's told")
    p.add_argument("--as-is", action="store_true",
                   help="MuJoCo only: load the robot exactly as the xml says (it slips, see fix_floor_contacts)")
    args = p.parse_args()
    OUT.mkdir(exist_ok=True)
    routes = list(ROUTES) if args.route == "all" else [args.route]
    tag = make_tag(args)

    if args.check_sim:
        return check_sim(args)
    if args.report:
        return report(args)
    if args.viewer:
        import mujoco.viewer
        plant = MujocoTB3(camera=args.camera, wheel_gain=args.wheel_gain, as_is=args.as_is)
        with mujoco.viewer.launch_passive(plant.model, plant.data) as viewer:
            res = run(routes[0], plant, seed=args.seed, viewer=viewer)
        return print(score(res))

    plant = make_plant(args, wheel_gain=args.wheel_gain)
    if args.leak_check:
        for name in routes:
            same, steps, blind = leak_check(name, plant, seed=args.seed)
            print(f"{name}: wheel commands identical with truth() returning NaN: {same} ({steps} steps;"
                  f" the second run's log really was all NaN: {blind})")
        return
    if args.sweep:
        return sweep(args, plant, routes, tag)
    if args.compare:
        return compare(args, plant, routes, tag)
    trials(args, plant, routes, tag)


if __name__ == "__main__":
    main()
