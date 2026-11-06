# controller.py - PART D (20 marks). Owner: Andre.
# LQR trajectory tracker for the TurtleBot3 Waffle Pi, running on the EKF estimate.
#
# The controller only ever gets x_hat = [x, y, theta] from ekf.py. It never reads
# the simulator's true pose. run_tracking.py logs the true pose separately, only to
# score each run after it finishes.
#
# Every control step (dt = 0.02 s, the same 50 Hz the EKF runs at):
#   1. look up the reference at time t: pose (x_r, y_r, th_r) and u_ref = (v_ref, w_ref)
#   2. tracking error e = reference - estimate, rotated into the robot frame (A4 in model.py)
#   3. LQR gain K for this (v_ref, w_ref), from the linear error model
#   4. u = u_ref - K e   (feedforward drives the plan, feedback fixes the error)
#   5. if either wheel would pass its speed limit, slow both by the same factor
#
# Quick check, no simulator:   python -m part_d_control.controller
# Unit checks:                 python -m unittest part_d_control.test_controller -v

import sys
from pathlib import Path

import numpy as np
from scipy.linalg import solve_discrete_are

if __package__ in (None, ""):  # run as a file, not with -m: let Python find the part_* packages
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from part_a_modelling.model import r, L, W_MAX, linear_error_model, body_to_wheels

DT = 0.02                    # control period (s), matches the EKF's Q
V_MAX = r * W_MAX            # 0.26 m/s with both wheels flat out
WHEEL_LIMIT = 0.95 * W_MAX   # I keep 5 % spare so the actuator itself never clips
V_REVERSE = 0.05             # most I'll let it back up (m/s), only to fix overshoot
FRONT = 0.086                # base origin to the front of the camera bracket (m), robot xml


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# --- D1. tracking error and its linear model ---------------------------------------
# Use Part A's model so control shares the group's robot definition.
def error_model(v_ref, w_ref, dt=DT):
    return linear_error_model(v_ref, w_ref, dt)


def tracking_error(x_hat, pose_ref):
    # reference minus estimate, turned into the robot frame: [ahead, left, heading]
    dx = pose_ref[0] - x_hat[0]
    dy = pose_ref[1] - x_hat[1]
    c, s = np.cos(x_hat[2]), np.sin(x_hat[2])
    return np.array([c * dx + s * dy,
                     -s * dx + c * dy,
                     wrap(pose_ref[2] - x_hat[2])])


def controllability_matrix(A, B):
    return np.hstack([B, A @ B, A @ A @ B])


# --- D2. discrete LQR --------------------------------------------------------------
# Week 6: solve the discrete Riccati equation, then apply du = -K e.
def dlqr(A, B, Q, R):
    P = solve_discrete_are(A, B, Q, R)
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    return K, P


# --- D3. weights, Bryson's rule: 1 / (largest value I can accept)^2 -------------------
E_X_MAX = 0.10              # m     along the path. Being a bit late is harmless.
E_Y_MAX = 0.05              # m     across the path. The routes leave about 0.25 m between
                            #       the robot's side and the boxes, and docking needs the
                            #       robot centred, so this is the one I care about most.
E_TH_MAX = np.radians(10)   # rad   heading
DV_MAX = 0.10               # m/s   speed on top of the plan (cruise 0.15, top speed 0.26)
DW_MAX = 0.60               # rad/s turn rate on top of the plan. At 0.15 m/s one wheel
                            #       saturates once |w| > 0.76 rad/s.
Q_LQR = np.diag([1 / E_X_MAX**2, 1 / E_Y_MAX**2, 1 / E_TH_MAX**2])
R_LQR = np.diag([1 / DV_MAX**2, 1 / DW_MAX**2])


# --- D4. gain schedule -------------------------------------------------------------
# A depends on v_ref and w_ref, so I redo the LQR for whatever the reference is doing,
# and cache it so each (v, w) pair is only solved once.
# At v_ref = 0 the sideways error is uncontrollable (rank 2, see A5), and the Riccati
# equation has no stabilising answer. So I never design for slower than V_SCHED_MIN.
V_SCHED_MIN = 0.05


class GainSchedule:
    def __init__(self, Q=Q_LQR, R=R_LQR, dt=DT, scheduled=True, v_fixed=0.15):
        self.Q, self.R, self.dt = Q, R, dt
        self.scheduled = scheduled   # False = one gain for straight driving at v_fixed
        self.v_fixed = v_fixed
        self.cache = {}

    def gain(self, v_ref, w_ref):
        if not self.scheduled:
            v_ref, w_ref = self.v_fixed, 0.0
        v = float(np.clip(v_ref, V_SCHED_MIN, V_MAX))
        key = (round(v / 0.005), round(w_ref / 0.01))
        if key not in self.cache:
            A, B = error_model(key[0] * 0.005, key[1] * 0.01, self.dt)
            self.cache[key], _ = dlqr(A, B, self.Q, self.R)
        return self.cache[key]


# --- D5. the reference: straights joined by arcs, timed with a speed profile -------
# The planner gives waypoints. I round each corner with an arc so the heading never
# jumps, then work out how fast the robot should be at every point on the path:
#   * cruise speed on the straights
#   * slower on arcs, so turning plus feedback still fits inside the wheel limit
#   * a slow creep for the last part into the dock
#   * start and finish at rest, with the acceleration limited in between
V_CRUISE = 0.15         # m/s
A_MAX = 0.15            # m/s^2
TURN_RADIUS = 0.25      # m, corner rounding
V_DOCK = 0.05           # m/s for the last DOCK_SLOW metres
DOCK_SLOW = 0.30        # m
WHEEL_SHARE = 0.70      # the plan may use 70 % of the wheel speed, feedback gets the rest
MIN_FINAL_STRAIGHT = 0.5  # m of straight line I always keep before the dock


def _unit(v):
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError("two waypoints are on top of each other")
    return v / n


class Reference:
    def __init__(self, waypoints, v_cruise=V_CRUISE, a_max=A_MAX, turn_radius=TURN_RADIUS,
                 v_end=V_DOCK, slow_zone=DOCK_SLOW, final_straight=MIN_FINAL_STRAIGHT,
                 dt=DT, ds=0.002):
        P = np.asarray(waypoints, dtype=float)
        if len(P) < 2:
            raise ValueError("need at least two waypoints")
        self.waypoints = P
        s, x, y, th, kappa = self._path(P, turn_radius, final_straight, ds)

        # speed limit at each point on the path. Each point also takes the curvature of its
        # neighbours, so the points either side of a straight/arc join both get the arc's
        # limit (otherwise resampling between them can go a few % over the wheel budget).
        k_near = np.abs(kappa)
        k_near = np.maximum(k_near, np.maximum(np.r_[k_near[1:], 0.0], np.r_[0.0, k_near[:-1]]))
        v_lim = np.minimum(v_cruise, WHEEL_SHARE * V_MAX / (1 + k_near * L / 2))
        creep = s[-1] - s <= slow_zone + ds
        v_lim[creep] = np.minimum(v_lim[creep], v_end)
        v_lim[0] = v_lim[-1] = 0.0
        # forward pass caps acceleration, backward pass caps braking
        v = v_lim.copy()
        step = np.diff(s)
        for i in range(1, len(v)):
            v[i] = min(v[i], np.sqrt(v[i - 1]**2 + 2 * a_max * step[i - 1]))
        for i in range(len(v) - 2, -1, -1):
            v[i] = min(v[i], np.sqrt(v[i + 1]**2 + 2 * a_max * step[i]))
        # time to cover each little piece of path at the average of its end speeds
        t = np.concatenate([[0.0], np.cumsum(2 * step / (v[:-1] + v[1:]))])

        # resample on the control clock so lookups are cheap
        self.dt = dt
        self.t = np.arange(0.0, t[-1] + dt, dt)
        s_t = np.interp(self.t, t, s)
        self.x = np.interp(s_t, s, x)
        self.y = np.interp(s_t, s, y)
        self.th = np.interp(s_t, s, np.unwrap(th))
        self.v = np.interp(s_t, s, v)
        idx = np.clip(np.searchsorted(s, s_t), 0, len(s) - 1)
        self.w = self.v * kappa[idx]          # feedforward turn rate = speed x curvature
        self.s = s_t
        self.length = s[-1]
        self.duration = self.t[-1]

    @staticmethod
    def _path(P, radius, final_straight, ds):
        n = len(P)
        seg = [np.linalg.norm(P[i + 1] - P[i]) for i in range(n - 1)]
        # how much of each leg a corner arc may eat. A middle leg is shared by the
        # corners at both its ends, so each gets half. The first leg only has one
        # corner. On the last leg I keep final_straight metres of straight line.
        room = [length / 2 for length in seg]
        if n > 2:
            room[0] = seg[0]
            room[-1] = seg[-1] - final_straight
            if room[-1] <= 0:
                raise ValueError(f"the last leg is shorter than final_straight = {final_straight} m")

        pieces = []
        start = P[0]
        for i in range(1, n - 1):
            d_in, d_out = _unit(P[i] - P[i - 1]), _unit(P[i + 1] - P[i])
            turn = np.arctan2(d_in[0] * d_out[1] - d_in[1] * d_out[0], d_in @ d_out)  # + = left
            if abs(turn) < 1e-6:
                continue
            tangent = min(radius * np.tan(abs(turn) / 2), room[i - 1], room[i])
            rad = tangent / np.tan(abs(turn) / 2)
            if rad < 0.05:
                raise ValueError(f"no room to round the corner at waypoint {i}, spread the points out")
            a = P[i] - d_in * tangent                      # where the arc starts
            left = np.array([-d_in[1], d_in[0]])
            centre = a + np.sign(turn) * rad * left
            pieces.append(("line", start, a))
            pieces.append(("arc", centre, rad, np.arctan2(d_in[1], d_in[0]), turn))
            start = P[i] + d_out * tangent                 # where the arc ends
        pieces.append(("line", start, P[-1]))

        first = P[1] - P[0]
        S, X, Y = [0.0], [P[0][0]], [P[0][1]]
        TH, KA = [np.arctan2(first[1], first[0])], [0.0]
        for piece in pieces:
            if piece[0] == "line":
                _, p0, p1 = piece
                length = np.linalg.norm(p1 - p0)
                if length < 1e-9:
                    continue
                count = max(1, int(np.ceil(length / ds)))
                f = np.arange(1, count + 1) / count
                X += list(p0[0] + f * (p1[0] - p0[0]))
                Y += list(p0[1] + f * (p1[1] - p0[1]))
                TH += [np.arctan2(p1[1] - p0[1], p1[0] - p0[0])] * count
                KA += [0.0] * count
                S += list(S[-1] + f * length)
            else:
                _, c, rad, h0, turn = piece
                sign = np.sign(turn)
                length = rad * abs(turn)
                count = max(2, int(np.ceil(length / ds)))
                h = h0 + np.arange(1, count + 1) / count * turn
                X += list(c[0] + sign * rad * np.sin(h))
                Y += list(c[1] - sign * rad * np.cos(h))
                TH += list(h)
                KA += [sign / rad] * count
                S += list(S[-1] + np.arange(1, count + 1) / count * length)
        return (np.array(S), np.array(X), np.array(Y), np.array(TH), np.array(KA))

    def sample(self, tau):
        # reference pose and feedforward input at tau seconds into the plan
        k = tau / self.dt
        i = int(np.clip(np.floor(k), 0, len(self.t) - 1))
        j = min(i + 1, len(self.t) - 1)
        f = float(np.clip(k - i, 0.0, 1.0))
        pose = np.array([self.x[i] + f * (self.x[j] - self.x[i]),
                         self.y[i] + f * (self.y[j] - self.y[i]),
                         wrap(self.th[i] + f * (self.th[j] - self.th[i]))])
        return pose, self.v[i] + f * (self.v[j] - self.v[i]), self.w[i]

    def end_pose(self):
        return np.array([self.x[-1], self.y[-1], wrap(self.th[-1])])


# --- D6. docking targets -----------------------------------------------------------
# The last leg comes straight in along the marker's normal. That keeps the marker in
# the middle of the camera image (so the EKF keeps getting fixes) and it lines the
# robot up with the dock before it arrives.
DOCK_STANDOFF = 0.05    # gap between the robot's front and the marker face when docked (m)
DOCK_APPROACH = 0.90    # length of the straight final leg (m)


def dock_targets(marker_xy, normal_xy, standoff=DOCK_STANDOFF, approach=DOCK_APPROACH):
    # normal_xy points out of the marker face, into the room
    n = _unit(np.asarray(normal_xy, dtype=float))
    m = np.asarray(marker_xy, dtype=float)
    p_dock = m + n * (standoff + FRONT)
    p_pre = p_dock + n * approach
    heading = np.arctan2(-n[1], -n[0])
    return p_pre, p_dock, heading


def docking_reference(route, marker_xy, normal_xy, **kwargs):
    # route = waypoints from the start up to somewhere near the dock (planner output)
    p_pre, p_dock, _ = dock_targets(marker_xy, normal_xy)
    return Reference(np.vstack([np.asarray(route, dtype=float), p_pre, p_dock]), **kwargs)


# --- D7. wheel limits --------------------------------------------------------------
# If one wheel would go past the limit I scale BOTH wheel speeds by the same factor.
# That keeps the ratio v / w, so the robot stays on the curve it was asked for and just
# goes slower, instead of turning less than it should (which clipping one wheel does).
def wheel_speeds(v, w):
    w_left, w_right = body_to_wheels(v, w)
    biggest = max(abs(w_left), abs(w_right))
    scale = 1.0 if biggest <= WHEEL_LIMIT else WHEEL_LIMIT / biggest
    return w_left * scale, w_right * scale, scale


# --- D8. the tracker ---------------------------------------------------------------
ALIGN_START = np.radians(15)   # turn on the spot first if the start heading is off by more
ALIGN_DONE = np.radians(3)
K_ALIGN, W_ALIGN = 2.0, 0.8    # rad/s per rad, and the most it may turn while aligning
DOCK_TOL_X = 0.01              # m, along the dock axis
DOCK_TOL_TH = np.radians(2)
DOCK_HOLD = 0.5                # s inside tolerance before I call it docked
SETTLE_TIMEOUT = 5.0           # s, stop anyway after this


class LQRTracker:
    # scheduled=False and feedforward=False are only there so run_tracking.py --compare
    # can show what each piece is worth. The final design uses both.
    def __init__(self, reference, scheduled=True, feedforward=True, Q=Q_LQR, R=R_LQR, dt=DT):
        self.ref = reference
        self.dt = dt
        self.feedforward = feedforward
        self.gains = GainSchedule(Q, R, dt, scheduled=scheduled)
        # once the plan has ended the robot is parked, so only ahead and heading can
        # still be fixed (sideways needs motion). Same weights, 2-state model.
        A_park, B_park = np.eye(2), -dt * np.eye(2)
        self.K_park, _ = dlqr(A_park, B_park, Q[np.ix_([0, 2], [0, 2])], R)
        self.reset()

    def reset(self):
        self.mode = "align"
        self.rotating = False
        self.t0 = None
        self.t_settle = None
        self.t_in_tol = 0.0
        self.last = {}

    @property
    def docked(self):
        return self.mode == "docked"

    def step(self, x_hat, t):
        x_hat = np.asarray(x_hat, dtype=float)
        e = np.zeros(3)
        v_ref = w_ref = 0.0

        if self.mode == "align":
            # if it starts pointing roughly along the path I go straight away, otherwise
            # I turn on the spot until it's within ALIGN_DONE, then start the clock
            e_th = wrap(self.ref.th[0] - x_hat[2])
            if abs(e_th) < (ALIGN_DONE if self.rotating else ALIGN_START):
                self.mode, self.t0 = "track", t
            else:
                self.rotating = True
                e[2] = e_th
                w = float(np.clip(K_ALIGN * e_th, -W_ALIGN, W_ALIGN))
                return self._finish(0.0, w, e, v_ref, w_ref, t)

        if self.mode == "track":
            tau = t - self.t0
            if tau < self.ref.duration:
                pose_ref, v_ref, w_ref = self.ref.sample(tau)
                e = tracking_error(x_hat, pose_ref)
                du = -self.gains.gain(v_ref, w_ref) @ e
                w_ff = w_ref if self.feedforward else 0.0
                return self._finish(v_ref + du[0], w_ff + du[1], e, v_ref, w_ref, t)
            self.mode, self.t_settle = "settle", t

        if self.mode == "settle":
            e = tracking_error(x_hat, self.ref.end_pose())
            du = -self.K_park @ e[[0, 2]]
            v = float(np.clip(du[0], -V_DOCK, V_DOCK))
            w = float(du[1])
            inside = abs(e[0]) < DOCK_TOL_X and abs(e[2]) < DOCK_TOL_TH
            self.t_in_tol = self.t_in_tol + self.dt if inside else 0.0
            if self.t_in_tol >= DOCK_HOLD:
                self.mode = "docked"
            elif t - self.t_settle > SETTLE_TIMEOUT:
                self.mode = "timeout"
            else:
                return self._finish(v, w, e, 0.0, 0.0, t)

        return self._finish(0.0, 0.0, e, 0.0, 0.0, t)

    def _finish(self, v, w, e, v_ref, w_ref, t):
        v = float(np.clip(v, -V_REVERSE, V_MAX))
        _, _, scale = wheel_speeds(v, w)
        v, w = v * scale, w * scale
        self.last = dict(t=t, mode=self.mode, e=e.copy(), v_ref=v_ref, w_ref=w_ref,
                         v=v, w=w, scale=scale)
        return v, w


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    print(f"top speed {V_MAX:.3f} m/s, wheel limit used {WHEEL_LIMIT:.2f} rad/s")

    for v_ref in (0.0, 0.15):
        A, B = error_model(v_ref, 0.0)
        print(f"rank Ctrb at v_ref = {v_ref:.2f}:", np.linalg.matrix_rank(controllability_matrix(A, B)))

    for v_ref, w_ref in ((0.05, 0.0), (0.15, 0.0), (0.115, 0.46)):
        A, B = error_model(v_ref, w_ref)
        K, _ = dlqr(A, B, Q_LQR, R_LQR)
        z = np.linalg.eigvals(A - B @ K)
        print(f"v_ref {v_ref:g} m/s, w_ref {w_ref:g} rad/s: K =\n{K}\n   |eig(A - BK)| = {np.abs(z)}")
        assert np.all(np.abs(z) < 1)

    # what one fixed gain (designed for straight driving) costs in the routes' corners:
    # 0.25 m arcs, driven at 0.115 m/s
    def slowest_time_constant(A, B, K):
        return -DT / np.log(np.abs(np.linalg.eigvals(A - B @ K)).max())
    A, B = error_model(0.115, 0.46)
    print(f"slowest time constant in a corner: scheduled gain"
          f" {slowest_time_constant(A, B, GainSchedule().gain(0.115, 0.46)):.2f} s, one fixed gain"
          f" {slowest_time_constant(A, B, GainSchedule(scheduled=False).gain(0.115, 0.46)):.2f} s")

    # Euler against exact (zero-order hold) discretisation of the same error model
    from scipy.linalg import expm
    worst = 0.0
    for v_ref, w_ref in ((0.05, 0.0), (0.15, 0.0), (0.115, 0.46)):
        A, B = error_model(v_ref, w_ref)
        M = np.zeros((5, 5))
        M[:3, :3], M[:3, 3:] = (A - np.eye(3)) / DT, B / DT     # back to continuous time
        E = expm(M * DT)
        K_euler, _ = dlqr(A, B, Q_LQR, R_LQR)
        K_zoh, _ = dlqr(E[:3, :3], E[:3, 3:], Q_LQR, R_LQR)
        worst = max(worst, np.abs(K_zoh - K_euler).max() / np.abs(K_euler).max())
    print(f"exact (zero-order hold) discretisation changes K by at most {100 * worst:.1f} %")

    # drive the LINEAR error model from a 10 cm sideways, 5 deg heading error
    A, B = error_model(0.15, 0.0)
    K, _ = dlqr(A, B, Q_LQR, R_LQR)
    e = np.array([0.0, 0.10, np.radians(5)])
    for k in range(int(10 / DT)):
        e = A @ e + B @ (-K @ e)
    print("error after 10 s on the linear model:", e, " should be ~0")

    ref = docking_reference([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]], [2.888, 0.0], [-1.0, 0.0])
    print(f"test reference: {ref.length:.2f} m in {ref.duration:.1f} s,"
          f" top speed {ref.v.max():.3f} m/s, top turn rate {np.abs(ref.w).max():.3f} rad/s")
    print("Basic checks passed.")
