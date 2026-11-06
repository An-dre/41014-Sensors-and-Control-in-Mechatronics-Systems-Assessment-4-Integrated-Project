# Checks for Part D: the error model, LQR, wheel limits, the reference, docking logic,
# route clearance, and that ground truth never reaches the controller.
# From the repository root:
#   python -m unittest part_d_control.test_controller -v
#
# These check the maths and the logic. The closed-loop results (tracking error, docking
# accuracy, effort) come from run_tracking.py, not from here.

import ast
import inspect
import sys
import unittest
from pathlib import Path

import numpy as np

if __package__ in (None, ""):  # run as a file, not with -m: let Python find the part_* packages
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from part_a_modelling import model
from part_d_control import controller as ctl
from part_d_control import run_tracking as rt


def mujoco_available():
    try:
        import mujoco  # noqa: F401
    except ImportError:
        return False
    return True


class TestErrorModel(unittest.TestCase):
    def test_signs(self):
        # A4's sign check: behind the reference (e_x > 0), more speed must shrink e_x
        A, B = ctl.error_model(0.15, 0.0)
        e_next = A @ np.array([0.1, 0.0, 0.0]) + B @ np.array([0.05, 0.0])
        self.assertLess(e_next[0], 0.1)
        # reference on the left (e_y > 0) and the robot already turned towards it
        # (e_th < 0): just driving on must shrink e_y
        e_next = A @ np.array([0.0, 0.1, -0.2]) + B @ np.zeros(2)
        self.assertLess(e_next[1], 0.1)

    def test_tracking_error_is_in_the_robot_frame(self):
        # robot at the origin facing +y: a reference 1 m along +y is straight ahead,
        # and one at x = -1 is on its left
        x_hat = np.array([0.0, 0.0, np.pi / 2])
        np.testing.assert_allclose(ctl.tracking_error(x_hat, [0.0, 1.0, np.pi / 2]), [1.0, 0.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(ctl.tracking_error(x_hat, [-1.0, 0.0, np.pi / 2]), [0.0, 1.0, 0.0], atol=1e-12)
        # +179 vs -179 degrees is a 2 degree error, not 358
        e = ctl.tracking_error([0, 0, np.radians(179)], [0, 0, np.radians(-179)])
        self.assertAlmostEqual(e[2], np.radians(2))

    def test_controllability_needs_motion(self):
        A0, B0 = ctl.error_model(0.0, 0.0)
        A1, B1 = ctl.error_model(0.15, 0.0)
        self.assertEqual(np.linalg.matrix_rank(ctl.controllability_matrix(A0, B0)), 2)
        self.assertEqual(np.linalg.matrix_rank(ctl.controllability_matrix(A1, B1)), 3)


class TestLQR(unittest.TestCase):
    def test_riccati_solution(self):
        A, B = ctl.error_model(0.15, 0.3)
        K, P = ctl.dlqr(A, B, ctl.Q_LQR, ctl.R_LQR)
        Q, R = ctl.Q_LQR, ctl.R_LQR
        residual = A.T @ P @ A - A.T @ P @ B @ np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A) + Q - P
        self.assertLess(np.abs(residual).max(), 1e-6 * np.abs(P).max())

    def test_stable_everywhere_the_schedule_is_used(self):
        # every (v_ref, w_ref) the reference can ask for gives a stable closed loop
        sched = ctl.GainSchedule()
        for v in np.linspace(ctl.V_SCHED_MIN, ctl.V_MAX, 8):
            for w in np.linspace(-1.2, 1.2, 9):
                A, B = ctl.error_model(v, w)
                z = np.linalg.eigvals(A - B @ sched.gain(v, w))
                self.assertTrue(np.all(np.abs(z) < 1), f"unstable at v={v:.2f}, w={w:.2f}: {np.abs(z)}")

    def test_cache_and_fixed_gain(self):
        sched = ctl.GainSchedule()
        np.testing.assert_array_equal(sched.gain(0.15, 0.2), sched.gain(0.1502, 0.2004))
        fixed = ctl.GainSchedule(scheduled=False)
        np.testing.assert_array_equal(fixed.gain(0.05, 0.9), fixed.gain(0.2, -0.5))


class TestWheels(unittest.TestCase):
    def test_inside_the_limit_passes_straight_through(self):
        wl, wr, scale = ctl.wheel_speeds(0.165, 0.0)
        self.assertEqual(scale, 1.0)
        np.testing.assert_allclose([wl, wr], [5.0, 5.0])

    def test_saturation_keeps_the_curve(self):
        v, w = 0.2, 1.5                       # asks for more than a wheel can give
        wl, wr, scale = ctl.wheel_speeds(v, w)
        self.assertLess(scale, 1.0)
        self.assertAlmostEqual(max(abs(wl), abs(wr)), ctl.WHEEL_LIMIT)
        v_out = model.r * (wr + wl) / 2
        w_out = model.r * (wr - wl) / model.L
        self.assertAlmostEqual(w_out / v_out, w / v)   # same curvature, just slower


class TestReference(unittest.TestCase):
    def setUp(self):
        self.marker, self.normal = np.array([2.888, 0.0]), np.array([-1.0, 0.0])
        self.ref = ctl.docking_reference([(-2.0, -1.0), (-1.0, -1.0), (0.0, 0.5), (1.0, 0.5)],
                                         self.marker, self.normal)

    def test_smooth_and_inside_the_limits(self):
        ref = self.ref
        self.assertLess(np.abs(np.diff(ref.th)).max(), 0.05)        # no heading jumps
        step = np.hypot(np.diff(ref.x), np.diff(ref.y))
        self.assertLess(step.max(), ctl.V_MAX * ctl.DT + 1e-6)     # never faster than it can go
        self.assertLessEqual(ref.v.max(), ctl.V_CRUISE + 1e-9)
        dv = np.diff(ref.v) / ctl.DT
        self.assertLess(np.abs(dv).max(), ctl.A_MAX * 1.05)
        wl = (ref.v - ref.w * model.L / 2) / model.r
        wr = (ref.v + ref.w * model.L / 2) / model.r
        self.assertLess(np.maximum(abs(wl), abs(wr)).max(), ctl.WHEEL_SHARE * model.W_MAX + 1e-6)
        self.assertEqual(ref.v[0], 0.0)
        self.assertAlmostEqual(ref.v[-1], 0.0, places=6)

    def test_ends_docked_facing_the_marker(self):
        end = self.ref.end_pose()
        _, p_dock, heading = ctl.dock_targets(self.marker, self.normal)
        np.testing.assert_allclose(end[:2], p_dock, atol=1e-9)
        self.assertAlmostEqual(ctl.wrap(end[2] - heading), 0.0, places=6)
        # last DOCK_SLOW metres at creep speed, straight in
        last = self.ref.s > self.ref.length - ctl.DOCK_SLOW
        self.assertLessEqual(self.ref.v[last].max(), ctl.V_DOCK + 1e-9)
        self.assertLess(np.abs(self.ref.w[last]).max(), 1e-9)

    def test_rejects_routes_it_cannot_round(self):
        with self.assertRaises(ValueError):
            ctl.Reference([(0, 0), (1, 0), (1.0, 0.02), (2, 0)])
        with self.assertRaises(ValueError):
            ctl.Reference([(0, 0), (0, 0), (1, 0)])


class TestTracker(unittest.TestCase):
    def drive(self, start, route, steps=6000):
        # A perfect unicycle with a perfect estimate, so only the controller's logic is
        # being tested here. The real pipeline (EKF, noise, MuJoCo) is run_tracking.py.
        ref = ctl.docking_reference(route, [2.888, 0.0], [-1.0, 0.0])
        tracker = ctl.LQRTracker(ref)
        x = np.array(start, dtype=float)
        modes = []
        for k in range(steps):
            v, w = tracker.step(x, k * ctl.DT)
            modes.append(tracker.mode)
            if tracker.docked:
                break
            x = model.f(x, [v, w], ctl.DT)
            x[2] = ctl.wrap(x[2])
        return x, ref, modes

    def test_docks_from_an_offset_start(self):
        x, ref, modes = self.drive([-1.0, 0.08, np.radians(8)], [(-1.0, 0.0), (0.5, 0.0)])
        self.assertEqual(modes[-1], "docked")
        e = ctl.tracking_error(x, ref.end_pose())
        self.assertLess(abs(e[0]), 0.012)
        self.assertLess(abs(e[1]), 0.01)
        self.assertLess(abs(e[2]), np.radians(2.5))

    def test_turns_on_the_spot_first_when_facing_the_wrong_way(self):
        x, ref, modes = self.drive([0.0, -1.0, np.pi], [(0.0, -1.0), (1.0, -1.0)])
        self.assertEqual(modes[0], "align")
        self.assertIn("track", modes)
        self.assertEqual(modes[-1], "docked")


class TestGroundTruth(unittest.TestCase):
    def test_truth_never_reaches_the_controller(self):
        # Same run twice on the fast stand-in, the second with the plant's truth() swapped
        # for one that only returns NaN. If the true pose leaked into the control path the
        # wheel commands would differ (or turn into NaN).
        for route in rt.ROUTES:
            same, steps, blind = rt.leak_check(route, rt.KinematicTB3(), seed=3)
            self.assertTrue(blind)                  # the second run really couldn't see it
            self.assertTrue(same, f"{route}: commands changed when truth() went blind")
            self.assertGreater(steps, 1000)

    def test_leak_check_catches_a_planted_leak(self):
        # A check that can't fail proves nothing, so cheat on purpose (hand the controller
        # the true pose instead of the estimate) and make sure leak_check notices.
        plant = rt.KinematicTB3()
        honest_step = ctl.LQRTracker.step
        ctl.LQRTracker.step = lambda tracker, x_hat, t: honest_step(tracker, plant.truth(), t)
        try:
            same, _, _ = rt.leak_check("south", plant, seed=3)
        finally:
            ctl.LQRTracker.step = honest_step
        self.assertFalse(same)

    def test_controller_cannot_see_the_simulator(self):
        # controller.py only imports numpy, scipy and Part A's model, never names anything
        # from the simulator, and the tracker's only inputs are the estimate and the time.
        tree = ast.parse(inspect.getsource(ctl))
        imported = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
        imported |= {n.module.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
        self.assertLessEqual(imported, {"sys", "pathlib", "numpy", "scipy", "part_a_modelling"})
        names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
        names |= {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
        for banned in ("truth", "_pose", "qpos", "sensordata", "mujoco", "plant", "run_tracking", "sim"):
            self.assertNotIn(banned, names)
        self.assertEqual(list(inspect.signature(ctl.LQRTracker.step).parameters), ["self", "x_hat", "t"])

    def test_routes_keep_clear_of_the_boxes(self):
        others = [n for n in rt.BOXES if not n.startswith("dock")]
        for name in rt.ROUTES:
            ref = rt.route_reference(name)
            before_dock = ref.s < ref.length - ctl.DOCK_APPROACH
            pts = np.c_[ref.x, ref.y][before_dock]
            for box in others:
                c, h = rt.BOXES[box]
                gap = np.linalg.norm(np.maximum(np.abs(pts - c) - h, 0.0), axis=1).min()
                self.assertGreaterEqual(gap, rt.ROUTE_CLEARANCE, f"{name} passes {gap:.2f} m from {box}")


@unittest.skipUnless(mujoco_available(), "MuJoCo isn't installed")
class TestMujocoScene(unittest.TestCase):
    # Checks the robot rolls the way its encoders say, which it doesn't without the
    # floor-contact correction (15 % short driving straight, section 3.1 of the report).
    def slip(self, as_is, w_left, w_right, steps=150):
        plant = rt.MujocoTB3(camera="synthetic", as_is=as_is)
        plant.reset(np.zeros(3))
        start = plant.wheel_angles()
        for _ in range(steps):
            plant.drive(w_left, w_right)
        dphi = plant.wheel_angles() - start
        true = plant.truth()
        said_d = model.r * dphi.mean()
        said_th = model.r * (dphi[1] - dphi[0]) / model.L
        return np.hypot(true[0], true[1]) / said_d - 1 if abs(said_d) > 1e-3 else true[2] / said_th - 1

    def test_wheels_roll_without_slipping(self):
        self.assertLess(abs(self.slip(False, 4.0, 4.0)), 0.02)     # straight
        self.assertLess(abs(self.slip(False, -3.0, 3.0)), 0.02)    # spin on the spot

    def test_truth_never_reaches_the_controller_in_mujoco(self):
        # the leak check again, this time in the real scene (fake camera, so it runs in a few
        # seconds; run_tracking --leak-check does it with the rendered camera)
        same, steps, blind = rt.leak_check("south", rt.MujocoTB3(camera="synthetic"), seed=3)
        self.assertTrue(blind)
        self.assertTrue(same)


if __name__ == "__main__":
    unittest.main()
