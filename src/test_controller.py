# Core Part D checks: stability, wheel limits, docking, route clearance and truth isolation.

import unittest
import numpy as np
import controller as ctl
import model


class TestErrorModel(unittest.TestCase):
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




class TestWheels(unittest.TestCase):
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




class TestPipeline(unittest.TestCase):
    def test_routes_keep_clear_of_the_boxes(self):
        import run_tracking as rt
        others = [n for n in rt.BOXES if not n.startswith("dock")]
        for name in rt.ROUTES:
            ref = rt.route_reference(name)
            before_dock = ref.s < ref.length - ctl.DOCK_APPROACH
            pts = np.c_[ref.x, ref.y][before_dock]
            for box in others:
                c, h = rt.BOXES[box]
                gap = np.linalg.norm(np.maximum(np.abs(pts - c) - h, 0.0), axis=1).min()
                self.assertGreaterEqual(gap, rt.ROUTE_CLEARANCE, f"{name} passes {gap:.2f} m from {box}")




class TestMujocoScene(unittest.TestCase):
    def slip(self, as_is, w_left, w_right, steps=150):
        try:
            import mujoco  # noqa: F401
        except ImportError:
            self.skipTest("MuJoCo isn't installed")
        import run_tracking as rt
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
        # seconds; run_tracking.py --leak-check does it with the rendered camera)
        try:
            import mujoco  # noqa: F401
        except ImportError:
            self.skipTest("MuJoCo isn't installed")
        import run_tracking as rt
        same, steps, blind = rt.leak_check("south", rt.MujocoTB3(camera="synthetic"), seed=3)
        self.assertTrue(blind)
        self.assertTrue(same)



if __name__ == "__main__":
    unittest.main()
