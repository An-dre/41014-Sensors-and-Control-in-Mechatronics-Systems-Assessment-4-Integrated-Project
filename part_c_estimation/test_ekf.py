# Checks for Part C. The robot never imports or runs this file.
# From the repository root:
#   python -m unittest part_c_estimation.test_ekf -v
#
# unittest runs each test_ method; a failed assertion reports a failed check.
# assert_allclose compares numbers while allowing small rounding differences.
# Passing these checks verifies examples of the maths, not successful docking
# or suitable Q/R values for the actual robot. Those need separate experiments.

import sys
import unittest
from pathlib import Path

import numpy as np

if __package__ in (None, ""):  # run as a file, not with -m: let Python find the part_* packages
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from part_a_modelling import model
from part_c_estimation import ekf


def numerical_jacobian(function, values):
    # Nudge one input at a time to check the handwritten derivatives.
    step = 1e-6
    result = np.zeros((len(function(values)), len(values)))
    for column in range(len(values)):
        above = values.copy()
        below = values.copy()
        above[column] += step
        below[column] -= step
        result[:, column] = (function(above) - function(below)) / (2 * step)
    return result


class TestEKF(unittest.TestCase):
    def setUp(self):
        # unittest calls this before each test, giving it a fresh starting point.
        ekf.reset_update_counts()
        self.x = np.array([0.0, 0.0, 0.0])
        self.P = np.diag([0.5, 0.5, 0.1])
        self.marker = np.array([2.0, 0.0])

    def test_motion_and_prediction(self):
        # Facing up, forward motion must change y rather than x.
        x = np.array([1.0, 2.0, np.pi / 2])
        result, covariance = ekf.predict(x, self.P, [0.2, 0.1], 0.1)
        np.testing.assert_allclose(result, [1.0, 2.02, np.pi / 2 + 0.01])
        self.assertTrue(np.all(np.linalg.eigvalsh(covariance) > 0))
        np.testing.assert_array_equal(x, [1.0, 2.0, np.pi / 2])

    def test_motion_jacobians(self):
        x = np.array([0.3, -0.4, 0.7])
        u = np.array([0.15, -0.2])
        F_numeric = numerical_jacobian(lambda pose: model.f(pose, u, 0.02), x)
        G_numeric = numerical_jacobian(lambda speed: model.f(x, speed, 0.02), u)
        np.testing.assert_allclose(model.jacobian_F(x, u, 0.02), F_numeric, atol=1e-9)
        np.testing.assert_allclose(model.jacobian_G(x, u, 0.02), G_numeric, atol=1e-9)

    def test_process_noise_at_fixed_rate(self):
        # Q is added once per prediction at the normal 50 Hz rate.
        x1, P1 = ekf.predict(self.x, self.P, [0.0, 0.0], 0.02)
        _, P2 = ekf.predict(x1, P1, [0.0, 0.0], 0.02)
        np.testing.assert_allclose(P1, self.P + ekf.Q)
        np.testing.assert_allclose(P2, self.P + 2 * ekf.Q)
        self.assertTrue(np.all(np.diag(P1) > np.diag(self.P)))

    def test_measurement_and_jacobian(self):
        # A 3-4-5 triangle has range 5; a 90-degree heading changes its bearing.
        np.testing.assert_allclose(
            ekf.h([0.0, 0.0, np.pi / 2], [3.0, 4.0]),
            [5.0, np.arctan2(4.0, 3.0) - np.pi / 2],
        )
        x = np.array([0.3, -0.4, 0.7])
        H_numeric = numerical_jacobian(lambda pose: ekf.h(pose, self.marker), x)
        np.testing.assert_allclose(ekf.jacobian_H(x, self.marker), H_numeric, atol=1e-9)

    def test_update_matches_scalar_range_solution(self):
        # Here range corrects x independently, giving a known scalar solution.
        x2, P2 = ekf.update(self.x, self.P, [1.5, 0.0], self.marker)
        expected_x = 0.5 * 0.5 / (0.5 + ekf.R[0, 0])
        expected_variance = 0.5 * ekf.R[0, 0] / (0.5 + ekf.R[0, 0])
        self.assertAlmostEqual(x2[0], expected_x)
        self.assertAlmostEqual(P2[0, 0], expected_variance)
        self.assertEqual(ekf.updates_accepted, 1)
        np.testing.assert_array_equal(self.x, np.zeros(3))
        np.testing.assert_array_equal(self.P, np.diag([0.5, 0.5, 0.1]))

    def test_bearing_crossing_pi_is_a_small_innovation(self):
        marker = np.array([-2.0, 0.01])
        z = ekf.h(self.x, marker)
        z[1] = -np.pi + 0.005
        x2, _ = ekf.update(self.x, self.P, z, marker)
        self.assertEqual(ekf.updates_accepted, 1)
        self.assertLess(abs(x2[2]), 0.02)

    def test_outlier_is_rejected(self):
        x2, P2 = ekf.update(self.x, self.P, [20.0, 0.0], self.marker)
        np.testing.assert_array_equal(x2, self.x)
        np.testing.assert_array_equal(P2, self.P)
        self.assertEqual(ekf.updates_rejected, 1)

    def test_invalid_measurements_and_zero_distance(self):
        for z in [[np.nan, 0.0], [2.0, np.inf], [-1.0, 0.0], [0.0, 0.0]]:
            x2, P2 = ekf.update(self.x, self.P, z, self.marker)
            np.testing.assert_array_equal(x2, self.x)
            np.testing.assert_array_equal(P2, self.P)
        self.assertEqual(ekf.updates_rejected, 4)
        with self.assertRaises(ValueError):
            ekf.update(self.x, self.P, [1.0, 0.0], self.x[:2])
        with self.assertRaises(ValueError):
            ekf.jacobian_H(self.x, self.x[:2])

    def test_heading_wrap_and_nees(self):
        # +179 and -179 degrees differ by 2 degrees, rather than 358.
        x_hat = np.array([0.0, 0.0, np.radians(179)])
        x_true = np.array([0.0, 0.0, np.radians(-179)])
        P = np.diag([1.0, 1.0, np.radians(1)**2])
        self.assertAlmostEqual(ekf.nees(x_hat, P, x_true), 4.0)
        self.assertEqual(ekf.nees(self.x, self.P, self.x), 0.0)
        # Integer inputs must still preserve fractional radians after wrapping.
        expected = (4.0 - 2 * np.pi)**2
        self.assertAlmostEqual(ekf.nees([0, 0, 0], np.eye(3), [0, 0, 4]), expected)
        x2, _ = ekf.predict(x_hat, P, [0.0, 1.0], 0.1)
        self.assertTrue(-np.pi <= x2[2] < np.pi)

    def test_repeated_updates_with_noise_dropout_and_outlier(self):
        # A small synthetic experiment checks many predictions/updates together.
        # Truth generates test measurements; it is never passed into the filter.
        # Noise is generated using the assumed Q/R, so this does not calibrate them.
        # A fixed seed makes the random sequence repeatable for debugging.
        rng = np.random.default_rng(7)
        truth = np.zeros(3)
        estimate = np.array([0.08, -0.05, 0.02])
        P = np.diag([0.2**2, 0.2**2, np.radians(5)**2])
        marker = np.array([4.0, 2.0])
        u = np.array([0.15, 0.08])
        for tick in range(500):
            truth = model.f(truth, u, 0.02) + rng.multivariate_normal(np.zeros(3), ekf.Q)
            truth[2] = ekf.wrap_angle(truth[2])
            estimate, P = ekf.predict(estimate, P, u, 0.02)
            if tick % 5 == 0 and not 100 <= tick < 200:
                z = ekf.h(truth, marker) + rng.multivariate_normal(np.zeros(2), ekf.R)
                if tick == 300:
                    z[0] += 20.0
                estimate, P = ekf.update(estimate, P, z, marker)
            self.assertTrue(np.all(np.isfinite(estimate)))
            np.testing.assert_allclose(P, P.T, atol=1e-12)
            self.assertTrue(np.all(np.linalg.eigvalsh(P) > 0))
        self.assertLess(np.linalg.norm(estimate[:2] - truth[:2]), 0.15)
        self.assertGreater(ekf.updates_accepted, 0)
        self.assertGreaterEqual(ekf.updates_rejected, 1)


if __name__ == "__main__":
    unittest.main()
