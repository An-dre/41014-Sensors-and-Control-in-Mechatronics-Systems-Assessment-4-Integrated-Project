# Checks for Part E: the integrated loop cannot see the simulator's true pose.
# From the repository root:
#   python -m unittest part_e_integration.test_pipeline -v

import ast
import inspect
import sys
import unittest
from pathlib import Path

import numpy as np

if __package__ in (None, ""):  # run as a file, not with -m: let Python find the part_* packages
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from part_b_sensing import sensing
from part_c_estimation import ekf
from part_e_integration import navigation, pipeline, sensor_adapter

BANNED = ("truth", "qpos", "sensordata", "collided", "set_pose", "Simulator", "simulator")


def names_in(source):
    tree = ast.parse(source)
    found = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    return found | {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}


class TestNoTruthInTheLoop(unittest.TestCase):
    def test_step_takes_only_sensor_readings_and_time(self):
        params = list(inspect.signature(pipeline.Pipeline.step).parameters)
        self.assertEqual(params, ["self", "angles", "t", "raw_scan", "image"])

    def test_loop_code_never_names_the_simulator(self):
        # The code that runs inside the loop: the Pipeline class, the planner, the sensor
        # adapter, the EKF's predict and update, and the Part B functions the adapter calls.
        # (ekf.nees and ekf.estimator_metrics do take the true pose: they score runs afterwards.)
        parts = [inspect.getsource(pipeline.Pipeline), inspect.getsource(navigation),
                 inspect.getsource(sensor_adapter)]
        parts += [inspect.getsource(getattr(ekf, name)) for name in ("predict", "update", "h", "jacobian_H")]
        parts += [inspect.getsource(getattr(sensing, name)) for name in
                  ("noisy_encoders", "odometry", "noisy_lidar", "detect_marker", "cam_to_base", "range_bearing")]
        for source in parts:
            found = names_in(source)
            for banned in BANNED:
                self.assertNotIn(banned, found)

    def test_hiding_the_truth_changes_no_command(self):
        # Run the same 6 s of the west trial twice, the second time with the simulator's
        # truth() and collided() returning NaN / False. The wheel commands must be identical.
        try:
            import mujoco  # noqa: F401
        except ImportError:
            self.skipTest("MuJoCo isn't installed")
        from part_e_integration import sim
        options = pipeline.default_options(duration=6.0, placement_noise=True)
        simulator = sim.Simulator()
        try:
            honest, _ = pipeline.run_trial(options, 7, simulator)
            simulator.truth = lambda: np.full(3, np.nan)
            simulator.collided = lambda: False
            blind, _ = pipeline.run_trial(options, 7, simulator)
        finally:
            simulator.close()
        self.assertTrue(np.all(np.isnan(blind["true"])))
        np.testing.assert_array_equal(honest["wheels"], blind["wheels"])
        np.testing.assert_array_equal(honest["estimate"], blind["estimate"])


if __name__ == "__main__":
    unittest.main()
