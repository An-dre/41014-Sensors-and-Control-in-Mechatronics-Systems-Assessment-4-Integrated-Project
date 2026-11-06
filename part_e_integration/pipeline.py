"""Run the integrated pipeline: Part B sensing -> Part C EKF -> LiDAR A* planner -> Part D LQR.

From the repository root:
    python main.py                       one docking run from the west start
    python main.py --viewer              the same run in the MuJoCo viewer
    python main.py --start south --placement-noise --trials 3 --seed 20 --wheel-gain 0.9 1 --camera-dropout 25 30

By default the camera uses Part B's calibrated intrinsics and the EKF uses Part C's design
noise Q and R, the setting Part C's study chose; --measurement-noise loads Part B's measured R
instead. Results (NPZ log, JSON summary, PNG plot per trial) go to
part_e_integration/results_pipeline/ unless --output says otherwise.
"""

import argparse
import csv
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

if __package__ in (None, ""):  # run as a file, not with -m: let Python find the part_* packages
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from part_a_modelling import model
from part_b_sensing import sensing
from part_c_estimation import ekf
from part_d_control import controller
from part_e_integration import sim
from part_e_integration.navigation import Navigator
from part_e_integration.sensor_adapter import Sensors

DT = 0.02
SENSOR_EVERY = 5  # camera and LiDAR at 10 Hz; odometry, EKF and control at 50 Hz
START_SD = np.array([0.02, 0.02, np.radians(1.5)])
STARTS = {
    "west": np.array([-2.3, 0.0, 0.0]),
    "north": np.array([-2.2, 2.0, -np.pi / 2]),
    "south": np.array([-1.0, -2.2, 0.0]),
}
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "results_pipeline"


def default_options(**changes):
    """Every trial setting with its default value, for scripts that call run_trial()."""
    options = dict(start="west", duration=150.0, placement_noise=False, placement_scale=1.0,
                   wheel_gain=(1.0, 1.0), camera_dropout=(-1.0, -1.0),
                   calibration=sensing.CALIBRATION_FILE, measurement_noise=None,
                   q_scale=1.0, r_scale=1.0, lidar_noise_scale=1.0, encoder_noise_scale=1.0)
    unknown = set(changes) - set(options)
    if unknown:
        raise TypeError(f"Unknown trial options: {sorted(unknown)}")
    options.update(changes)
    return SimpleNamespace(**options)


class Pipeline:
    """One control step. Inputs are sensors and time, never simulator truth."""

    def __init__(self, start, sensors, marker, normal, lidar_position, lidar_angles,
                 process_covariance=None, measurement_covariance=None):
        self.sensors = sensors
        self.marker = np.asarray(marker)
        self.estimate = np.asarray(start, dtype=float).copy()
        self.P = np.diag(START_SD**2)
        self.Q = ekf.Q if process_covariance is None else process_covariance
        self.R = ekf.R if measurement_covariance is None else measurement_covariance
        self.odometry_pose = self.estimate.copy()
        self.previous_encoders = None
        self.navigator = Navigator(marker, normal, lidar_position, lidar_angles)
        self.last_u = np.full(2, np.nan)
        self.last_z = np.full(2, np.nan)
        self.nis = np.nan
        ekf.reset_update_counts()

    def step(self, angles, t, raw_scan=None, image=None):
        self.nis = np.nan
        self.last_u = np.full(2, np.nan)
        self.last_z = np.full(2, np.nan)
        encoders = self.sensors.encoders(angles)
        if self.previous_encoders is not None:
            u = self.sensors.odometry(encoders, self.previous_encoders, DT)
            self.last_u = u.copy()
            self.estimate, self.P = ekf.predict(self.estimate, self.P, u, DT, self.Q)
            self.odometry_pose = model.f(self.odometry_pose, u, DT)
            self.odometry_pose[2] = ekf.wrap_angle(self.odometry_pose[2])
        self.previous_encoders = encoders

        accepted = False
        if image is not None:
            measurement = self.sensors.marker(image)
            if measurement is not None:
                self.last_z = measurement.copy()
                before = ekf.updates_accepted
                self.estimate, self.P = ekf.update(self.estimate, self.P, measurement, self.marker, self.R)
                accepted = ekf.updates_accepted > before
                self.nis = ekf.last_nis

        scan = self.sensors.lidar(raw_scan) if raw_scan is not None else None
        v, w = self.navigator.step(self.estimate, t, scan, raw_scan)
        left, right, _ = controller.wheel_speeds(v, w)
        return left, right, accepted


def run_trial(args, seed, simulator, viewer=None):
    # Part B uses NumPy's global random generator. Seed it once per trial so its
    # noise functions stay unchanged and repeated runs can be reproduced.
    np.random.seed(seed)
    setup_rng = np.random.default_rng(seed)
    start = STARTS[args.start]
    # The robot is put down off its intended pose; the EKF is only told the intended pose.
    placement = setup_rng.normal(0, START_SD * args.placement_scale) if args.placement_noise else np.zeros(3)
    simulator.reset(start + placement)
    sensors = Sensors(simulator.camera_position, simulator.marker_size, args.calibration,
                      lidar_noise_scale=args.lidar_noise_scale,
                      encoder_noise_scale=args.encoder_noise_scale)
    Q, R = ekf.noise_matrices(args.q_scale, args.r_scale, args.measurement_noise)
    pipeline = Pipeline(start, sensors, simulator.marker, simulator.marker_normal,
                        simulator.lidar_position, simulator.lidar_angles, Q, R)
    log = {key: [] for key in ("time", "true", "estimate", "odometry", "P", "wheels", "fix", "mode", "reference",
                              "tracking", "odometry_input", "camera_measurement", "nis")}
    collided = False
    for step in range(round(args.duration / DT)):
        t = step * DT
        raw_scan = image = None
        if step % SENSOR_EVERY == 0:
            raw_scan = simulator.lidar()
            if not args.camera_dropout[0] <= t < args.camera_dropout[1]:
                image = simulator.image()
        left, right, fix = pipeline.step(simulator.wheel_angles(), t, raw_scan, image)

        tracker = pipeline.navigator.tracker
        reference = pipeline.estimate.copy()
        if tracker is not None:
            elapsed = 0.0 if tracker.t0 is None else t - tracker.t0
            reference = tracker.ref.sample(elapsed)[0]

        # Evaluation is deliberately outside Pipeline.step(). Nothing below is
        # fed back into the EKF, planner or controller.
        values = (t, simulator.truth(), pipeline.estimate.copy(), pipeline.odometry_pose.copy(),
                  pipeline.P.copy(), [left, right], fix, pipeline.navigator.mode,
                  reference, tracker is not None and tracker.mode == "track",
                  pipeline.last_u.copy(), pipeline.last_z.copy(), pipeline.nis)
        for key, value in zip(log, values):
            log[key].append(value)
        collided |= simulator.collided()
        if pipeline.navigator.mode in ("docked", "timeout"):
            break
        simulator.drive(left, right)
        if viewer is not None:
            if not viewer.is_running():
                break
            viewer.sync()
            time.sleep(DT)

    simulator.data.ctrl[:] = 0
    result = {key: np.asarray(values) for key, values in log.items()}
    result.update(initial_pose=start.copy(), initial_P=np.diag(START_SD**2),
                  marker=simulator.marker.copy(), Q=Q, R=R)
    result["map_blocked"] = pipeline.navigator.map.blocked.copy()
    result["map_coordinates"] = pipeline.navigator.map.coordinates.copy()
    result["dock_pose"] = np.r_[pipeline.navigator.dock,
                                np.arctan2(-simulator.marker_normal[1], -simulator.marker_normal[0])]
    summary = evaluate(result, collided)
    summary.update(seed=seed, start=args.start, replans=pipeline.navigator.replans,
                   camera_accepted=ekf.updates_accepted, camera_rejected=ekf.updates_rejected,
                   camera_invalid=sensors.invalid_camera, calibrated=sensors.calibrated,
                   wheel_gain=list(args.wheel_gain), camera_dropout=list(args.camera_dropout),
                   placement_noise=args.placement_noise, placement_scale=args.placement_scale,
                   lidar_noise_scale=args.lidar_noise_scale, encoder_noise_scale=args.encoder_noise_scale,
                   sensor_rejects=dict(sensing.REJECTS), q_scale=args.q_scale, r_scale=args.r_scale,
                   Q=Q.tolist(), R=R.tolist(),
                   measurement_noise_source=_relative(args.measurement_noise) or "Part C design R",
                   calibration_source=_relative(args.calibration) or "nominal pinhole K")
    return result, summary


def _relative(path):
    """A path as it would be typed from the repository root, for the JSON summaries."""
    if path is None:
        return None
    path = Path(path).resolve()
    root = Path(__file__).resolve().parents[1]
    return path.relative_to(root).as_posix() if path.is_relative_to(root) else str(path)


def evaluate(result, collided):
    odo_error = result["odometry"] - result["true"]
    dock_error = controller.tracking_error(result["true"][-1], result["dock_pose"])
    nees = np.array([ekf.nees(x, P, truth) for x, P, truth in
                     zip(result["estimate"], result["P"], result["true"])])
    result["nees"] = nees
    final_mode = str(result["mode"][-1])
    docked = final_mode == "docked"
    status = final_mode if final_mode in ("docked", "timeout") else "duration_limit"
    tracking_errors = np.array([controller.tracking_error(truth, reference) for truth, reference
                               in zip(result["true"], result["reference"])])
    cross_track = tracking_errors[result["tracking"], 1]
    success = (docked and not collided and abs(dock_error[0]) < 0.03
               and abs(dock_error[1]) < 0.02 and abs(dock_error[2]) < np.radians(3))
    summary = dict(success=bool(success), status=status,
                   collided=bool(collided), time=float(result["time"][-1]),
                   odometry_position_rmse=float(np.sqrt(np.mean(np.sum(odo_error[:, :2]**2, axis=1)))),
                   cross_track_rmse=float(np.sqrt(np.mean(cross_track**2))) if len(cross_track) else None,
                   wheel_command_rms=float(np.sqrt(np.mean(result["wheels"]**2))),
                   dock_along=float(dock_error[0]),
                   dock_side=float(dock_error[1]), dock_heading_deg=float(np.degrees(dock_error[2])))
    summary.update(ekf.estimator_metrics(result["estimate"], result["P"], result["true"],
                                        result["nis"]))
    return summary


def plot_trial(result, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    ax = axes[0, 0]
    coordinates = result["map_coordinates"]
    ax.imshow(result["map_blocked"].T, origin="lower", extent=[coordinates[0] - 0.05, coordinates[-1] + 0.05] * 2,
              cmap="Greys", alpha=0.3)
    for key, label in (("true", "True path (evaluation)"), ("estimate", "EKF"), ("odometry", "Odometry")):
        ax.plot(*result[key][:, :2].T, label=label)
    ax.plot(*result["dock_pose"][:2], "kx", label="Dock target")
    ax.set(title="LiDAR map and navigation", xlabel="World x (m)", ylabel="World y (m)", aspect="equal")
    ax.legend(fontsize=8)
    for index, ax in enumerate((axes[0, 1], axes[1, 0], axes[1, 1])):
        estimate = result["estimate"][:, index]
        truth = result["true"][:, index]
        sd = np.sqrt(result["P"][:, index, index])
        if index == 2:
            # Put truth on the nearest branch to the estimate before plotting.
            estimate = np.unwrap(estimate)
            truth = estimate + ekf.wrap_angle(truth - estimate)
            estimate, truth, sd = np.degrees(estimate), np.degrees(truth), np.degrees(sd)
        t = result["time"]
        ax.plot(t, truth, label="True (evaluation)")
        ax.plot(t, estimate, label="EKF estimate")
        ax.fill_between(t, estimate - 2 * sd, estimate + 2 * sd, alpha=0.25, label="EKF ±2 standard deviations")
        ax.set(xlabel="Time (s)", ylabel=("x (m)", "y (m)", "Heading (degrees)")[index])
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--start", choices=STARTS, default="west", help="starting corner of the room")
    parser.add_argument("--trials", type=int, default=1, help="number of trials, seeds SEED, SEED+1, ...")
    parser.add_argument("--seed", type=int, default=0, help="seed of the first trial")
    parser.add_argument("--duration", type=float, default=150, help="time limit per trial (s)")
    parser.add_argument("--placement-noise", action="store_true",
                        help="put the robot down 2 cm / 1.5 degrees (1 sd) off its start without telling the EKF")
    parser.add_argument("--placement-scale", type=float, default=1.0,
                        help="multiply the placement error's standard deviation (robustness tests)")
    parser.add_argument("--wheel-gain", type=float, nargs=2, default=(1.0, 1.0), metavar=("LEFT", "RIGHT"),
                        help="motor strength, e.g. 0.9 1 for a left motor at 90 percent")
    parser.add_argument("--camera-dropout", type=float, nargs=2, default=(-1.0, -1.0), metavar=("START", "END"),
                        help="hide every camera image between these times (s)")
    parser.add_argument("--calibration", type=Path, default=sensing.CALIBRATION_FILE,
                        help="camera intrinsics .npz (K, dist); default: Part B's calibration")
    parser.add_argument("--nominal-camera", action="store_true",
                        help="use the ideal pinhole K from the field of view instead of the calibration")
    parser.add_argument("--measurement-noise", type=Path,
                        help="camera noise R for the EKF (.npz, 2x2, metres and radians), e.g. Part B's "
                             "part_b_sensing/results_part_b/camera_R_calibrated.npz; default: Part C's design R")
    parser.add_argument("--q-scale", type=float, default=1.0, help="multiplier of the EKF process covariance")
    parser.add_argument("--r-scale", type=float, default=1.0, help="multiplier of the EKF camera covariance")
    parser.add_argument("--lidar-noise-scale", type=float, default=1.0, help="multiply Part B's LiDAR noise")
    parser.add_argument("--encoder-noise-scale", type=float, default=1.0, help="multiply Part B's encoder noise")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="folder for the results")
    parser.add_argument("--viewer", action="store_true", help="watch in the MuJoCo viewer (near real time)")
    args = parser.parse_args()
    if args.trials < 1 or args.duration < DT or not all(0 < gain <= 1 for gain in args.wheel_gain):
        parser.error("Use at least one trial, a positive duration and wheel gains in (0, 1].")
    if args.camera_dropout[1] < args.camera_dropout[0]:
        parser.error("Camera dropout END must be at or after START.")
    if min(args.placement_scale, args.lidar_noise_scale, args.encoder_noise_scale) < 0:
        parser.error("Noise scales cannot be negative.")
    if args.nominal_camera:
        args.calibration = None
    elif not Path(args.calibration).is_file():
        parser.error(f"No calibration file at {args.calibration}. "
                     "Run: python -m part_b_sensing.sensing calib, or use --nominal-camera.")
    try:
        ekf.noise_matrices(args.q_scale, args.r_scale, args.measurement_noise)
    except (ValueError, OSError, KeyError) as exc:
        parser.error(str(exc))
    args.output.mkdir(parents=True, exist_ok=True)
    print(f"Camera: {_relative(args.calibration) or 'nominal pinhole K'}. "
          f"EKF camera noise: {_relative(args.measurement_noise) or 'Part C design R'}.", flush=True)
    simulator = sim.Simulator(args.wheel_gain)
    summaries = []
    try:
        for trial in range(args.trials):
            seed = args.seed + trial
            if args.viewer:
                import mujoco.viewer
                with mujoco.viewer.launch_passive(simulator.model, simulator.data) as viewer:
                    result, summary = run_trial(args, seed, simulator, viewer)
            else:
                result, summary = run_trial(args, seed, simulator)
            stem = f"{args.start}_seed{seed}"
            np.savez_compressed(args.output / f"{stem}.npz", **result)
            (args.output / f"{stem}.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
            plot_trial(result, args.output / f"{stem}.png")
            summaries.append(summary)
            print(f"{stem}: {summary['status']}, success={summary['success']}, "
                  f"EKF RMSE={100 * summary['ekf_position_rmse']:.1f} cm, "
                  f"camera fixes={summary['camera_accepted']}, replans={summary['replans']}", flush=True)
    finally:
        simulator.close()
    with (args.output / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    print(f"Successes: {sum(row['success'] for row in summaries)}/{len(summaries)}. Results: {args.output}")


if __name__ == "__main__":
    main()
