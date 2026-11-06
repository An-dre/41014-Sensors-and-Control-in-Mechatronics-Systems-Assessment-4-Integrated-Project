"""Part C: collect rendered-camera trials and compare EKF noise settings on identical sensor logs.

From the repository root:
    python -m part_c_estimation.evaluate_estimator --collect                  tuning set (9 trials)
    python -m part_c_estimation.evaluate_estimator --collect --seed 2000 --output part_c_estimation/results_part_c/validation
    python -m part_c_estimation.evaluate_estimator                            replay existing logs only
    python -m part_c_estimation.evaluate_estimator --logs part_e_integration/results_robustness/logs --output part_c_estimation/results_part_c/robustness_replay

Collection runs the full pipeline (main.py's code) and saves each trial's odometry and
camera measurements. Every variant below is then replayed on exactly those inputs. The last
command replays Part E's robustness study the same way.
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

if __package__ in (None, ""):  # run as a file, not with -m: let Python find the part_* packages
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from part_b_sensing import sensing
from part_c_estimation import ekf
from part_e_integration import pipeline

HERE = Path(__file__).resolve().parent
SCENARIOS = {
    "west": dict(start="west", wheel_gain=(1., 1.), camera_dropout=(-1., -1.)),
    "north": dict(start="north", wheel_gain=(1., 1.), camera_dropout=(-1., -1.)),
    "south_disturbed": dict(start="south", wheel_gain=(.9, 1.), camera_dropout=(25., 30.)),
}
# variant: (Q scale, R scale, use Part B's measured R instead of the design R). The scales apply to
# Part C's design Q and to the R chosen, so every variant is the same absolute setting whatever the
# trials were driven with. odometry_only ignores the camera.
VARIANTS = {"design": (1., 1., False), "Q_x0.1": (.1, 1., False), "Q_x10": (10., 1., False),
            "R_x0.1": (1., .1, False), "R_x10": (1., 10., False), "R_measured": (1., 1., True),
            "Q_x0.1_R_measured": (.1, 1., True), "odometry_only": (1., 1., False)}
MEASURED_R = sensing.R_CALIBRATED_FILE   # Part B's measured R, for the calibrated camera the pipeline uses
DRIVEN_WITH = {"design": "Part C's design Q and R", "R_measured": "Part C's design Q with Part B's measured R"}
BAD_READING = 0.03   # m: a camera range more than this far from the true range counts as a bad reading


def variant_noise(variant):
    q_scale, r_scale, measured = VARIANTS[variant]
    return ekf.noise_matrices(q_scale, r_scale, MEASURED_R if measured else None)


def camera_errors(log):
    """Evaluation only: which steps had a camera reading, and how far off its range really was."""
    z = log["camera_measurement"]
    have = np.all(np.isfinite(z), axis=1)
    error = np.full(len(z), np.nan)
    error[have] = [z[k, 0] - ekf.h(log["true"][k], log["marker"])[0] for k in np.flatnonzero(have)]
    return have, error


def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def collect(args):
    # Importing MuJoCo is only needed for collection, not offline replay.
    from part_e_integration import sim
    rows = []
    for scenario_index, (scenario, settings) in enumerate(SCENARIOS.items()):
        simulator = sim.Simulator(settings["wheel_gain"])
        try:
            for trial in range(args.trials):
                seed = args.seed + scenario_index * 100 + trial
                options = pipeline.default_options(**settings, placement_noise=True,
                                                   calibration=args.calibration,
                                                   measurement_noise=args.measurement_noise,
                                                   q_scale=args.q_scale, r_scale=args.r_scale)
                result, summary = pipeline.run_trial(options, seed, simulator)
                name = f"{scenario}_seed{seed}"
                np.savez_compressed(args.output / "logs" / f"{name}.npz", **result)
                rows.append(dict(scenario=scenario, **summary))
                print(f"{name}: success={summary['success']}, "
                      f"EKF={100 * summary['ekf_position_rmse']:.2f} cm, "
                      f"odometry={100 * summary['odometry_position_rmse']:.2f} cm", flush=True)
        finally:
            simulator.close()
    write_csv(args.output / "closed_loop.csv", rows)


def replay(log, Q, R, camera=True):
    """Re-estimate from recorded measurements with process noise Q and camera noise R.
    Truth is used only for scoring."""
    x, P = log["initial_pose"].copy(), log["initial_P"].copy()
    estimates, covariances, scores, accepted = [], [], [], []
    ekf.reset_update_counts()
    for u, z in zip(log["odometry_input"], log["camera_measurement"]):
        if np.all(np.isfinite(u)):
            x, P = ekf.predict(x, P, u, pipeline.DT, Q)
        before, nis = ekf.updates_accepted, np.nan
        if camera and np.all(np.isfinite(z)):
            x, P = ekf.update(x, P, z, log["marker"], R)
            nis = ekf.last_nis
        estimates.append(x.copy())
        covariances.append(P.copy())
        scores.append(nis)
        accepted.append(ekf.updates_accepted > before)
    result = dict(estimate=np.asarray(estimates), P=np.asarray(covariances),
                  nis=np.asarray(scores), accepted=np.asarray(accepted))
    metrics = ekf.estimator_metrics(result["estimate"], result["P"], log["true"], result["nis"])
    metrics.update(camera_accepted=ekf.updates_accepted, camera_rejected=ekf.updates_rejected)
    return result, metrics


def group_table(rows):
    # one row per scenario (the log name without its seed), mean position RMSE and NEES per variant
    groups = list(dict.fromkeys(row["trial"].rsplit("_seed", 1)[0] for row in rows))
    lines = ["| Scenario | Trials | " + " | ".join(VARIANTS) + " |", "|---|---|" + "---|" * len(VARIANTS)]
    for group in groups:
        cells = []
        for variant in VARIANTS:
            picked = [row for row in rows if row["variant"] == variant and row["trial"].rsplit("_seed", 1)[0] == group]
            cells.append(f"{100 * np.mean([row['ekf_position_rmse'] for row in picked]):.2f} "
                         f"({np.mean([row['mean_nees'] for row in picked]):.2f})")
        trials = len({row["trial"] for row in rows if row["trial"].rsplit("_seed", 1)[0] == group})
        lines.append(f"| {group} | {trials} | " + " | ".join(cells) + " |")
    return lines


def analyse(output, logs=None):
    paths = sorted((logs or output / "logs").glob("*.npz"))
    if not paths:
        raise ValueError("No sensor logs found. Run with --collect first.")
    noise = {variant: variant_noise(variant) for variant in VARIANTS}
    measured_R = noise["R_measured"][1]
    rows, worst, driven, off_axis = [], (-1., None), set(), []
    for path in paths:
        with np.load(path, allow_pickle=False) as saved:
            log = dict(saved)
        # Replaying with the trial's own Q and R must reproduce the live filter exactly,
        # or the comparison is unfair.
        live, live_metrics = replay(log, log["Q"], log["R"])
        np.testing.assert_allclose(live["estimate"], log["estimate"], atol=1e-10, rtol=0)
        np.testing.assert_allclose(live["P"], log["P"], atol=1e-10, rtol=0)
        np.testing.assert_array_equal(live["accepted"], log["fix"])
        if live_metrics["ekf_position_rmse"] > worst[0]:
            worst = (live_metrics["ekf_position_rmse"], {**log, **live})
        driven.add(next((DRIVEN_WITH.get(variant, variant) for variant, (Q, R) in noise.items()
                         if variant != "odometry_only" and np.allclose(Q, log["Q"], rtol=1e-9, atol=0)
                         and np.allclose(R, log["R"], rtol=1e-9, atol=0)), "other settings"))
        have, error = camera_errors(log)
        bad = have & (np.abs(np.nan_to_num(error)) > BAD_READING)
        off_axis += list(np.degrees(np.abs(log["camera_measurement"][bad, 1])))
        for variant, (Q, R) in noise.items():
            result, metrics = replay(log, Q, R, camera=variant != "odometry_only")
            rejected = have & ~result["accepted"] if variant != "odometry_only" else np.zeros_like(have)
            metrics.update(readings=int(have.sum()), bad_readings=int(bad.sum()),
                           bad_rejected=int((rejected & bad).sum()), good_rejected=int((rejected & ~bad).sum()))
            rows.append(dict(trial=path.stem, variant=variant, **metrics))
        print(f"Replayed {path.stem}", flush=True)
    write_csv(output / "replay_trials.csv", rows)
    summaries = []
    for variant in VARIANTS:
        group = [row for row in rows if row["variant"] == variant]
        summary = dict(variant=variant, trials=len(group))
        for key in group[0]:
            if key not in ("trial", "variant"):
                values = [row[key] for row in group if row[key] is not None]
                summary[key] = float(np.mean(values)) if values else None
        summary["position_rmse_sd"] = float(np.std([row["ekf_position_rmse"] for row in group]))
        summaries.append(summary)
    write_csv(output / "tuning_summary.csv", summaries)
    pipeline.plot_trial(worst[1], output / "estimator_diagnostics.png")
    lines = ["# Part C experiment results", "",
             f"Replay of {len(paths)} rendered-camera trajectories; each trial has equal weight. "
             f"The trials were driven live with {' and '.join(sorted(driven))}.",
             "Every variant replays the same recorded inputs with a fixed setting, so these are estimator "
             "comparisons, not new closed-loop trials. design = Part C's design Q and R; the next four scale one "
             f"of them; R_measured is Part B's measured R ({MEASURED_R.name}), used alone and with Q ×0.1: σ range "
             f"{np.sqrt(measured_R[0, 0]) * 1000:.1f} mm, σ bearing {np.degrees(np.sqrt(measured_R[1, 1])):.3f}°.", "",
             "| Variant | Mean position RMSE (cm) | Heading RMSE (deg) | Mean NEES | x/y/heading coverage (%) "
             "| Mean pre-gate NIS | Rejects/trial |",
             "|---|---|---|---|---|---|---|"]
    for row in summaries:
        coverage = "/".join(f"{100 * row[key]:.1f}" for key in ("coverage_x", "coverage_y", "coverage_heading"))
        nis = "n/a" if row["mean_nis_all"] is None else f"{row['mean_nis_all']:.3f}"
        lines.append(f"| {row['variant']} | {100 * row['ekf_position_rmse']:.2f} | {row['heading_rmse_deg']:.2f} | "
                     f"{row['mean_nees']:.3f} | {coverage} | {nis} | {row['camera_rejected']:.2f} |")
    lines += ["", "Coverage is the fraction inside ±2 standard deviations. Ideal mean NEES is 3 and NIS is 2.",
              "NIS includes rejected measurements. Time samples are correlated; these are descriptive consistency checks.",
              "", "Per scenario: mean position RMSE in cm, with mean NEES in brackets.", ""]
    lines += group_table(rows)
    total = lambda variant, key: sum(row[key] for row in rows if row["variant"] == variant)
    lines += ["", f"What the innovation gate (NIS > {ekf.GATE_THRESHOLD}) did with the camera readings. A reading is bad when "
              f"its range is more than {100 * BAD_READING:.0f} cm from the truth (scored afterwards, never used live)."
              + (f" The bad readings were seen {min(off_axis):.0f}–{max(off_axis):.0f}° off the camera axis." if off_axis else ""),
              "", "| Variant | Readings | Bad readings | Bad ones rejected | Good ones rejected |", "|---|---|---|---|---|"]
    lines += [f"| {variant} | {total(variant, 'readings')} | {total(variant, 'bad_readings')} | "
              f"{total(variant, 'bad_rejected')} | {total(variant, 'good_rejected')} |"
              for variant in VARIANTS if variant != "odometry_only"]
    lines += ["", "![Worst trial as driven: path and states with uncertainty](estimator_diagnostics.png)", ""]
    (output / "results.md").write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--collect", action="store_true", help="run new closed-loop trials before replaying")
    parser.add_argument("--trials", type=int, default=3, help="trials per scenario")
    parser.add_argument("--seed", type=int, default=1000, help="first seed (scenarios add 0, 100, 200)")
    parser.add_argument("--output", type=Path, default=HERE / "results_part_c", help="folder for logs and results")
    parser.add_argument("--logs", type=Path,
                        help="replay the sensor logs in this folder instead of OUTPUT/logs, "
                             "e.g. part_e_integration/results_robustness/logs")
    parser.add_argument("--calibration", type=Path, default=sensing.CALIBRATION_FILE,
                        help="camera intrinsics used while collecting; default: Part B's calibration")
    parser.add_argument("--measurement-noise", type=Path, help="R used live while collecting; default: design R")
    parser.add_argument("--q-scale", type=float, default=1.)
    parser.add_argument("--r-scale", type=float, default=1.)
    args = parser.parse_args()
    if args.trials < 1:
        parser.error("Use at least one trial per scenario.")
    if args.logs and args.collect:
        parser.error("--logs replays existing logs; it can't be combined with --collect.")
    try:
        ekf.noise_matrices(args.q_scale, args.r_scale, args.measurement_noise)
        (args.output if args.logs else args.output / "logs").mkdir(parents=True, exist_ok=True)
        if args.collect:
            if any((args.output / "logs").glob("*.npz")):
                parser.error("Choose a fresh --output directory for collection.")
            collect(args)
        analyse(args.output, args.logs)
    except (ValueError, OSError, KeyError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
