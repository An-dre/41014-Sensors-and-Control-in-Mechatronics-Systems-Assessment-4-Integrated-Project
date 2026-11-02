"""Part C: collect real-camera trials and compare Q/R on identical sensor logs.

Run with --collect once; omit --collect to replay existing logs.
"""

import argparse
import csv
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import ekf
import run_pipeline as pipeline

HERE = Path(__file__).resolve().parent
SCENARIOS = {
    "west": dict(start="west", wheel_gain=(1., 1.), camera_dropout=(-1., -1.)),
    "north": dict(start="north", wheel_gain=(1., 1.), camera_dropout=(-1., -1.)),
    "south_disturbed": dict(start="south", wheel_gain=(.9, 1.), camera_dropout=(25., 30.)),
}
VARIANTS = {"baseline": (1., 1.), "Q_x0.1": (.1, 1.), "Q_x10": (10., 1.),
            "R_x0.1": (1., .1), "R_x10": (1., 10.), "odometry_only": (1., 1.)}


def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def collect(args):
    # Importing MuJoCo is only needed for collection, not offline replay.
    import sim
    rows = []
    for scenario_index, (scenario, settings) in enumerate(SCENARIOS.items()):
        simulator = sim.Simulator(settings["wheel_gain"])
        try:
            for trial in range(args.trials):
                seed = args.seed + scenario_index * 100 + trial
                options = SimpleNamespace(**settings, duration=150., placement_noise=True,
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


def replay(log, q_scale=1., r_scale=1., camera=True):
    """Re-estimate from recorded measurements; truth is used only for scoring."""
    x, P = log["initial_pose"].copy(), log["initial_P"].copy()
    Q, R = log["Q"] * q_scale, log["R"] * r_scale
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


def analyse(output):
    paths = sorted((output / "logs").glob("*.npz"))
    if not paths:
        raise ValueError("No sensor logs found. Run with --collect first.")
    rows, worst = [], (-1., None)
    for path in paths:
        with np.load(path, allow_pickle=False) as saved:
            log = dict(saved)
        for variant, (q, r) in VARIANTS.items():
            result, metrics = replay(log, q, r, camera=variant != "odometry_only")
            rows.append(dict(trial=path.stem, variant=variant, **metrics))
            if variant == "baseline":
                # Simplifying the implementation must preserve the live filter result.
                np.testing.assert_allclose(result["estimate"], log["estimate"], atol=1e-10, rtol=0)
                np.testing.assert_allclose(result["P"], log["P"], atol=1e-10, rtol=0)
                np.testing.assert_array_equal(result["accepted"], log["fix"])
                if metrics["ekf_position_rmse"] > worst[0]:
                    worst = (metrics["ekf_position_rmse"], {**log, **result})
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
             f"Replay of {len(paths)} rendered-camera trajectories; each trial has equal weight.",
             "All variants use identical inputs. These are estimator comparisons, not new closed-loop trials.", "",
             "| Variant | Mean position RMSE (cm) | Heading RMSE (deg) | Mean NEES | x/y/heading coverage (%) | Mean pre-gate NIS | Rejects/trial |",
             "|---|---|---|---|---|---|---|"]
    for row in summaries:
        coverage = "/".join(f"{100 * row[key]:.1f}" for key in ("coverage_x", "coverage_y", "coverage_heading"))
        nis = "n/a" if row["mean_nis_all"] is None else f"{row['mean_nis_all']:.3f}"
        lines.append(f"| {row['variant']} | {100 * row['ekf_position_rmse']:.2f} | {row['heading_rmse_deg']:.2f} | "
                     f"{row['mean_nees']:.3f} | {coverage} | {nis} | {row['camera_rejected']:.2f} |")
    lines += ["", "Coverage is the fraction inside +/-2 standard deviations. Ideal mean NEES is 3 and NIS is 2.",
              "NIS includes rejected measurements. Time samples are correlated; these are descriptive consistency checks.", "",
              "![Worst baseline trial: path and states with uncertainty](estimator_diagnostics.png)", ""]
    (output / "results.md").write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collect", action="store_true")
    parser.add_argument("--trials", type=int, default=3, help="Trials per scenario")
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--output", type=Path, default=HERE / "results_part_c")
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--measurement-noise", type=Path)
    parser.add_argument("--q-scale", type=float, default=1.)
    parser.add_argument("--r-scale", type=float, default=1.)
    args = parser.parse_args()
    if args.trials < 1:
        parser.error("Use at least one trial per scenario.")
    try:
        ekf.noise_matrices(args.q_scale, args.r_scale, args.measurement_noise)
        (args.output / "logs").mkdir(parents=True, exist_ok=True)
        if args.collect:
            if any((args.output / "logs").glob("*.npz")):
                parser.error("Choose a fresh --output directory for collection.")
            collect(args)
        analyse(args.output)
    except (ValueError, OSError, KeyError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
