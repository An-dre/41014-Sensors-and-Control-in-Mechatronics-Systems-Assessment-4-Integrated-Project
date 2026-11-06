"""Part E robustness study: how far each disturbance can go before the pipeline stops docking.

Every condition changes one thing from the nominal trial (west start, 2 cm / 1.5 degree
placement error) and runs the full pipeline with the same three seeds, so each level is
compared with the same starts. The nominal trial is also run from the north and south starts.
Noise levels multiply Part B's models: LiDAR sigma 7.5 mm / 2.5 % of range, encoder sigma
one tick, placement error 2 cm / 1.5 degrees (1 sd).

From the repository root:
    python -m part_e_integration.robustness --list                  the conditions
    python -m part_e_integration.robustness --condition motor_0.7   one condition
    python -m part_e_integration.robustness --all                   every condition (about 3.5 hours)
    python -m part_e_integration.robustness --report                tables and plots from saved results

The trials use the pipeline's defaults, including Part C's design Q and R. The same study with
Part B's measured R, for comparison:
    python -m part_e_integration.robustness --all --measurement-noise part_b_sensing/results_part_b/camera_R_calibrated.npz --output part_e_integration/results_robustness_measured_R

Results: part_e_integration/results_robustness/ (one CSV per condition, summary.md,
robustness.png, and a plot of every failed trial). Each trial's sensor log goes to its logs/
folder (not in Git), so Part C can replay the study with other noise settings.
"""

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np

if __package__ in (None, ""):  # run as a file, not with -m: let Python find the part_* packages
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from part_e_integration import pipeline

RESULTS = Path(__file__).resolve().parent / "results_robustness"
SEEDS = (500, 501, 502)
CAMERA_LOSS_START = 30.0   # s; from the west start the camera first sees the marker 33-37 s in

# name: (factor, level shown in the tables, trial options that differ from the nominal trial)
CONDITIONS = {
    "nominal_west": ("nominal", "west", dict(start="west")),
    "nominal_north": ("nominal", "north", dict(start="north")),
    "nominal_south": ("nominal", "south", dict(start="south")),
    "camera_loss_10s": ("camera lost from t = 30 s", "10 s", dict(camera_dropout=(CAMERA_LOSS_START, 40.0))),
    "camera_loss_20s": ("camera lost from t = 30 s", "20 s", dict(camera_dropout=(CAMERA_LOSS_START, 50.0))),
    "camera_loss_30s": ("camera lost from t = 30 s", "30 s", dict(camera_dropout=(CAMERA_LOSS_START, 60.0))),
    "camera_loss_40s": ("camera lost from t = 30 s", "40 s", dict(camera_dropout=(CAMERA_LOSS_START, 70.0))),
    "camera_loss_rest": ("camera lost from t = 30 s", "rest of run", dict(camera_dropout=(CAMERA_LOSS_START, 1e9))),
    "motor_0.8": ("left motor strength", "80 %", dict(wheel_gain=(0.8, 1.0))),
    "motor_0.7": ("left motor strength", "70 %", dict(wheel_gain=(0.7, 1.0))),
    "motor_0.6": ("left motor strength", "60 %", dict(wheel_gain=(0.6, 1.0))),
    "motor_0.5": ("left motor strength", "50 %", dict(wheel_gain=(0.5, 1.0))),
    "lidar_noise_x3": ("LiDAR noise", "×3", dict(lidar_noise_scale=3.0)),
    "lidar_noise_x10": ("LiDAR noise", "×10", dict(lidar_noise_scale=10.0)),
    "lidar_noise_x30": ("LiDAR noise", "×30", dict(lidar_noise_scale=30.0)),
    "placement_x3": ("placement error", "×3", dict(placement_scale=3.0)),
    "placement_x6": ("placement error", "×6", dict(placement_scale=6.0)),
    "encoder_noise_x10": ("encoder noise", "×10", dict(encoder_noise_scale=10.0)),
    "encoder_noise_x30": ("encoder noise", "×30", dict(encoder_noise_scale=30.0)),
}
FIELDS = ["condition", "factor", "level", "seed", "start", "success", "status", "collided", "time",
          "ekf_position_rmse", "odometry_position_rmse", "heading_rmse_deg", "mean_nees",
          "dock_along", "dock_side", "dock_heading_deg", "camera_accepted", "camera_rejected",
          "replans", "cross_track_rmse", "wheel_command_rms"]


def run_condition(name, output=RESULTS, measurement_noise=None):
    from part_e_integration import sim
    factor, level, changes = CONDITIONS[name]
    options = pipeline.default_options(placement_noise=True, measurement_noise=measurement_noise, **changes)
    seeds = [seed + {"west": 0, "north": 10, "south": 20}[options.start] for seed in SEEDS]
    simulator = sim.Simulator(options.wheel_gain)
    rows = []
    try:
        for seed in seeds:
            started = time.time()
            result, summary = pipeline.run_trial(options, seed, simulator)
            np.savez_compressed(output / "logs" / f"{name}_seed{seed}.npz", **result)
            row = dict(condition=name, factor=factor, level=level, **summary)
            rows.append({key: row[key] for key in FIELDS})
            if not summary["success"]:
                pipeline.plot_trial(result, output / f"failed_{name}_seed{seed}.png")
            print(f"{name} seed {seed}: {summary['status']}, success={summary['success']}, "
                  f"EKF RMSE {100 * summary['ekf_position_rmse']:.1f} cm, "
                  f"{time.time() - started:.0f} s", flush=True)
    finally:
        simulator.close()
    with (output / "conditions" / f"{name}.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    return rows


def read_rows(output=RESULTS):
    rows = []
    for name in CONDITIONS:
        path = output / "conditions" / f"{name}.csv"
        if path.is_file():
            with path.open(encoding="utf-8") as stream:
                rows += list(csv.DictReader(stream))
    for row in rows:
        row["success"] = row["success"] == "True"
        row["collided"] = row["collided"] == "True"
        for key in FIELDS[8:]:
            row[key] = float(row[key]) if row[key] not in ("", "None") else np.nan
    return rows


def failure(row):
    # why a trial missed: the dock tolerance it broke, or how far from the dock it stopped
    if row["collided"]:
        return "hit a box"
    if row["status"] != "docked":
        return f"stopped {np.hypot(row['dock_along'], row['dock_side']):.1f} m from the dock ({row['status'].replace('_', ' ')})"
    missed = [f"{100 * abs(row['dock_along']):.1f} cm along" if abs(row["dock_along"]) >= 0.03 else "",
              f"{100 * abs(row['dock_side']):.1f} cm sideways" if abs(row["dock_side"]) >= 0.02 else "",
              f"{abs(row['dock_heading_deg']):.1f}° off" if abs(row["dock_heading_deg"]) >= 3 else ""]
    return "docked " + ", ".join(m for m in missed if m)


def report(output=RESULTS, measurement_noise=None):
    rows = read_rows(output)
    if not rows:
        raise SystemExit("No results yet. Run --condition NAME or --all first.")
    noise = "Part B's measured R" if measurement_noise else "Part C's design Q and R"
    lines = ["# Part E robustness study", "",
             f"Made by `python -m part_e_integration.robustness`, with {noise} in the EKF. "
             "Full pipeline with the rendered camera; "
             f"seeds {', '.join(map(str, SEEDS))} (north +10, south +20) in every row, each with a random "
             "2 cm / 1.5° placement error the EKF is not told about. Success = docked within 3 cm along, "
             "2 cm sideways and 3° (true pose), with no collision.", "",
             "| Condition | Level | Docked OK | EKF position RMSE (cm) | Odometry RMSE (cm) | Mean NEES "
             "| Sideways at the dock (cm) | Camera fixes | Camera readings rejected | Time (s) | Failures |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for name, (factor, level, _) in CONDITIONS.items():
        group = [row for row in rows if row["condition"] == name]
        if not group:
            continue
        ok = sum(row["success"] for row in group)
        failures = "; ".join(f"seed {int(row['seed'])}: {failure(row)}" for row in group if not row["success"]) or "none"
        docked = [abs(row["dock_side"]) for row in group if row["status"] == "docked"]

        def mean(key, scale=1.0):
            return np.nanmean([row[key] for row in group]) * scale

        lines.append(f"| {factor} | {level} | {ok}/{len(group)} | {mean('ekf_position_rmse', 100):.2f} "
                     f"| {mean('odometry_position_rmse', 100):.2f} | {mean('mean_nees'):.2f} "
                     f"| {f'{100 * np.mean(docked):.2f}' if docked else '–'} "
                     f"| {mean('camera_accepted'):.0f} | {mean('camera_rejected'):.0f} | {mean('time'):.0f} | {failures} |")
    total = sum(row["success"] for row in rows)
    lines += ["", f"{total} of {len(rows)} trials docked within tolerance.", "",
              "![Robustness study](robustness.png)", ""]
    (output / "summary.md").write_text("\n".join(lines), encoding="utf-8")
    plot(rows, output)
    print("\n".join(lines))


def plot(rows, output=RESULTS):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import FixedLocator, NullLocator

    TICKS = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200]   # cm, on the log axis
    factors = [factor for factor in dict.fromkeys(f for f, _, _ in CONDITIONS.values()) if factor != "nominal"]
    nominal = [row for row in rows if row["condition"] == "nominal_west"]
    fig, axes = plt.subplots(2, len(factors), figsize=(3.1 * len(factors), 5.6), sharex="col", sharey="row",
                             gridspec_kw={"height_ratios": [1, 1.6]})
    for column, factor in enumerate(factors):
        names = [name for name, (f, _, _) in CONDITIONS.items() if f == factor]
        labels = ["nominal"] + [CONDITIONS[name][1].replace("rest of run", "rest") for name in names]
        groups = [nominal] + [[row for row in rows if row["condition"] == name] for name in names]
        x = np.arange(len(groups))
        top, bottom = axes[0, column], axes[1, column]
        for index, group in enumerate(groups):
            if not group:
                continue
            ok = sum(row["success"] for row in group)
            top.bar(index, ok / len(group), color="tab:green" if ok == len(group) else "tab:orange", width=0.6)
            top.text(index, ok / len(group) + 0.04, f"{ok}/{len(group)}", ha="center", fontsize=8)
            for row in group:
                value = 100 * row["ekf_position_rmse"]
                if row["success"]:
                    bottom.plot(index, value, "o", color="tab:green", markersize=5)
                else:
                    bottom.plot(index, value, "x", color="tab:red", markersize=7, markeredgewidth=2)
        top.set_title(factor, fontsize=9)
        top.set(ylim=(0, 1.2), yticks=[0, 0.5, 1])
        bottom.set(xticks=x, yscale="log")
        bottom.set_xticklabels(labels, fontsize=8)
        bottom.yaxis.set_major_locator(FixedLocator(TICKS))
        bottom.yaxis.set_minor_locator(NullLocator())
        bottom.yaxis.set_major_formatter(lambda value, _: f"{value:g}")
        bottom.grid(axis="y", alpha=0.3)
    values = [100 * row["ekf_position_rmse"] for row in rows]
    low = min(1.0, 0.8 * min(values))
    axes[1, 0].set_ylim(max([tick for tick in TICKS if tick <= low], default=low), max(20.0, 1.5 * max(values)))
    axes[0, 0].set_ylabel("fraction docked\nwithin tolerance")
    axes[1, 0].set_ylabel("EKF position RMSE (cm)\no docked, x failed")
    fig.tight_layout()
    fig.savefig(output / "robustness.png", dpi=130)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--condition", choices=CONDITIONS, help="run one condition")
    group.add_argument("--all", action="store_true", help="run every condition, then --report")
    group.add_argument("--report", action="store_true", help="tables and plots from saved results")
    group.add_argument("--list", action="store_true", help="list the conditions")
    parser.add_argument("--output", type=Path, default=RESULTS,
                        help="results folder (default: part_e_integration/results_robustness)")
    parser.add_argument("--measurement-noise", type=Path,
                        help="camera noise R for the EKF in every trial (.npz); default: Part C's design R")
    args = parser.parse_args()
    if args.measurement_noise and not args.measurement_noise.is_file():
        parser.error(f"--measurement-noise: no file at {args.measurement_noise}")
    if args.list:
        for name, (factor, level, changes) in CONDITIONS.items():
            print(f"{name:20s} {factor}: {level}  {changes}")
        return
    for folder in ("conditions", "logs"):
        (args.output / folder).mkdir(parents=True, exist_ok=True)
    if args.condition:
        run_condition(args.condition, args.output, args.measurement_noise)
    elif args.all:
        for name in CONDITIONS:
            run_condition(name, args.output, args.measurement_noise)
        report(args.output, args.measurement_noise)
    else:
        report(args.output, args.measurement_noise)


if __name__ == "__main__":
    main()
