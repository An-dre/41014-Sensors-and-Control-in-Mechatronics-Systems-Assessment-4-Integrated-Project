"""Part A model check: does the unicycle model predict how the MuJoCo robot really moves?

For three manoeuvres the robot is driven with fixed wheel speeds. The model integrates the
wheel encoder rates (wheels_to_body, then f) every 2 ms physics step, and the result is
compared with the simulator's true pose at the end (evaluation only). Each manoeuvre is run
twice: with the robot exactly as the xml loads, and with the floor-contact correction the
pipeline uses (sim.load()).

    python -m part_a_modelling.validatemodel

Writes results_part_a/validation.md.
"""

import sys
from pathlib import Path

import mujoco
import numpy as np

if __package__ in (None, ""):  # run as a file, not with -m: let Python find the part_* packages
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from part_a_modelling import model as tb3
from part_e_integration import sim

RESULTS = Path(__file__).resolve().parent / "results_part_a"

# name, start pose (x, y, heading) in free space, wheel speeds (left, right) in rad/s, seconds
MANOEUVRES = [
    ("straight", (-2.0, 1.5, 0.0), (4.0, 4.0), 7.5),
    ("spin on the spot", (-2.0, 1.5, 0.0), (-3.0, 3.0), 9.0),
    ("arc (left 2, right 4 rad/s)", (-1.5, 1.2, 0.0), (2.0, 4.0), 10.0),
]


def yaw(data):
    w, qx, qy, qz = data.qpos[3:7]
    return np.arctan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def drive(mj, data, start, wheels, seconds):
    sim.set_pose(mj, data, *start)
    dt = mj.opt.timestep
    encoder = [mj.sensor(name).adr[0] for name in ("enc_left", "enc_right")]
    previous = data.sensordata[encoder].copy()
    pose_model = np.array(start, dtype=float)
    data.ctrl[:] = wheels
    for _ in range(round(seconds / dt)):
        mujoco.mj_step(mj, data)
        now = data.sensordata[encoder].copy()
        w_left, w_right = (now - previous) / dt
        previous = now
        v, omega = tb3.wheels_to_body(w_left, w_right)
        pose_model = tb3.f(pose_model, np.array([v, omega]), dt)
    pose_true = np.array([data.qpos[0], data.qpos[1], yaw(data)])   # evaluation only
    return pose_model, pose_true


def main():
    as_xml = mujoco.MjModel.from_xml_path(str(sim.SCENE))
    corrected, _ = sim.load()
    rows = []
    for label, mj in (("as the xml loads", as_xml), ("with the contact correction", corrected)):
        data = mujoco.MjData(mj)
        for name, start, wheels, seconds in MANOEUVRES:
            start = np.array(start)
            pose_model, pose_true = drive(mj, data, start, wheels, seconds)
            # what the model says happened, against what really happened
            model_dist = np.hypot(*(pose_model[:2] - start[:2]))
            true_dist = np.hypot(*(pose_true[:2] - start[:2]))
            model_turn = pose_model[2] - start[2]
            true_turn = np.unwrap([model_turn, pose_true[2] - start[2]])[1]
            position_error = np.hypot(*(pose_model[:2] - pose_true[:2]))
            heading_error = np.degrees(model_turn - true_turn)
            rows.append((label, name, seconds, model_dist, true_dist, np.degrees(model_turn),
                         np.degrees(true_turn), position_error, heading_error))
            print(f"{label:28s} {name:28s} model {model_dist:5.3f} m {np.degrees(model_turn):7.1f} deg | "
                  f"true {true_dist:5.3f} m {np.degrees(true_turn):7.1f} deg | "
                  f"error {100 * position_error:5.1f} cm {heading_error:+6.1f} deg")

    RESULTS.mkdir(exist_ok=True)
    lines = ["# Part A model check", "",
             "Made by `python -m part_a_modelling.validatemodel`. The model integrates the encoder rates every "
             "2 ms physics step; the true pose is read from the simulator only to score it.", "",
             "| Robot model | Manoeuvre | Time (s) | Model: distance, turn | True: distance, turn "
             "| Position error | Heading error |",
             "|---|---|---|---|---|---|---|"]
    for label, name, seconds, md, td, mt, tt, pe, he in rows:
        lines.append(f"| {label} | {name} | {seconds:g} | {md:.3f} m, {mt:.1f}° | {td:.3f} m, {tt:.1f}° "
                     f"| {100 * pe:.1f} cm | {he:+.1f}° |")
    (RESULTS / "validation.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("saved", (RESULTS / "validation.md").relative_to(Path(__file__).resolve().parents[1]).as_posix())


if __name__ == "__main__":
    main()
