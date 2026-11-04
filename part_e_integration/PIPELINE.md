# Integrated TurtleBot pipeline

The new entry point is `main.py`. It uses the actual MuJoCo
camera and LiDAR, Dinel's sensing functions, Gabriel's model, Abdul's EKF and
Andre's LQR. It navigates to a known docking approach point using measured
LiDAR obstacles, then makes a straight visual docking approach.

This completes the runnable connection between the components. It does not
complete Part B's calibration experiments. Part C now includes comparative
simulation noise tuning and fresh-seed validation in
[PART_C_ESTIMATION.md](../part_c_estimation/PART_C_ESTIMATION.md); sensor-measured tuning
remains dependent on Part B's handoff.

## Install and run

Tested on Windows with Python 3.10.11. From the repository root:

```powershell
python -m venv .venv
.venv/Scripts/python -m pip install -r requirements.txt
.venv/Scripts/python main.py
.venv/Scripts/python main.py --viewer
```

The first command without `--viewer` runs as fast as possible and saves plots.
The viewer runs near real time. A trial ends when docking finishes, the tracker
times out, or the duration limit is reached. Success additionally requires the
true final pose to be within 3 cm along, 2 cm sideways and 3 degrees, with no
recorded obstacle contact. True pose is used only after computing commands.

The pinned requirements use `opencv-contrib-python`, which supplies ArUco.
Install one OpenCV distribution in the fresh environment.

Other examples:

```powershell
# Repeat with random placement error (2 cm and 1.5 degrees standard deviation).
python main.py --placement-noise --trials 3 --seed 30 --output part_e_integration/results_pipeline/validation/west

# Start from the north of the room.
python main.py --start north --placement-noise --trials 3 --seed 10 --output part_e_integration/results_pipeline/validation/north

# Weak left motor and camera loss from 25 to 30 seconds.
python main.py --start south --placement-noise --trials 3 --seed 20 --wheel-gain 0.9 1 --camera-dropout 25 30 --output part_e_integration/results_pipeline/validation/south_disturbed

# Use Dinel's calibration when ready (NPZ keys: K and dist).
python main.py --calibration path/to/calib.npz

# Load the separate, measured range/bearing covariance from Part B.
python main.py --calibration path/to/calib.npz --measurement-noise path/to/camera_noise.npz

# Compare estimator noise settings on identical recorded sensor inputs.
python part_c_estimation/evaluate_estimator.py --collect

# Existing model, EKF and controller tests.
python -B -m unittest discover -s part_c_estimation -p "test_*.py" -v
python -B -m unittest discover -s part_d_control -p "test_*.py" -v
```

Use a different `--output` directory for each set of conditions. Running with
the same start and seed in the same directory replaces that trial's artifacts;
`summary.csv` describes the trials from the latest invocation in that directory.

## Follow one control step

```text
Wheel angles -> Part B noisy_encoders + odometry -> Part C EKF predict
Camera image -> Part B detect_marker + frame conversion -> EKF update
LiDAR ranges -> Part B noisy_lidar -> obstacle map -> A* waypoints
Estimated pose + next waypoint -> Part D LQR -> Part A wheel conversion -> MuJoCo

True pose -> evaluation, logs and plots only
```

The main loop runs every 0.02 seconds. Encoders, prediction and control run at
50 Hz. Camera and LiDAR run at 10 Hz. During camera loss, the EKF continues
predicting from encoders and carries its uncertainty forward.

The state is `[world x, world y, heading]`, in metres and radians. Odometry gives
`[forward speed, yaw rate]`, in m/s and rad/s. The camera supplies planar range
in metres and bearing in radians, both relative to the robot base. Wheel commands
are left/right angular velocities in rad/s. The LQR receives the EKF estimate.

## Walkthrough of the changes

### 1. `part_e_integration/sim.py`: simulator access

This supplies the missing `sim` module expected by Part B. It loads the current
scene, reads sensors by name, renders the `rgb` camera and advances the physics
ten times per control step. The trial setup function sets the starting pose;
it never supplies a measured state to the filter.

Andre's existing floor-contact correction lives here so both runners use the
same loaded robot. It gives the casters contact priority and lifts the chassis
collision mesh clear of the floor. The source XML files remain unchanged.
`truth()` and `collided()` are reserved for evaluating trials.

The camera position, LiDAR position and known marker position are read from
the model definition. Reading fixed mounting dimensions and a known landmark
is different from reading the robot's live world pose.

### 2. `part_e_integration/sensor_adapter.py`: Part B connection

Both existing `sensing.py` files are unchanged. The adapter calls Part B's
encoder, odometry, LiDAR, marker detection and frame conversion functions.

Two explicit scene settings bridge the different assumptions in those files:

* The current marker plate is 0.20 m, but its black square is 0.16 m because of
  the image's white border. PnP translation is scaled by the ratio of the actual
  black-square size to Part B's configured size.
* Part B's camera translation is adjusted to the camera mount in the current XML.
  The camera axis rotation still comes from Part B's `cam_to_base()`.

This avoids modifying or monkey-patching Dinel's constants or algorithms.
Non-finite camera outputs are discarded before the EKF. No alternate detector
is substituted if Part B rejects a frame.

An optional calibration NPZ supplies `K` and `dist`. Without it, the runner
prints that it is using nominal pinhole intrinsics and records `calibrated=false`.
Applying a calibration file does not change the EKF's measurement covariance.
Use `--measurement-noise` to load a separate NPZ containing a positive definite
2 by 2 `R` in range-metres/bearing-radians order. `--q-scale` and `--r-scale`
change one trial's matrices without modifying the legacy Part D settings.

### 3. `part_e_integration/navigation.py`: LiDAR navigation

The map is a 0.10 m grid in the known 6 by 6 m room. Internal obstacles come
from LiDAR endpoints transformed using the estimated pose and LiDAR mounting
offset. Obstacle locations are not read from the simulator's world geometry.
No-return beams are excluded using their original mask.

Observed endpoints receive a 0.40 m clearance margin. A* finds a path to the
pre-dock point; line-of-sight simplification reduces the grid path to waypoints.
The LQR tracks one straight leg at a time, stopping and aligning between legs.
If a new scan blocks the remaining leg, the robot replans from its estimate.

A 0.15 m start allowance lets the robot leave an inflated margin around its
current centre. This prevents noise or placement error from trapping a safe
robot in its own clearance margin. It only relaxes the start of a leg; it does
not erase mapped obstacles or relax the destination.

At the approach point, the controller follows the existing 0.90 m final
straight into the dock. This simple method prioritises readability over speed;
it does not reproduce the smooth full-route trajectories in Part D's old trials.

### 4. `main.py`: one loop and evaluation

`Pipeline.step()` accepts wheel angles, time, a scan and an image. It has no
simulator argument and cannot read ground truth. `run_trial()` handles setup,
calls that step, then separately logs true pose and contacts for evaluation.

Every trial writes:

* `<start>_seed<N>.npz`: timestamps, true/estimated/odometry poses, **full 3 by 3
  covariance**, wheel commands, camera update flags, control references,
  navigation modes, the obstacle map and NEES.
  Replay inputs include odometry, camera measurements, pre-gate NIS,
  initial conditions and Q/R.
* `<start>_seed<N>.json`: trial conditions, success, collision flag, docking
  errors, estimator and cross-track RMSE, wheel-command RMS, camera rejection
  counts and mean NEES.
* `<start>_seed<N>.png`: path/map and all three estimated states with uncertainty
  alongside truth.
* `summary.csv`: one row per trial in that invocation.

RMSE distances are metres; heading errors are degrees where the name says
`deg`. NEES uses the full covariance, including cross terms. A mean near three
is only a rough diagnostic here, not proof of filter consistency: samples
within a trajectory are correlated and the sensor noise is not yet calibrated.

### 5. Small changes to existing control code

`controller.py` now calls Part A's `linear_error_model()` and `body_to_wheels()`
instead of keeping duplicate formulas. Gains and control weights are unchanged.
The tracker now reports `timeout` when settling takes too long; only sustained
in-tolerance settling reports `docked`.

`run_tracking.py` keeps its existing interface and experiments. Its contact
correction delegates to the shared simulator helper, and it stops a trial on
the new timeout status. Its old results are separate from the new pipeline's
results and must not be described as trials of this new navigation method.

### 6. Dependencies and repository housekeeping

`requirements.txt` records the tested dependency versions. `.gitignore` excludes
Python caches, the local virtual environment and generated pipeline results.
Regenerate or deliberately include selected result artifacts for the project
page; ignored local plots are not automatically published to GitHub.

The README links this guide and now describes integration and calibration
status accurately.

## Validation of this implementation

The **12 controller tests pass**, covering control stability, docking,
wheel limits, route clearance and ground-truth isolation.
The pipeline is also checked by running a
complete simulation and inspecting its saved success, collision and error metrics.

The nine final trials below use rendered camera images and the unchanged Part B
functions. All include random starting placement error. Their exact commands
and seeds appear in the run examples above.

| Conditions | Seeds | Successes | Recorded collisions | Mean EKF position RMSE | Mean odometry position RMSE | Mean true cross-track RMSE |
|---|---|---|---|---|---|---|
| West start | 30–32 | 3/3 | 0 | 2.66 cm | 5.24 cm | 1.69 cm |
| North start | 10–12 | 3/3 | 0 | 4.15 cm | 9.95 cm | 4.04 cm |
| South, left motor at 90%, camera unavailable at 25–30 s | 20–22 | 3/3 | 0 | 4.13 cm | 13.00 cm | 4.21 cm |

Across these trials, the largest absolute final errors were **0.40 cm along,
0.38 cm sideways and 1.39 degrees**. These are nine checks of selected conditions,
not a claim of a general success rate. The default nominal west-start run also
docked in the integration test.

During development, two varied trials initially stopped when an inflated
obstacle margin covered the current cell. The start allowance described above
fixed that planner deadlock; the final trials include those same seeds.

Detailed local outputs are in `part_e_integration/results_pipeline/validation/`.
Each group's `summary.csv`, individual JSON files and plots can be inspected
alongside the full NPZ logs. These files are generated and ignored by Git.

## Dinel's remaining work and current limits

* Part B still owns calibration images/board setup, measured sensor-noise results,
  and any changes to its detector or sensing algorithms. The current scene has
  no calibration board; adding `sim.py` alone does not complete calibration.
* The EKF's Q and R now have comparative simulation tuning evidence, but are not
  measured sensor calibration. See the Part C report for the consistency limits
  and the calibration and measured-noise inputs still needed from Part B.
* Unknown space is treated as traversable. The map accumulates obstacle hits and
  does not clear old obstacles, so large pose drift or moving obstacles can make
  it conservative. If no path exists, it holds still and eventually reaches
  the trial duration limit. This is not a general SLAM system.
* LiDAR replanning applies to navigation up to the pre-dock point. The final
  straight assumes a clear docking station; it does not navigate around a new
  obstruction placed in that final corridor.
* The project page, group number, contribution statements, AI declaration and
  five-minute demonstration still need the team's final submission work.
  This integration was written with OpenAI Codex assistance for code, tests and
  documentation; include that purpose in the team's AI declaration.
