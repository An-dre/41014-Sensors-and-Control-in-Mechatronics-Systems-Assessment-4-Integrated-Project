# Part C: Estimation (Abdul)

The estimator fuses wheel-encoder odometry with camera range and bearing to the known docking marker. The LQR and the LiDAR planner receive its estimated pose; the simulator's true pose is used only in evaluation. This is Track A's required EKF, with its design, a tuning study that chose its noise settings, and quantitative results on fresh seeds and under disturbances.

| file in `part_c_estimation/` | what it is |
|---|---|
| `ekf.py` | predict, update (with the NIS gate), noise matrices, and the NEES/coverage scoring used afterwards |
| `test_ekf.py` | 10 checks of the maths: Jacobians against finite differences, a scalar update, angle wrapping, gating, a 500-step synthetic run |
| `evaluate_estimator.py` | collects closed-loop trials and replays every noise setting on the same sensor logs |
| `results_part_c/` | the tuning study (`results.md`, CSVs, `estimator_diagnostics.png`); `validation/` and `validation_measured_R/` for the fresh seeds; `robustness_replay/` and `robustness_replay_measured_R/` for Part E's robustness logs |

## Run and reproduce

From the repository root, after installing `requirements.txt`:

```
python -m unittest part_c_estimation.test_ekf -v

# Tuning set: nine rendered-camera closed-loop trials driven with the design Q and R,
# then every setting replayed on their sensor logs (about 30 min).
python -m part_c_estimation.evaluate_estimator --collect

# Validation: nine fresh seeds driven with the chosen settings, then with Part B's measured R (about 30 min each).
python -m part_c_estimation.evaluate_estimator --collect --seed 2000 --output part_c_estimation/results_part_c/validation
python -m part_c_estimation.evaluate_estimator --collect --seed 2000 --output part_c_estimation/results_part_c/validation_measured_R --measurement-noise part_b_sensing/results_part_b/camera_R_calibrated.npz

# Every setting replayed on Part E's two robustness studies (after running them, see PART_E_INTEGRATION.md).
python -m part_c_estimation.evaluate_estimator --logs part_e_integration/results_robustness/logs --output part_c_estimation/results_part_c/robustness_replay
python -m part_c_estimation.evaluate_estimator --logs part_e_integration/results_robustness_measured_R/logs --output part_c_estimation/results_part_c/robustness_replay_measured_R

# Recompute the tables and figure from existing sensor logs (add --output for another set).
python -m part_c_estimation.evaluate_estimator

# Other noise settings in the live closed loop, not only in replay.
python main.py --q-scale 0.1 --output part_e_integration/results_pipeline/q01
python main.py --measurement-noise part_b_sensing/results_part_b/camera_R_calibrated.npz --output part_e_integration/results_pipeline/measured_R
```

Collection needs a fresh output folder so experiments are never mixed; omit `--collect` to replay an existing set. The compact CSVs, tables and figures are kept in `results_part_c/`. The raw sensor logs go to `logs/` subfolders, which Git ignores because the commands above regenerate them.

## State, prediction and camera update

The state is `[world x, world y, heading]` in metres and radians. The world frame is the fixed room frame of the known marker map; it is not reset at each start. The filter starts at the intended placement with standard deviations `[0.02 m, 0.02 m, 1.5°]`. In the trials with placement noise the actual robot is put down off that pose by the same amount, without the filter being told.

At 50 Hz, Part B differences the noisy wheel angles and calls Part A's wheel-to-body conversion to produce `[v, omega]`. The EKF uses Part A's nonlinear motion model and its state Jacobian F:

```text
x_pred = f(x, [v, omega], dt)
P_pred = F P F' + Q * dt / 0.02
```

The additive Q is a floor for residual motion-model uncertainty at 20 ms, treated as diffusion when dt changes. It covers effects the kinematic model leaves out; it is not a measurement of encoder noise. This approximation does not model correlated slip or the temporal correlation from differencing noisy encoders.

At up to 10 Hz, Part B detects the marker in the rendered image (calibrated K, sub-pixel corners), solves its pose and transforms it to the robot base. Only the resulting planar range and bearing enter the EKF:

```text
dx = marker_x - x; dy = marker_y - y
h(x) = [sqrt(dx*dx + dy*dy), atan2(dy, dx) - heading]
innovation = z - h(x)                         # wrap bearing to [-pi, pi)
S = H P H' + R
NIS = innovation' solve(S, innovation)
K = solve(S, H P)'
x_updated = x + K innovation                 # wrap heading
P_updated = (I-KH) P (I-KH)' + K R K'         # Joseph covariance update
```

H is the analytic derivative of h (Part A). Linear solves replace explicit matrix inverses. The Joseph form and symmetry clean-up protect the covariance from numerical loss of positivity.

Measurements with the wrong shape, non-finite values or non-positive range are rejected; bearing is undefined at the landmark itself. A measurement is also rejected when its NIS exceeds **9.21**, the 99% chi-square threshold for two degrees of freedom under the ideal Gaussian model. When no marker is detected, the filter keeps predicting from wheel odometry.

One function-based API (`predict`, `update`, `nees`) serves the pipeline and the replay. The pipeline passes its Q and R explicitly. Trials run one at a time and reset the shared update counters before each run.

## Noise settings and the tuning decision

At the reference 20 ms prediction interval:

| Matrix | Diagonal entries | Meaning |
|---|---|---|
| Q, design | `1e-6, 1e-6, 1.21846968e-5` | 1 mm in x and y and 0.2° of heading, standard deviation per 20 ms prediction |
| R, design | `9e-4, 3.04617420e-4` | 3 cm of range and 1° of bearing, standard deviation per camera reading |
| R, measured by Part B | `1.522e-4, 1.388e-7` | 12.3 mm of range and 0.021° of bearing (`part_b_sensing/results_part_b/camera_R_calibrated.npz`) |

The zero off-diagonals are a simplifying independence assumption. Q and the design R are **simulation design values**; the measured R is Part B's worst case over 1–2 m, with the bias folded in. **The pipeline uses the design Q and R**, the choice the study below made; `--measurement-noise part_b_sensing/results_part_b/camera_R_calibrated.npz` loads Part B's measured R instead.

### Tuning study on identical sensor logs

Nine rendered-camera closed-loop trials were driven with the design Q and R, each with a random 2 cm / 1.5° placement error: west (seeds 1000–1002), north (1100–1102), and south (1200–1202) with the left motor at 90 % and no camera from 25 to 30 s. All nine docked within tolerance, with no collisions. Their recorded odometry and camera readings were then replayed through each setting, so every setting sees exactly the same inputs. Replaying a trial with its own settings reproduces every saved live estimate, covariance and gate decision, which `evaluate_estimator.py` checks.

| Setting | Position RMSE (cm) | Heading RMSE (°) | Mean NEES | ±2σ coverage x / y / heading (%) | Bad readings rejected | Good readings rejected |
|---|---|---|---|---|---|---|
| **Design Q and R (used)** | **3.71** | **1.19** | **0.67** | **100 / 100 / 100** | **3 of 10** | **0** |
| Q ×0.1 | 3.64 | 1.14 | 1.76 | 95.8 / 100 / 100 | 3 of 10 | 0 |
| Q ×10 | 3.74 | 1.20 | 0.19 | 100 / 100 / 100 | 2 of 10 | 0 |
| R ×0.1 | 3.71 | 1.19 | 1.41 | 97.8 / 100 / 100 | 10 of 10 | 0 |
| R ×10 | 3.71 | 1.19 | 0.42 | 100 / 100 / 100 | 0 of 10 | 0 |
| Design Q, measured R | 3.71 | 1.19 | 1.45 | 98.5 / 100 / 100 | 10 of 10 | 0 |
| Q ×0.1, measured R | 3.62 | 1.13 | 3.63 | 85.5 / 100 / 100 | 10 of 10 | 0 |
| Odometry only | 9.80 | 1.74 | 0.42 | 100 / 100 / 100 | – | – |

A bad reading is a camera range more than 3 cm from the truth, scored afterwards. Of the 2438 readings, 10 were bad, all 5–11 cm short and seen 10–23° off the camera axis; Part B's noise experiment only measured head-on views. Full tables, per-scenario results and CSVs: [results_part_c/results.md](results_part_c/results.md).

**Accuracy hardly depends on Q and R.** Every setting is within 0.1 cm of 3.7 cm, against 9.8 cm for odometry alone. The error is decided before the camera sees the marker: the first fix comes 17–32 s into each run, after up to 14 cm of drift from the unknown placement error, and 2 s later the error is down to 1–4 cm.

**On these nominal runs, smaller Q and R describe the filter better.**

* With the design values the mean NEES is 0.67 against an ideal of 3, and every state stays inside ±2σ the whole time: the filter is conservative, its real error smaller than it believes. Part of that is Q. 0.2° of heading per 20 ms adds up to 7.7° of heading uncertainty after 30 s on odometry alone, far more than the real drift. Q ×0.1 brings NEES to 1.76 and x coverage to 95.8 %, close to the 95 % that ±2σ should give.
* The design R (3 cm, 1°) is far bigger than the camera's real error, so its gate only catches the three worst bad readings. The other seven, 5–9 cm short, get through with little weight. Part B's measured R makes the gate tight enough to reject all ten while still accepting all 2428 good readings, and brings NEES to 1.45. Combined with Q ×0.1 it overshoots into overconfidence: NEES 3.63, with only 85.5 % of the x error inside ±2σ.

**Under disturbances, the tight settings lock the camera out.** Part E also ran its whole robustness study with the measured R in the loop (`part_e_integration/results_robustness_measured_R/`), and replaying those 57 runs through every setting shows the cost. In one run with the left motor at 80 % (seed 500), the very first reading the camera gave was a bad one: 10 cm short, seen 25° off the axis. The estimate was then 4 cm out and still uncertain, so the gate let the reading in, and with an R that small the update dragged the estimate to 9 cm out and collapsed its covariance around the wrong place. Every good reading for the next 29 s disagreed by about 10 cm and was rejected, 278 of them, until Q had grown the uncertainty enough to let one through. A 70 % motor run locked out for 8 s the same way. The design R took in the same first reading with less weight, accepted the good readings that followed, and was back within 3 cm two seconds later.

| Setting, replayed on the 57 robustness runs | Position RMSE (cm) | Mean NEES | Bad readings rejected | Good readings rejected |
|---|---|---|---|---|
| **Design Q and R (used)** | **3.52** | **0.84** | **52 of 103** | **0** |
| Design Q, measured R (driven live) | 3.64 | 1.71 | 96 of 103 | 374 |
| Q ×0.1 | 3.53 | 2.47 | 51 of 103 | 311 |
| Q ×0.1, measured R | 3.54 | 4.74 | 100 of 103 | 725 |

Q ×0.1 locks out the same way after a large placement error: in one run with six times the usual error it rejected 311 good readings between 29 and 66 s. Full tables: [robustness_replay_measured_R/results.md](results_part_c/robustness_replay_measured_R/results.md).

**Decision: the design Q and R.** Accuracy is the same for every setting, in every set. The smaller settings describe the nominal runs better and keep out more bad readings, but when a bad reading slips in first they can shut out good ones for half a minute. The design Q and R never rejected a good reading in any of the tuning, validation or robustness runs, so the pipeline keeps them and accepts a conservative NEES as the price. On the real robot, Part B's measured R would be the starting point, with a guard against lock-out: for example, re-opening the gate when several readings in a row agree with each other but not with the estimate.

## Independent validation and robustness

### Fresh seeds, driven both ways

The same three scenarios with fresh seeds (west 2000–2002, north 2100–2102, disturbed south 2200–2202) were driven in closed loop twice: once with the chosen settings and once with Part B's measured R. Each pair starts from the same placement error and sees the same sensor noise, so the two runs differ only in R.

| | **Design Q and R (used)** | Design Q, measured R |
|---|---|---|
| Docked within tolerance, no collisions | **9/9** | 9/9 |
| EKF position RMSE (cm) | **3.60** | 3.64 |
| Odometry-only RMSE (cm) | 9.53 | 9.54 |
| Mean NEES | **0.94** | 1.74 |
| ±2σ coverage x / y / heading (%) | 100 / 99.6 / 99.8 | 97.8 / 99.6 / 99.8 |
| Bad camera readings rejected by the live gate | **10 of 21** | 32 of 34 |
| Good readings rejected | 0 | 0 |
| Worst final error: along / sideways / heading | 0.42 cm / 0.24 cm / 0.98° | 0.41 cm / 0.24 cm / 0.95° |

With the chosen settings the EKF's position error is 62 % smaller than odometry alone overall, and smaller in all nine runs: west 2.67 cm against 6.48 cm, north 4.66 cm against 13.30 cm, and 3.47 cm against 8.80 cm in the south runs with the weak motor and the camera gap, where the filter predicted through the five seconds without a camera. Driven live, the measured R gave the same accuracy and docking, a NEES closer to its ideal and fewer bad readings let through; neither lost a good reading on these runs. The two runs saw different numbers of bad readings because their paths differ slightly. Counting the tuning set, all 27 closed-loop trials in this part docked within tolerance. Full tables: [validation/results.md](results_part_c/validation/results.md) and [validation_measured_R/results.md](results_part_c/validation_measured_R/results.md).

### Replaying Part E's final robustness study

Part E's robustness study with the chosen settings (57 runs, `part_e_integration/results_robustness/`) gives a second set of disturbed logs, and replaying them shows the same picture. The design Q and R rejected no good reading and 55 of the 99 bad ones. Part B's measured R rejected 648 good readings, Q ×0.1 rejected 284, and the two together 659, again in lock-outs after a bad reading got in. Across the tuning, validation and both robustness sets, the design settings never rejected a good camera reading. Full tables: [robustness_replay/results.md](results_part_c/robustness_replay/results.md).

![Worst tuning trial: estimated states and uncertainty alongside the truth](results_part_c/estimator_diagnostics.png)

The worst tuning trial (north, seed 1101) drives for about 30 s before the camera sees the marker. In that time the estimate drifts 14 cm while its ±2σ band widens to cover it. The first fix, at 29.6 s, pulls the estimate back in one step and the band collapses. From there the camera keeps the error under a few centimetres to the dock.

## Evaluation, observability and limits

The full 3 × 3 covariance is saved, including cross terms. NEES is `error' solve(P, error)` with the heading error wrapped. Per-state coverage is the fraction of samples inside the plotted ±2 standard deviations. NIS is logged before gating, so rejected readings stay visible. Means weight each trajectory equally. Consecutive time samples are correlated, so these are descriptive checks, not independent-sample confidence intervals or a proof of consistency.

One point landmark giving only range and bearing has a rotational ambiguity: rotating the whole trajectory around the landmark, and the heading with it, leaves the measurements unchanged (Part A's observability rank is 2). Odometry and a known placement prior let this local filter work, but they do not make every global pose direction observable. A second known landmark, or the marker's orientation (full pose), would remove it. During marker loss there is no absolute correction and the uncertainty grows. Large initial pose errors, strong slip, biased measurements and long dropouts can take the filter outside its local operating range or its innovation gate; Part E's robustness study measures where. A bad reading accepted while the filter is uncertain can also make the gate reject good ones afterwards, as with the measured R above. The design R makes that much less likely, but nothing in the filter detects it.

Ground truth is never a live input to `predict`, `update`, `Pipeline.step`, the planner or the controller. Evaluation reads the truth after the commands have been computed, and the replay reads it only in its final scoring function. `part_e_integration/test_pipeline.py` checks that hiding the truth changes no command.

## Generative AI

OpenAI Codex assisted with the Part C implementation, tests, evaluation tooling, running the experiments and documentation. The final re-run of the tuning and validation sets, the measured-R comparison, the camera-gate and lock-out analysis behind the final noise choice, and the robustness replay were done with Claude (Anthropic). The team's full declaration is in the repository README.
