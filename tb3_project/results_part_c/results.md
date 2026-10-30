# Part C experiment results

Replay of 9 rendered-camera trajectories; each trial has equal weight.
All variants use identical inputs. These are estimator comparisons, not new closed-loop trials.

| Variant | Mean position RMSE (cm) | Heading RMSE (deg) | Mean NEES | x/y/heading coverage (%) | Mean pre-gate NIS | Rejects/trial |
|---|---|---|---|---|---|---|
| baseline | 3.77 | 1.21 | 2.230 | 88.2/100.0/100.0 | 0.159 | 1.22 |
| Q_x0.1 | 3.66 | 1.13 | 6.321 | 72.6/100.0/100.0 | 0.197 | 1.22 |
| Q_x10 | 3.97 | 1.30 | 0.716 | 99.2/100.0/100.0 | 0.127 | 1.11 |
| R_x0.1 | 3.90 | 1.25 | 6.701 | 81.4/100.0/100.0 | 1.323 | 5.67 |
| R_x10 | 3.70 | 1.19 | 0.819 | 99.5/100.0/100.0 | 0.020 | 0.00 |
| odometry_only | 9.84 | 1.75 | 0.427 | 100.0/100.0/100.0 | n/a | 0.00 |

Coverage is the fraction inside +/-2 standard deviations. Ideal mean NEES is 3 and NIS is 2.
NIS includes rejected measurements. Time samples are correlated; these are descriptive consistency checks.

![Worst baseline trial: path and states with uncertainty](estimator_diagnostics.png)
