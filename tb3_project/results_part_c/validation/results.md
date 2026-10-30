# Part C experiment results

Replay of 9 rendered-camera trajectories; each trial has equal weight.
All variants use identical inputs. These are estimator comparisons, not new closed-loop trials.

| Variant | Mean position RMSE (cm) | Heading RMSE (deg) | Mean NEES | x/y/heading coverage (%) | Mean pre-gate NIS | Rejects/trial |
|---|---|---|---|---|---|---|
| baseline | 3.81 | 1.14 | 2.397 | 89.1/99.6/99.8 | 0.207 | 2.22 |
| Q_x0.1 | 3.73 | 1.07 | 6.498 | 75.6/97.4/98.2 | 0.258 | 2.22 |
| Q_x10 | 3.89 | 1.20 | 0.731 | 99.5/100.0/100.0 | 0.163 | 1.44 |
| R_x0.1 | 3.87 | 1.17 | 6.650 | 85.0/99.6/99.8 | 1.602 | 5.44 |
| R_x10 | 3.76 | 1.14 | 1.011 | 100.0/99.6/99.8 | 0.028 | 0.00 |
| odometry_only | 9.55 | 1.65 | 0.840 | 100.0/99.6/99.8 | n/a | 0.00 |

Coverage is the fraction inside +/-2 standard deviations. Ideal mean NEES is 3 and NIS is 2.
NIS includes rejected measurements. Time samples are correlated; these are descriptive consistency checks.

![Worst baseline trial: path and states with uncertainty](estimator_diagnostics.png)
