### Design comparison (kinematic, 10 trial(s) per route)

| variant | docked ok | rms e_y true (cm) | max e_y true (cm) | rms e_y seen (cm) | max e_y seen (cm) | rms feedback w (rad/s) | wheel limit hit (% of steps) |
|---|---|---|---|---|---|---|---|
| final: scheduled LQR + feedforward | 30/30 | 3.05 | 17.03 | 0.78 | 14.67 | 0.031 | 0.3 |
| one fixed gain (straight line, 0.15 m/s) | 30/30 | 3.06 | 17.03 | 0.79 | 14.68 | 0.037 | 0.3 |
| no turn-rate feedforward | 30/30 | 3.18 | 17.02 | 1.09 | 14.68 | 0.086 | 0.4 |
