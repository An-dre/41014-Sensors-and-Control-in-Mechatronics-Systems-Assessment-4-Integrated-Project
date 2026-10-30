### Design comparison (mujoco_synthetic, 10 trial(s) per route)

| variant | docked ok | rms e_y true (cm) | max e_y true (cm) | rms e_y seen (cm) | max e_y seen (cm) | rms feedback w (rad/s) | wheel limit hit (% of steps) |
|---|---|---|---|---|---|---|---|
| final: scheduled LQR + feedforward | 30/30 | 2.92 | 18.42 | 0.77 | 14.23 | 0.031 | 0.3 |
| one fixed gain (straight line, 0.15 m/s) | 30/30 | 2.93 | 18.42 | 0.78 | 14.24 | 0.037 | 0.3 |
| no turn-rate feedforward | 30/30 | 3.02 | 18.78 | 1.09 | 13.99 | 0.085 | 0.4 |
