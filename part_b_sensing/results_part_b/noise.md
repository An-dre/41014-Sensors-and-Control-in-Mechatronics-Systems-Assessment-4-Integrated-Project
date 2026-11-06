# B7 measured camera noise

Made by `python -m part_b_sensing.sensing noise`. 150 photos per distance, head-on, with pixel noise σ = 2 grey levels and pose jitter σ = 2 mm, 0.2°. Error = measured − true.

## Ideal K (B3)

| Distance | Seen | Range bias | Range σ | Bearing bias | Bearing σ |
|---|---|---|---|---|---|
| 1.0 m | 150/150 | -0.1 mm | 0.2 mm | 0.068° | 0.013° |
| 1.5 m | 150/150 | 4.3 mm | 2.9 mm | 0.066° | 0.006° |
| 2.0 m | 150/150 | 9.9 mm | 6.7 mm | 0.066° | 0.003° |

Saved `camera_R_nominal.npz`: R = diag(1.437e-04 m², 1.469e-06 rad²), that is σ range 12.0 mm and σ bearing 0.069° (worst distance, bias folded in).

## Calibrated K (B4)

| Distance | Seen | Range bias | Range σ | Bearing bias | Bearing σ |
|---|---|---|---|---|---|
| 1.0 m | 150/150 | 0.1 mm | 0.2 mm | 0.016° | 0.014° |
| 1.5 m | 150/150 | 4.6 mm | 2.9 mm | 0.015° | 0.006° |
| 2.0 m | 150/150 | 10.4 mm | 6.7 mm | 0.015° | 0.003° |

Saved `camera_R_calibrated.npz`: R = diag(1.522e-04 m², 1.388e-07 rad²), that is σ range 12.3 mm and σ bearing 0.021° (worst distance, bias folded in).
