Part B: B7 Experiment

### Purpose

The EKF uses R to work out how much weight to give to each camera measurement. If R is too small it will be chasing the noise; if R is too large it will effectively be ignoring the good measurements. B7 measures R rather than having to work out what it should be.

### Method

The robot was parked facing the marker at 1.0, 1.5 and 2.0m with 150 photos taken at each distance. Each of those photos was then processed via detect_marker(), cam_to_base() and range_bearing(). As we were the ones that placed the robot the true range and bearing are known:

$$e = z_{meas} - z_{true}, \qquad \text{bias} = \bar{e}, \qquad \sigma = \mathrm{std}(e)$$

The simulator is perfectly repeatable so we've added pixel noise (σ = 2 grey levels) and pose jitter (σ = 2mm, 0.2°). Those assumptions fix the value of σ but not the trends that we show below. The experiment was run with the nominal K (B3) and the calibrated K (B4).

This is the measure_noise() function in sensing.py.

### Results

Nominal K (B3):

| Distance | Seen | Range bias | Range σ | Bearing bias | Bearing σ |
|---|---|---|---|---|---|
| 1.0 m | 150/150 | 3.9 mm | 4.2 mm | 0.063° | 0.026° |
| 1.5 m | 140/150 | 10.3 mm | 13.0 mm | 0.069° | 0.040° |
| 2.0 m | 150/150 | 33.3 mm | 18.5 mm | 0.065° | 0.033° |

Calibrated K (B4):

| Distance | Seen | Range bias | Range σ | Bearing bias | Bearing σ |
|---|---|---|---|---|---|
| 1.0 m | 150/150 | 4.1 mm | 4.1 mm | 0.015° | 0.027° |
| 1.5 m | 150/150 | 9.1 mm | 13.9 mm | 0.017° | 0.040° |
| 2.0 m | 150/150 | 33.8 mm | 18.5 mm | 0.014° | 0.033° |

Results vary by a few mm between machines because of rendering differences; the trends are unchanged.

The 10 missing frames at 1.5 m returned nan poses, which led to the "not finite" check in detect_marker().

### Discussion

**Range noise grows with d².** Distance comes from how big the marker looks, d = f s / w, so

$$\sigma_d = \frac{\sigma_w}{f\,s}\,d^2$$

The data fits σ_d ≈ 0.0045 d² (4.5, 10.1, 18.0 mm predicted), which means the corners only wobble by about 0.3 px. The detector is precise; the geometry makes range worse with distance.

**Bearing is vastly more accurate than range.** At 2m the bearing sigma (0.033°) is about 1mm sideways, compared to 18.5mm in range. The camera is excellent at giving you the direction and very poor at giving you the distance.

**Range is biased long** (about 4, 10, 34 mm), and calibration did not change it, so K is not the cause. The likely cause is the corners of a small marker being detected slightly inside its true edges, which we did not test directly.

**Calibration removed the bearing bias.** A half-pixel error at the image centre gives 0.5/415.69 rad = 0.069°, which is the nominal bias. With the calibrated K it's down to 0.015° (about 75% less). σ is the same in both tables as calibration removes the systematic error (it doesn't remove the random noise).

### R for the EKF

ekf.py takes a fixed R so we went with the worst case (2m) and treated the bias as extra variance as the EKF doesn't get rid of it:

$$R = \mathrm{diag}\left(b_\rho^2 + \sigma_\rho^2,\; b_\beta^2 + \sigma_\beta^2\right)$$

| File | Use with | σ range | σ bearing |
|---|---|---|---|
| camera_R_nominal.npz | default K | 38.1 mm | 0.080° |
| camera_R_calibrated.npz | --calibration calib.npz | 38.5 mm | 0.044° |

Each R only goes with the K it was measured with.

A worst-case R is safe but cautious: at 1 m the real range error is about 6 mm, but R uses 38.5 mm. So the EKF under-trusts the camera when it is closest to the dock.

### Limitations

- Only head-on views at 1 to 2 m; angled views and docking distances below 1 m were not tested.
- Pixel noise and pose jitter are assumed values.
- The scene draws LiDAR rays into camera images, which a real camera would not see.
- One fixed R is a compromise. A range-dependent R (σ_ρ = 0.0045 d²) and subtracting the range bias would be better.
