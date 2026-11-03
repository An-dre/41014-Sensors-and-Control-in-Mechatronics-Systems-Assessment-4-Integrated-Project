# Part B: Sensing (Dinel)

Part B turns the simulators exact wheel angles, LiDAR ranges and camera images into realistic sensor measurements, and converts them into the form the estimator and planner use. It supplies the EKF's odometry input `u = [v, omega]`, its camera measurement `z = [range, bearing]` and a measured camera covariance `R`. It also supplies the noisy LiDAR scan.

## Run and reproduce

From the repository root, install `requirements.txt`, then run:

```bash
python src\sensing.py              # self-check, expect range ≈ 1.888 m corrected
python src\sensing.py calib        # saves src\calib.npz, prints K vs true K and reprojection error
python src\sensing.py noise        # saves src\camera_R_calibrated.npz, prints the noise table

# Full pipeline with Part B's calibrated camera and measured R.
python src\run_pipeline.py --calibration src\calib.npz --measurement-noise src\camera_R_calibrated.npz --output src\results_pipeline\calibrated
```

B7 uses `calib.npz` when it exists and names its output after the K it used, so the two R files never overwrite each other. To regenerate the nominal table, rename `calib.npz` first. B7 uses fixed random seeds, so its tables are repeatable.

## Encoders and odometry (B1)

The simulator reports exact wheel angles. THe hardware encoder counts 4096 ticks per revolution, so each angle is rounded to whole ticks and given Gaussian noise:

```
tick = 2*pi / 4096 # 0.00153 rad
angle_meas = round(angle / tick) * tick + N(0, tick^2)
```

The noise standard deviation is one tick. Rounding alone gives up to half a tick of error, and the extra noise stands in for gear backlash and small slip of a similar size. This is a modelling choice tied to the sensors resolution.

At 50Hz odometry differences two readings and calls Part A's `wheels_to_body()`:

```
w_left  = (angle_left_now  - angle_left_prev)  / dt
w_right = (angle_right_now - angle_right_prev) / dt
v, omega = wheels_to_body(w_left, w_right) # u for the EKF predict step
```

Differencing over dt = 0.02 s multiplies angle noise by 50, giving about 2.6 mm/s on v (about 1% of the 0.26 m/s top speed) and 0.018 rad/s on omega. This random noise is small. The larger odometry errors are systematic (0.6% straight-line slip, effective wheel separation 0.290 m instead of 0.288 m), and they are what Part C's Q has to cover.

## LiDAR (B2)

The LDS-01 datasheet quotes ±15 mm for 0.12–0.5 m and ±5% for 0.5–3.5 m. We read "±" as a 2σ bound (about 95% of readings):

```
sigma = 7.5 mm          if range < 0.5 m
sigma = 0.025 * range   otherwise
```

Each beam gets independent Gaussian noise, readings are clipped to the rated 0.12–3.5 m, and beams with no return (-1) are set to 3.5 m so the planner treats them as free space. The step in σ at 0.5 m (7.5 to 12.5 mm) comes from the datasheet itself

## Camera model and calibration (B3, B4)

The rendered camera is a 640 × 480 pinhole with a 60° vertical field of view:

```
f = (480/2) / tan(60deg/2) = 415.69 px
K = [[f, 0, 320], [0, f, 240], [0, 0, 1]]
```

We checked this against the scene: fovy 60°, image 640 × 480, and the camera 0.07 m ahead of the base origin.

The project scene has no calibration target, so `calib_scene.xml` adds an 8 × 6 chessboard of 0.1 m squares (7 × 5 inner corners) to the same robot and camera. K belongs to the camera, so the result applies to the project scene. The robot was placed at 27 poses (3 distances × 3 sideways offsets × 3 headings) and `cv2.calibrateCamera` was run on the detected corners.

| Parameter | True K (B3) | Calibrated (B4) | Difference |
|---|---|---|---|
| fx | 415.69 | 415.80 | +0.11 px (0.03%) |
| fy | 415.69 | 415.78 | +0.09 px (0.02%) |
| cx | 320.00 | 319.62 | −0.38 px |
| cy | 240.00 | 239.33 | −0.67 px |

All 27 photos were used, with 0.097px reprojection error and almost no distortion. The focal length confirms the B3 formula. B3 put the image centre at 320, but because pixels are measured at their centres the true middle is 319.5. Calibration found this half pixel offset and it explains the bearing bias in B7. B7 shows that this half pixel offset caused a bearing bias.

The camera can only translate and yaw, never tilt, so the views are less varied than a hand held calibration. That is enough here only because the simulated camera has no distortion to estimate.

## Marker pose, frames and range/bearing (B5, B6)

`detect_marker()` finds ArUco DICT_4X4_50 id 0 and solves its pose with `solvePnP(SOLVEPNP_IPPE_SQUARE)`. The result is the marker centre in the OpenCV camera frame (x right, y down, z forward). `cam_to_base()` rotates it into the base frame and adds the camera offset, and `range_bearing()` reduces it to the floor plane:

```
p_base = R_BC @ p_cam + t_cam
R_BC = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]    # cam z -> base x, cam x -> base -y, cam y -> base -z
range = sqrt(x_b^2 + y_b^2); bearing = atan2(y_b, x_b)
```

The full chain is marker -> OpenCV camera -> base -> world, where the last step is the EKF's pose estimate. `sensing.py` keeps the scaffold's marker size (0.2m) and camera offset (0.086m). `sensor_adapter.py` rescales them to the project scene's 0.16 m black square and camera translation [0.07, 0, 0.15] m. B7 applies the same correction, so it measures exactly what the EKF receives.

A detection is rejected and counted in `REJECTS` if:

| Check | Threshold | Reason |
|---|---|---|
| Wrong id | not 0 | only the dock marker is a landmark |
| Too small | shortest side < 30 px | small markers give unreliable poses |
| Bad pose | behind the camera, non-finite, or > 3.5 m | impossible poses |

The 30 px threshold limits camera fixes to about `f * 0.16 / 30 = 2.2 m`. The check was added after B7 found 10 of 150 frames at 1.5 m returning nan poses.

## Measured camera noise and R (B7)

Refer to results_part_b

## Use in the pipeline

One closed-loop trial each from the west start:

| R used | Docked | EKF position RMSE | Camera fixes accepted |
|---|---|---|---|
| Part C design R | yes | 1.5 cm | 424 |
| B7 nominal R | yes | 1.4 cm | 375 |

The measured R works in the closed loop. With one trial each, the 0.1 cm difference is not significant.

Part C's design R uses 3 cm range and 1° bearing. B7 agrees on range but measures bearing noise 10 to 20 times smaller, consistent with Part C's low mean NIS (0.159 against an ideal of 2). B7 also provides the distance dependent range bias Part C has.

## Problems found and fixed

- **Range 25% too long.** The first corrected range read 2.46 m against a true 1.89 m. Measuring `aruco_0.png` showed the black square is 200 of its 250 pixels, so the detected marker is 0.16 m, not 0.2 m. `sensor_adapter.py` and B7 apply this correction.
- **Calibration failing.** Calibration first used only 1 of 27 photos because MuJoCo drew the LiDAR rays across the chessboard. Making the rays transparent in `calib_scene.xml` fixed it.

## Limits
- The encoder σ and the "± = 2σ" reading of the LiDAR datasheet are modelling choices.

Ground truth is never used in any of Part B. `noisy_encoders`, `odometry`, `noisy_lidar`, `detect_marker`, `cam_to_base` and `range_bearing` use only sensor data. B7 and B4 are offline experiments that place the robot themselves and compare against those known poses. They are not part of the closed loop.

## Generative AI
Claude (Anthropic) was used to help design, write(this page), debug, and test this part.
