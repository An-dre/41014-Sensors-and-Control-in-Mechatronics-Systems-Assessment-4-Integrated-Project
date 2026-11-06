"""Part C: wheel odometry prediction and camera range/bearing correction.

State: [world x, world y, heading], in metres and radians.
Q/R are simulation design values; see PART_C_ESTIMATION.md for tuning evidence.
"""

import numpy as np

from part_a_modelling import model

Q = np.diag([0.001**2, 0.001**2, np.radians(0.2)**2])  # per 20 ms prediction
R = np.diag([0.03**2, np.radians(1.0)**2])              # per camera measurement
GATE_THRESHOLD = 9.21  # approximate 99% chi-square threshold for two measurements
updates_accepted = updates_rejected = 0
last_nis = np.nan


def reset_update_counts():
    """Reset before each trial; the runners process one trial at a time."""
    global updates_accepted, updates_rejected, last_nis
    updates_accepted = updates_rejected = 0
    last_nis = np.nan


def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def predict(x, P, u, dt, process_covariance=None):
    F = model.jacobian_F(x, u, dt)
    x_pred = model.f(x, u, dt)
    x_pred[2] = wrap_angle(x_pred[2])
    motion_noise = Q if process_covariance is None else process_covariance
    # Residual motion-model uncertainty, scaled from the normal 20 ms interval.
    P_pred = F @ P @ F.T + motion_noise * (dt / 0.02)
    return x_pred, (P_pred + P_pred.T) / 2


def h(x, m):
    dx, dy = m[0] - x[0], m[1] - x[1]
    return np.array([np.hypot(dx, dy), wrap_angle(np.arctan2(dy, dx) - x[2])])


def jacobian_H(x, m):
    dx, dy = m[0] - x[0], m[1] - x[1]
    distance_squared = dx**2 + dy**2
    distance = np.sqrt(distance_squared)
    if distance < 1e-9:
        raise ValueError("Bearing is undefined at the marker position.")
    return np.array([[-dx / distance, -dy / distance, 0.],
                     [dy / distance_squared, -dx / distance_squared, -1.]])


def update(x, P, z, m, measurement_covariance=None):
    global updates_accepted, updates_rejected, last_nis
    last_nis = np.nan
    z = np.asarray(z, dtype=float)
    if z.shape != (2,) or not np.all(np.isfinite(z)) or z[0] <= 0:
        updates_rejected += 1
        return x, P

    H = jacobian_H(x, m)
    camera_noise = R if measurement_covariance is None else measurement_covariance
    innovation = z - h(x, m)
    innovation[1] = wrap_angle(innovation[1])
    S = H @ P @ H.T + camera_noise
    last_nis = float(innovation @ np.linalg.solve(S, innovation))
    if last_nis > GATE_THRESHOLD:
        updates_rejected += 1
        return x, P

    K = np.linalg.solve(S, H @ P).T
    x_updated = x + K @ innovation
    x_updated[2] = wrap_angle(x_updated[2])
    # Joseph form protects covariance positivity against numerical rounding.
    residual = np.eye(3) - K @ H
    P_updated = residual @ P @ residual.T + K @ camera_noise @ K.T
    updates_accepted += 1
    return x_updated, (P_updated + P_updated.T) / 2


def noise_matrices(q_scale=1., r_scale=1., measurement_noise=None):
    """Load optional Part B NPZ key R in [range metres, bearing radians] order."""
    if not all(np.isfinite(s) and s > 0 for s in (q_scale, r_scale)):
        raise ValueError("Q/R scales must be finite and positive.")
    camera_noise = R.copy()
    if measurement_noise is not None:
        with np.load(measurement_noise, allow_pickle=False) as saved:
            camera_noise = saved["R"].copy()
        if (camera_noise.shape != (2, 2) or not np.all(np.isfinite(camera_noise))
                or not np.allclose(camera_noise, camera_noise.T)):
            raise ValueError("R must be a finite symmetric 2 by 2 covariance.")
        try:
            np.linalg.cholesky(camera_noise)
        except np.linalg.LinAlgError as exc:
            raise ValueError("R must be positive definite.") from exc
    return Q * q_scale, camera_noise * r_scale


def nees(x_hat, P, x_true):
    """Evaluation only: consistency score using the full covariance."""
    error = np.asarray(x_true, dtype=float) - np.asarray(x_hat, dtype=float)
    error[2] = wrap_angle(error[2])
    return float(error @ np.linalg.solve(P, error))


def estimator_metrics(estimate, P, truth, nis):
    """Evaluation only; truth never enters prediction or correction."""
    error = np.asarray(estimate) - np.asarray(truth)
    error[:, 2] = wrap_angle(error[:, 2])
    scores = [nees(x, covariance, actual) for x, covariance, actual in zip(estimate, P, truth)]
    coverage = np.mean(np.abs(error) <= 2 * np.sqrt(np.diagonal(P, axis1=1, axis2=2)), axis=0)
    finite_nis = np.asarray(nis)[np.isfinite(nis)]
    return dict(ekf_position_rmse=float(np.sqrt(np.mean(np.sum(error[:, :2]**2, axis=1)))),
                heading_rmse_deg=float(np.degrees(np.sqrt(np.mean(error[:, 2]**2)))),
                mean_nees=float(np.mean(scores)), coverage_x=float(coverage[0]),
                coverage_y=float(coverage[1]), coverage_heading=float(coverage[2]),
                mean_nis_all=float(np.mean(finite_nis)) if len(finite_nis) else None)
