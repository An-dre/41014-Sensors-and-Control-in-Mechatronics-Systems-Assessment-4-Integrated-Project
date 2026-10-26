# Part C: extended Kalman filter (EKF)
# Wheel odometry predicts the pose; the camera corrects it.
#
# x = [world x, world y, heading]       P = uncertainty in that estimate
# u = [forward speed, turn rate]       z = [marker range, marker bearing]
# m = [marker world x, marker world y]
# Use metres and radians. In NumPy, @ multiplies matrices; .T transposes them.
# np.linalg.inv() calculates a matrix inverse, as used in the EKF formulas.
# The diagonal of P contains variances; their square roots are standard deviations.
#
# From the outer project folder, run: python tb3_project\ekf.py
# This runs the small example at the bottom without a camera or simulator.

import numpy as np
import model


# --- C1. Noise settings ------------------------------------------------------
# Both Q and R below are starting assumptions, not measured results.
# Q adds motion uncertainty at each 0.02 s prediction (50 Hz).
# Starting standard deviations: 1 mm in x/y and 0.2 degrees in heading.
# Square standard deviations to get variances.
Q = np.diag([0.001**2, 0.001**2, np.radians(0.2)**2])

# R describes uncertainty in one camera measurement.
# Starting standard deviations: 3 cm in range and 1 degree in bearing.
# These are assumptions: replace them with B7's measured camera noise.
R = np.diag([0.03**2, np.radians(1.0)**2])

# Zero off-diagonal entries assume independent noise components.
# To tune: first measure camera noise in B7, then compare the original Q with
# Q * 0.1 and Q * 10 on the same movement tests. Separately try R * 0.1.
# Compare position/heading errors, NEES and rejected updates. Start each test
# from the original matrices, so the scale factors do not keep accumulating.
# Increasing Q increases prediction uncertainty. Decreasing R makes accepted
# camera readings more influential, but can also make the gate stricter.

# Approximate 99% chi-square threshold for our two measurement values.
GATE_THRESHOLD = 9.21
# These count update calls, including invalid readings, not frames with no marker.
# The trial logger still needs connecting to save these counts for the report.
updates_accepted = 0
updates_rejected = 0


def reset_update_counts():
    # Call before each trial if you want separate counts for that trial.
    global updates_accepted, updates_rejected
    updates_accepted = 0
    updates_rejected = 0


def wrap_angle(angle):
    # Keep angles between -pi and pi, so +179 and -179 degrees are close.
    return (angle + np.pi) % (2 * np.pi) - np.pi


# --- C2. Predict the next pose -----------------------------------------------
def predict(x, P, u, dt):
    # F tells us how errors in the current pose affect the next pose.
    # Calculate it BEFORE changing x.
    F = model.jacobian_F(x, u, dt)

    x_pred = model.f(x, u, dt)
    x_pred[2] = wrap_angle(x_pred[2])

    # Move the old uncertainty forward, then add motion uncertainty.
    # Q is chosen for the normal dt = 0.02 s; retune Q if that rate changes.
    P_pred = F @ P @ F.T + Q
    return x_pred, P_pred


# --- C3. Expected marker measurement -----------------------------------------
def h(x, m):
    # If our estimated pose were correct, what range and bearing would we see?
    dx = m[0] - x[0]
    dy = m[1] - x[1]

    rng = np.sqrt(dx**2 + dy**2)
    # World direction to marker minus robot heading = robot-relative bearing.
    bearing = wrap_angle(np.arctan2(dy, dx) - x[2])
    return np.array([rng, bearing])


def jacobian_H(x, m):
    dx = m[0] - x[0]
    dy = m[1] - x[1]
    rng_squared = dx**2 + dy**2
    rng = np.sqrt(rng_squared)

    # At the marker position, bearing is undefined and we would divide by zero.
    if rng < 1e-9:
        raise ValueError("The robot and marker must be at different positions.")

    # Derivatives of h: rows are range/bearing; columns are x/y/heading.
    H = np.array([
        [-dx / rng, -dy / rng, 0.0],
        [dy / rng_squared, -dx / rng_squared, -1.0],
    ])
    return H


# --- C4. Correct the pose using the camera -----------------------------------
def update(x, P, z, m):
    global updates_accepted, updates_rejected

    # Ignore unusable camera values.
    if not np.all(np.isfinite(z)) or z[0] <= 0:
        updates_rejected += 1
        return x, P

    H = jacobian_H(x, m)

    # Innovation = what the camera measured minus what we expected.
    innovation = z - h(x, m)
    innovation[1] = wrap_angle(innovation[1])

    S = H @ P @ H.T + R       # uncertainty in this difference
    S_inverse = np.linalg.inv(S)

    # Compare the difference with its uncertainty. Reject a very large mismatch.
    score = innovation.T @ S_inverse @ innovation
    if score > GATE_THRESHOLD:
        updates_rejected += 1
        return x, P

    K = P @ H.T @ S_inverse   # Kalman gain: how much to correct the pose
    x_updated = x + K @ innovation
    x_updated[2] = wrap_angle(x_updated[2])

    P_updated = (np.eye(3) - K @ H) @ P   # np.eye(3) is the identity matrix I
    # Remove tiny asymmetry caused by rounding during matrix calculations.
    P_updated = (P_updated + P_updated.T) / 2

    updates_accepted += 1
    return x_updated, P_updated


# --- C5. Check whether the uncertainty matches the error ---------------------
# For a consistent three-state estimate, NEES has expected value 3.
# High NEES: error is large compared with the uncertainty we claimed.
# Low NEES: claimed uncertainty is large compared with the error.
# A time average from one run is a rough check, not proof of consistency.
# E4's mean_nees() in evaluate.py still needs to call this on saved trial data.
def nees(x_hat, P, x_true):
    # True pose is used only for evaluation, never to guide the robot.
    error = np.array(x_true, dtype=float) - np.array(x_hat, dtype=float)
    error[2] = wrap_angle(error[2])

    # A large error compared with P gives a large NEES.
    return float(error.T @ np.linalg.inv(P) @ error)


if __name__ == "__main__":
    # A small example that needs no simulator.
    # These checks do not replace noise tuning, robot trials or assessment plots.
    x = np.array([0.0, 0.0, 0.0])
    P = np.diag([0.5, 0.5, 0.1])
    m = np.array([2.0, 0.0])

    x_pred, P_pred = predict(x, P, [1.0, 0.0], 0.02)
    print("C2: predicted pose:", x_pred, " expected [0.02, 0, 0]")
    np.testing.assert_allclose(x_pred, [0.02, 0.0, 0.0])

    print("C3: expected range and bearing:", h(x, m), " expected [2, 0]")
    np.testing.assert_allclose(h(x, m), [2.0, 0.0])

    # Seeing the marker at 1.5 m suggests the robot is near x = 0.5.
    x2, P2 = update(x, P, np.array([1.5, 0.0]), m)
    print("C4: corrected pose:", x2, " x should be close to 0.5")
    assert 0 < x2[0] < 0.5

    # This impossible reading should be rejected.
    x3, P3 = update(x2, P2, np.array([20.0, 0.0]), m)
    np.testing.assert_allclose(x3, x2)
    print("C4: accepted:", updates_accepted, " rejected:", updates_rejected)

    score = nees(np.zeros(3), np.eye(3), np.ones(3))
    print("C5: NEES:", score, " expected 3")
    np.testing.assert_allclose(score, 3.0)
    print("Basic checks passed. Noise tuning and robot trials are still needed.")
