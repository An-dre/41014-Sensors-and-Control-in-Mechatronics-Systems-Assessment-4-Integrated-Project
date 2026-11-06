"""Part A: state-space model of the TurtleBot3 Waffle Pi (Gabriel).

The unicycle model, its Jacobians F and G (EKF), the linear tracking-error model A, B (LQR)
and the controllability / observability matrices. Self-checks:
    python -m part_a_modelling.model

State   x = [x, y, theta]   position in the world frame (m) and heading (rad)
Input   u = [v, omega]      forward speed (m/s) and turn rate (rad/s)
Robot numbers come from robotis_tb3/turtlebot3_waffle_pi.xml.
"""

import sys
from pathlib import Path

import numpy as np

if __package__ in (None, ""):  # run as a file, not with -m: let Python find the part_* packages
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


r = 0.033      # wheel radius (m), checked against the tyre mesh
L = 0.288      # wheel separation (m), wheels at y = +0.144 and y = -0.144
W_MAX = 7.88   # max wheel speed (rad/s)


# --- A1. wheels <-> body ------------------------------------------------------
# A differential drive moves at the average of the two wheel rim speeds and
# turns at their difference over the separation:
#     v     = r * (w_right + w_left) / 2
#     omega = r * (w_right - w_left) / L
def wheels_to_body(w_left, w_right):
    v = r * (w_right + w_left) / 2
    omega = r * (w_right - w_left) / L
    return v, omega


def body_to_wheels(v, omega):
    # the inverse of the above. Solve the two equations for w_left and w_right.
    # The controller needs this to turn (v, omega) into wheel commands.
    w_left = (v - (omega * L) / 2) / r
    w_right = (v + (omega * L) / 2) / r
    return w_left, w_right


# --- A2. motion model (discrete, Euler) ---------------------------------------
#     x_{k+1}     = x_k + v_k * dt * cos(theta_k)
#     y_{k+1}     = y_k + v_k * dt * sin(theta_k)
#     theta_{k+1} = theta_k + omega_k * dt
def f(x, u, dt):
    # x = np.array([x, y, theta]),  u = np.array([v, omega])
    # Use the current heading to split forward motion into world x and y.
    # A new array avoids changing the estimate that was passed into the function.
    theta = x[2]
    v, omega = u
    x_next = np.array([
        x[0] + v * dt * np.cos(theta),
        x[1] + v * dt * np.sin(theta),
        theta + omega * dt,
    ])
    return x_next


# --- A3. Jacobians of f (the EKF predict step needs these) --------------------
# F = df/dx  (3x3)   G = df/du (3x2)
# Differentiate the three lines above. Only cos/sin terms depend on theta.
def jacobian_F(x, u, dt):
    F = np.eye(3)
    # These derivatives describe how heading uncertainty affects position.
    theta = x[2]
    v = u[0]
    F[0, 2] = -v * dt * np.sin(theta)
    F[1, 2] = v * dt * np.cos(theta)
    return F


def jacobian_G(x, u, dt):
    G = np.zeros((3, 2))
    # Column 0 describes the effect of speed; column 1 describes turn rate.
    theta = x[2]
    G[0, 0] = dt * np.cos(theta)
    G[1, 0] = dt * np.sin(theta)
    G[2, 1] = dt
    return G


# --- A4. linear error model for LQR --------------------------------------------
# LQR needs a LINEAR model. Linearise about a reference point that moves at
# (v_ref, omega_ref). Define the tracking error as  (reference - robot), rotated
# into the ROBOT frame:
#     e = [e_x, e_y, e_th]   (ahead, to the left, heading)
# and the input change as  du = u - u_ref = [dv, domega].
# With Euler discretisation (dt = control period) the standard result is
#     e_{k+1} = A e_k + B du_k
#     A = [[ 1,            omega_ref*dt,  0        ],
#          [-omega_ref*dt, 1,             v_ref*dt ],
#          [ 0,            0,             1        ]]
#     B = [[-dt,  0  ],
#          [ 0,   0  ],
#          [ 0,  -dt ]]
# Derivation (PART_A_MODELLING.md): write the error in the world frame, rotate it into
# the robot frame, differentiate, drop products of small terms. Assumptions: small
# errors, no slip, constant v_ref over one step.
# Sanity check on the signs: if the robot is behind the reference, e_x > 0, and a
# positive dv must make e_x shrink. B[0,0] = -dt does exactly that.
def linear_error_model(v_ref, omega_ref, dt):
    A = np.array([[ 1,            omega_ref*dt,  0        ],
                  [-omega_ref*dt, 1,             v_ref*dt ],
                  [ 0,            0,             1        ]])
    B = np.array([[-dt,  0  ],
                  [ 0,   0  ],
                  [ 0,  -dt ]])
    return A, B


# --- A5. controllability and observability ------------------------------------
# Controllability matrix  Ctrb = [B, AB, A^2 B]       full rank (3) -> controllable
# Observability matrix    Obsv = [C; CA; CA^2]        full rank (3) -> observable
# Cases checked below and reported in PART_A_MODELLING.md:
#   * Ctrb with v_ref = 0 against v_ref > 0 (why the robot docks while still moving)
#   * C for encoders only (nothing measures x, y, theta directly)
#   * C = H, the range + bearing Jacobian from ekf.py, along a short path
def controllability_matrix(A, B):
    Ctrb = np.hstack([B, A @ B, A @ A @ B])
    return Ctrb


def observability_matrix(A, C):
    Obsv = np.vstack([C, C @ A, C @ A @ A])
    return Obsv


if __name__ == "__main__":
    from part_c_estimation import ekf
    # self-checks: python -m part_a_modelling.model
    print("wheels_to_body(5, 5) ->", wheels_to_body(5.0, 5.0), " expect v=0.165, omega=0")
    print("wheels_to_body(-5, 5) ->", wheels_to_body(-5.0, 5.0), " expect v=0, omega=1.146")
    print("body_to_wheels(0.165, 0) ->", body_to_wheels(0.165, 0.0), " expect (5, 5)")
    x = np.array([0.0, 0.0, np.pi / 2])
    print("f(x, [1, 0], 0.1) ->", f(x, np.array([1.0, 0.0]), 0.1), " expect [0, 0.1, 1.5708]")
    A, B = linear_error_model(0.2, 0.0, 0.02)
    if A is not None:
        print("rank Ctrb, v_ref=0.2 ->", np.linalg.matrix_rank(controllability_matrix(A, B)), " expect 3")
        A0, B0 = linear_error_model(0.0, 0.0, 0.02)
        print("rank Ctrb, v_ref=0   ->", np.linalg.matrix_rank(controllability_matrix(A0, B0)), " expect 2")

        m = np.array([2.888, 0.0])   # dock marker position from sceneforturtle.xml
        x_test = np.array([0.0, 0.0, 0.0])

        # robot moving forward at 0.2 m/s
        F_move = jacobian_F(x_test, np.array([0.2, 0.0]), 0.02)

        C_none = np.zeros((1, 3))           # encoders only, nothing measures x, y, theta
        C_pose = np.eye(3)                  # camera gives x, y, theta directly
        C_rb = ekf.jacobian_H(x_test, m)    # camera gives range + bearing to the marker

        print("rank Obsv, encoders only        ->", np.linalg.matrix_rank(observability_matrix(F_move, C_none)), " expect 0")
        print("rank Obsv, full pose            ->", np.linalg.matrix_rank(observability_matrix(F_move, C_pose)), " expect 3")
        print("rank Obsv, range+bearing, parked ->", np.linalg.matrix_rank(observability_matrix(np.eye(3), C_rb)), " expect 2")

        x0 = np.array([0.0, 0.3, 0.2])      # start pose, not lined up with the marker
        for u_try in [np.array([0.2, 0.0]), np.array([0.2, 0.5]), np.array([0.0, 0.5])]:
            x1 = f(x0, u_try, 0.02)
            x2 = f(x1, u_try, 0.02)
            F0 = jacobian_F(x0, u_try, 0.02)
            F1 = jacobian_F(x1, u_try, 0.02)

            H0 = ekf.jacobian_H(x0, m)
            H1 = ekf.jacobian_H(x1, m)
            H2 = ekf.jacobian_H(x2, m)

            Obsv_traj = np.vstack([H0, H1 @ F0, H2 @ F1 @ F0])
            print("rank Obsv, range+bearing along path, u =", u_try, "->", np.linalg.matrix_rank(Obsv_traj), " expect 2")
