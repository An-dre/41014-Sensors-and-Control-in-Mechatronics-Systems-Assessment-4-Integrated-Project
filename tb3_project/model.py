# model.py - PART A (20 marks). Owner: Person 4.
# The state-space model of the TurtleBot, its Jacobians, and the
# controllability / observability checks. Fill in every TODO.
#
# State   x = [x, y, theta]   position in the world frame (m) and heading (rad)
# Input   u = [v, omega]      forward speed (m/s) and turn rate (rad/s)
# Robot numbers come from robot/tb3_waffle_pi_sensors.xml:

import numpy as np

r = 0.033      # wheel radius (m)
L = 0.288      # wheel separation (m), wheels at y = +0.144 and y = -0.144
W_MAX = 7.88   # max wheel speed (rad/s)


# --- A1. wheels <-> body ------------------------------------------------------
# A differential drive moves at the average of the two wheel rim speeds and
# turns at their difference over the separation:
#     v     = r * (w_right + w_left) / 2
#     omega = r * (w_right - w_left) / L
def wheels_to_body(w_left, w_right):
    v = None       # TODO A1
    omega = None   # TODO A1
    return v, omega


def body_to_wheels(v, omega):
    # the inverse of the above. Solve the two equations for w_left and w_right.
    # The controller needs this to turn (v, omega) into wheel commands.
    w_left = None    # TODO A1
    w_right = None   # TODO A1
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
# Derive this yourself on the page: write the error in the world frame, rotate it
# into the robot frame, differentiate, drop products of small terms. State the
# assumptions (small errors, no slip, constant v_ref over one step).
# Sanity check on the signs: if the robot is behind the reference, e_x > 0, and a
# positive dv must make e_x shrink. B[0,0] = -dt does exactly that.
def linear_error_model(v_ref, omega_ref, dt):
    A = None   # TODO A4
    B = None   # TODO A4
    return A, B


# --- A5. controllability and observability ------------------------------------
# Controllability matrix  Ctrb = [B, AB, A^2 B]       full rank (3) -> controllable
# Observability matrix    Obsv = [C; CA; CA^2]        full rank (3) -> observable
# Use the same construction you used in the tutorial / Assessment 3.
# Things the marker will look for on the page:
#   * rank of Ctrb when v_ref = 0  vs  v_ref > 0  (what does it tell you about docking?)
#   * C for "encoders only" (nothing measures x,y,theta directly -> what rank?)
#   * C = H (the measurement Jacobian from ekf.py) for range+bearing to the marker
def controllability_matrix(A, B):
    # TODO A5
    return None


def observability_matrix(A, C):
    # TODO A5
    return None


if __name__ == "__main__":
    # self-checks: run  python model.py
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
