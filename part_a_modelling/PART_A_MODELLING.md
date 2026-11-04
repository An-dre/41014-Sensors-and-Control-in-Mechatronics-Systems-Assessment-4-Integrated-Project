# Part A Gabriel Modelling Section

### Robot Defintion

Verified Parameters:

- L (wheel seperation) = 0.288
- max commanded wheel speed limits = -7.88, 7.88
- Wheel radius = 0.033m
- Python updates the model every 0.02 seconds

### Assumptions

- **No Slip:** The speed is very low and the contact friction is pretty high enough that the slip is negligible
- **Flat Floor:** Also being a simulated environment, we can safely assume that the floor we create is flat with no uneven surfaces
- **Kinematic Model:** In a dynamic model, it would include motor torque, wheel interia, friction, etc... but in a kinematic does not include that, assuming that the v, ω act straight away which the model only tracks position and heading. We assume this as the Turtlebot is small and slow, which the motor have their own speed controllers so the commandedd wheel speed is reached within one dt.

### States and Inputs

```math
\mathbf{x} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}
```

This state holds where the robot currently is (x,y) and θ is which way the robot faces. All of this is measured in the world frame, with the origin in the starting pose. The units are in m and rad

Inputs:

```math
\mathbf{u} = \begin{bmatrix} v \\ \omega \end{bmatrix}
```

The two inputs that the robot takes are the commanded velocity, (how fast it goes foward in m/s) and how fast it turns in rad/s. Positive ω is considered anticlockise

### Controller (Wheels)

Velocity:

```math
v = \frac{r(\omega_R + \omega_L)}{2}
```

Turn Speed (Omega):

```math
\omega = \frac{r(\omega_R - \omega_L)}{L}
```

The controller can only work in v and ω, due to how the error model is designed. The MuJoCo robot can only accept left and right wheel speeds. The two formulas convert one to the other as v is the average between the two wheel rim speeds which their difference is divided by the gap between the wheels.

### Inverse

Left Wheel:

```math
\omega_L = \frac{v - \omega L/2}{r}
```

Right Wheel:

```math
\omega_R = \frac{v + \omega L/2}{r}
```

We can get the individual wheel speeds by taking the inverse of the above equations. Body To wheels. This is used in every control step to send wheel commands to the robot.

### Continous Time Model

```math
\dot{x} = v\cos\theta
```

```math
\dot{y} = v\sin\theta
```

```math
\dot{\theta} = \omega
```

Above is the standard unicycle model, where the robot can only move along its heading and spin, but can't move sideways. These models are not linear due to v being multiplied by cos θ and sin θ, which is an input multiplied by a function of a state. This matters as the EKF and LQR requires a linear model which we have to linearise.

We have two linearisations, one used by the EKF and one used by the LQR.

### Linearisations 1 - Jacobians F and G

The idea is that near the current estimate, we replace the curved function f with its slopem which the slope of the function of several variables is a matrix of partial deratives which we get the Jacobians F and G.

```math
x_{k+1} = x_k + v_k\Delta t\cos\theta_k
```

```math
y_{k+1} = y_k + v_k\Delta t\sin\theta_k
```

```math
\theta_{k+1} = \theta_k + \omega_k\Delta t
```

From the discrete model, we differentiated each line with respect to each state to get the Jacobian F

```math
F = \frac{\partial f}{\partial \mathbf{x}} = \begin{bmatrix} 1 & 0 & -v_k \Delta t \sin\theta_k \\ 0 & 1 & v_k \Delta t \cos\theta_k \\ 0 & 0 & 1 \end{bmatrix}
```

F Jacobian describes how small errors in the robots current positon an headidng affect its next predricted pose.

From this, we can call F[0,2] and F[1,2] to tell the EKF how far off x and y end up if theat is slightly wrong, which is scaled by the distance moved that step (v Δt) which is the `Jacobian_F()` function in `model.py`.

Then we differentiated each line with respect to each input to get Jacobian G

```math
G = \frac{\partial f}{\partial \mathbf{u}} = \begin{bmatrix} \Delta t \cos\theta_k & 0 \\ \Delta t \sin\theta_k & 0 \\ 0 & \Delta t \end{bmatrix}
```

This Jacobian, pretty much maps the uncertainty or small changes in the speed andd yaw rate into each pose step. Each column represnts an input with each row representing a state.

- **Column 0 (v)**, pushes the speed up a bit, moving along the heading via cos θ / sin θ.
- **Column 1 (ω)**, pushes the turn rate up a bit by only θ changes by Δt per unit of ω.

F and G are recomputedd every step at the current estimate and input because they depend on θ and v. These are used by the EKF, which uses them in the predict step F to push the covariance forward and G to add the input noise.

### Linearisations 2 - Error Model A and B (LQR)

Unfortunately LQR cannot use F and G directly as they both depend on θ (which is constantly changing) so insteadd we model the error to the reference path in the robots own current frame where it stays nearly linear.

First we define the error rotated into the robot frame

```math
\mathbf{e} = \begin{bmatrix} e_x \\ e_y \\ e_\theta \end{bmatrix}, \qquad \delta\mathbf{u} = \begin{bmatrix} v - v_{ref} \\ \omega - \omega_{ref} \end{bmatrix}
```

e_x means how far behind/ahead of the reference point, e_y = how far left/right, e_θ = the heading error of the robot and δu represents the adjustments to the forward speed and turning rate relative to their reference values

Then we differentiate the tracking errors to obtain their dynamics. Small angle approximations were applied, and products of two small terms were neglected. Euler discretisation was applied using the update interval Δt to obtain the discrete A and B matrices.

```math
\mathbf{e}_{k+1} = A\,\mathbf{e}_k + B\,\delta\mathbf{u}_k
```

```math
A = \begin{bmatrix} 1 & \omega_{ref}\Delta t & 0 \\ -\omega_{ref}\Delta t & 1 & v_{ref}\Delta t \\ 0 & 0 & 1 \end{bmatrix}
```

```math
B = \begin{bmatrix} -\Delta t & 0 \\ 0 & 0 \\ 0 & -\Delta t \end{bmatrix}
```

Ff the reference is ahead, e_x is positive so the robot must speed up (δv > 0) to reduce e_x. For a positive δv to make e_x smaller, B[0,0] has to be negative, which gives B[0,0] = −Δt.

No input moves the robot sideways, which is why row 2 of B is zero. To reduce e_y it must first turn (e_θ) and then drive forward; this link is the A[1,2] = v_ref Δt term. If v_ref = 0 that entry is 0 and e_y cannot be corrected, which is why the system is not fully controllable when stopped.

This is the `linear_error_model()` function in `model.py` which returns A and B.

### Meausrement Model

The camera detects an ArUco marker at the known world position (m_x, m_y). The measurement model predicts the planar range ρ and relative bearing β to this marker from the robot’s position and heading. In this model, both measurements are defined relative to the robot’s reference point.

```math
z = h(\mathbf{x}) = \begin{bmatrix} \rho \\ \beta \end{bmatrix} = \begin{bmatrix} \sqrt{(m_x - x)^2 + (m_y - y)^2} \\ \mathrm{atan2}(m_y - y,\; m_x - x) - \theta \end{bmatrix}
```

The EKF requires the Jacobian H of the measurement function h with respect to the state. This matrix describes how small changes in the estimated position and heading affect the predicted range and bearing.

```math
H = \frac{\partial h}{\partial \mathbf{x}} = \begin{bmatrix} -\dfrac{m_x - x}{\rho} & -\dfrac{m_y - y}{\rho} & 0 \\ \dfrac{m_y - y}{\rho^2} & -\dfrac{m_x - x}{\rho^2} & -1 \end{bmatrix}
```

The first row of H describes changes in range. Moving towards the marker decreases the range, while moving away increases it. The second row describes changes in bearing caused by sideways movement relative to the marker or by rotation. Its final entry is −1 because increasing the robot’s heading decreases the marker’s relative bearing by the same angle.

The measurement prediction is implemented in `h()`, while its Jacobian is calculated by `jacobian_H()`. These functions are used during the EKF measurement update.

### Controllability

Controllability describes whether the available inputs can move the tracking-error state from any initial value to a desired value, including zero. For this three-state model, the controllability matrix must have rank 3.

```math
\mathcal{C} = \begin{bmatrix} B & AB & A^2B \end{bmatrix}
```

| Case | rank | Controllable? |
|---|---|---|
| v_ref = 0.2 m/s | 3 | yes |
| v_ref = 0 | 2 | no |

When both reference speeds are zero, the second row of B is zero and there is no coupling from heading error into lateral error. Therefore, e_y remains unchanged in the stationary linearised model. Only two independent error directions can be controlled, giving a controllability rank of 2.

For docking, this means that an LQR controller designed around a stationary reference cannot correct a remaining lateral offset. The robot should correct its lateral alignment during the approach and stop once it reaches the required docking pose. This does not mean the robot must move continuously: if it stops with a sideways offset, it can still turn and drive again using an appropriate manoeuvre.

### Observability

Observability describes whether the robot’s states can be determined from the available measurements and known inputs over time. For the three-state pose model, an observability rank of 3 means that all three states are locally observable.

```math
\mathcal{O} = \begin{bmatrix} C \\ CF \\ CF^2 \end{bmatrix}
```

| Measurement | C | rank | Observable? |
|---|---|---|---|
| Encoders only | 0 (nothing measures x, y, θ) | 0 | no |
| Camera full pose | I₃ | 3 | yes |
| Camera range + bearing, robot moving | H along the path | [fill from your run] | |
| Camera range + bearing, turning on the spot | H along the path | [fill from your run] | |

Encoders measure wheel rotation rather than absolute position or heading. Without an external pose measurement, errors accumulate as odometry propagates the estimate. The terms F[0,2] and F[1,2] show how heading uncertainty becomes position uncertainty while the robot moves.

Range and bearing to one known point do not fully determine the robot’s world pose. Different positions around the marker, with corresponding headings, can produce the same measurements. This ambiguity remains during motion, giving an observability rank of 2. Using the marker’s orientation through a full-pose measurement, or observing another known landmark, can resolve the remaining ambiguity.

When the marker leaves the camera’s view, the EKF continues predicting the pose from wheel odometry without a camera correction. Uncertainty generally increases as motion errors accumulate. When a valid marker measurement becomes available again, the EKF resumes measurement updates to correct the estimate.
