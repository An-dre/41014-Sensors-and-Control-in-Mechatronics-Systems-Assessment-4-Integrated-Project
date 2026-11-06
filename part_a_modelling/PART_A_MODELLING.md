# Part A: Modelling (Gabriel)

The state-space model of the TurtleBot3 Waffle Pi used by every other part: the unicycle
model and its Jacobians for the EKF (Part C), the linear tracking-error model for the LQR
(Part D), the measurement model for the camera, and the controllability and observability
analysis. Checked against the MuJoCo robot at the end.

| file in `part_a_modelling/` | what it is |
|---|---|
| `model.py` | wheel ↔ body conversion, motion model `f`, Jacobians `F` and `G`, error model `A`, `B`, controllability and observability matrices |
| `validatemodel.py` | drives the MuJoCo robot and compares the model's prediction with what really happened |
| `results_part_a/validation.md` | the table `validatemodel.py` writes (section "Model check against MuJoCo") |

## Run and reproduce

From the repository root:

```
python -m part_a_modelling.model           self-checks, controllability and observability ranks
python -m part_a_modelling.validatemodel   model check against MuJoCo, writes results_part_a/validation.md
```

### Robot definition

Parameters, checked against `robotis_tb3/turtlebot3_waffle_pi.xml`:

- Wheel radius r = 0.033 m
- Wheel separation L = 0.288 m (wheels at y = ±0.144 m)
- Commanded wheel speed limits: −7.88 to 7.88 rad/s
- The model is updated every 0.02 s (the control and EKF rate)

### Assumptions

- **No slip:** the robot is slow and the contact friction is high enough that slip is negligible. The model check below shows this holds once the floor contact is fixed (it does not hold for the robot exactly as the xml loads).
- **Flat floor:** the simulated floor is flat, with no uneven surfaces.
- **Kinematic model:** a dynamic model would include motor torque, wheel inertia and friction. The kinematic model leaves these out and assumes v and ω act straight away, so it only tracks position and heading. The TurtleBot is small and slow, and its motors have their own speed controllers, so the commanded wheel speed is reached within about one dt.

### States and inputs

```math
\mathbf{x} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}
```

The state holds where the robot is (x, y) and which way it faces (θ), in metres and radians. All three are in the world frame, the fixed room frame of `sceneforturtle.xml`: origin at the centre of the 6 × 6 m room, x pointing towards the dock. It is the same frame whichever corner the robot starts from.

Inputs:

```math
\mathbf{u} = \begin{bmatrix} v \\ \omega \end{bmatrix}
```

The two inputs are the forward speed v (m/s) and the turn rate ω (rad/s). Positive ω is anticlockwise.

### Wheels to body

Forward speed:

```math
v = \frac{r(\omega_R + \omega_L)}{2}
```

Turn rate:

```math
\omega = \frac{r(\omega_R - \omega_L)}{L}
```

The controller and the EKF work in v and ω, but the MuJoCo robot only accepts left and right wheel speeds. v is the average of the two wheel rim speeds, and ω is their difference divided by the gap between the wheels. Part B uses these equations to turn encoder readings into odometry (`wheels_to_body()`).

### Body to wheels

Left wheel:

```math
\omega_L = \frac{v - \omega L/2}{r}
```

Right wheel:

```math
\omega_R = \frac{v + \omega L/2}{r}
```

Solving the two equations above for the wheel speeds gives `body_to_wheels()`. It is used at every control step to send wheel commands to the robot.

### Continuous-time model

```math
\dot{x} = v\cos\theta
```

```math
\dot{y} = v\sin\theta
```

```math
\dot{\theta} = \omega
```

This is the standard unicycle model: the robot can move along its heading and spin, but cannot move sideways. It is nonlinear because v multiplies cos θ and sin θ, an input times a function of a state. The EKF and the LQR need linear models, so it is linearised twice, once for each.

### Linearisation 1: Jacobians F and G (EKF)

Near the current estimate, the curved function f is replaced by its slope. The slope of a function of several variables is the matrix of partial derivatives, which gives the Jacobians F and G. Discretising with Euler steps:

```math
x_{k+1} = x_k + v_k\Delta t\cos\theta_k
```

```math
y_{k+1} = y_k + v_k\Delta t\sin\theta_k
```

```math
\theta_{k+1} = \theta_k + \omega_k\Delta t
```

Differentiating each line with respect to each state gives F:

```math
F = \frac{\partial f}{\partial \mathbf{x}} = \begin{bmatrix} 1 & 0 & -v_k \Delta t \sin\theta_k \\ 0 & 1 & v_k \Delta t \cos\theta_k \\ 0 & 0 & 1 \end{bmatrix}
```

F describes how small errors in the robot's current position and heading affect its next predicted pose. F[0,2] and F[1,2] tell the EKF how far off x and y end up if θ is slightly wrong, scaled by the distance moved that step (v Δt). This is `jacobian_F()` in `model.py`.

Differentiating each line with respect to each input gives G:

```math
G = \frac{\partial f}{\partial \mathbf{u}} = \begin{bmatrix} \Delta t \cos\theta_k & 0 \\ \Delta t \sin\theta_k & 0 \\ 0 & \Delta t \end{bmatrix}
```

G maps small changes, or uncertainty, in the speed and turn rate into each pose step. Each column is an input and each row a state.

- **Column 0 (v):** more speed moves the robot along its heading, through cos θ and sin θ.
- **Column 1 (ω):** more turn rate only changes θ, by Δt per unit of ω.

F and G are recomputed at every step from the current estimate and input, because they depend on θ and v. F pushes the EKF's covariance forward in the predict step. G is there for input noise: Part C's EKF uses a constant process noise Q instead (see `PART_C_ESTIMATION.md`).

### Linearisation 2: error model A and B (LQR)

The LQR cannot use F and G directly, because both depend on θ, which keeps changing. Instead the error to the reference path is modelled in the robot's own frame, where it stays nearly linear.

The tracking error, rotated into the robot frame, and the input correction:

```math
\mathbf{e} = \begin{bmatrix} e_x \\ e_y \\ e_\theta \end{bmatrix}, \qquad \delta\mathbf{u} = \begin{bmatrix} v - v_{ref} \\ \omega - \omega_{ref} \end{bmatrix}
```

e_x is how far the reference is ahead of the robot, e_y how far it is to the left, and e_θ the heading error. δu is the change to the forward speed and turn rate relative to their reference values.

Differentiating the tracking errors gives their dynamics. Small-angle approximations are applied and products of two small terms are dropped. Euler discretisation with the update interval Δt gives the discrete A and B matrices:

```math
\mathbf{e}_{k+1} = A\,\mathbf{e}_k + B\,\delta\mathbf{u}_k
```

```math
A = \begin{bmatrix} 1 & \omega_{ref}\Delta t & 0 \\ -\omega_{ref}\Delta t & 1 & v_{ref}\Delta t \\ 0 & 0 & 1 \end{bmatrix}
```

```math
B = \begin{bmatrix} -\Delta t & 0 \\ 0 & 0 \\ 0 & -\Delta t \end{bmatrix}
```

If the reference is ahead, e_x is positive, so the robot must speed up (δv > 0) to reduce it. For a positive δv to make e_x smaller, B[0,0] has to be negative, which gives B[0,0] = −Δt.

No input moves the robot sideways, which is why the middle row of B is zero. To reduce e_y the robot must first turn (e_θ) and then drive forward: that link is the A[1,2] = v_ref Δt term. If v_ref = 0 that entry is 0 and e_y cannot be corrected, which is why the system is not fully controllable when stopped.

This is `linear_error_model()` in `model.py`, which returns A and B. Part D's controller uses it directly.

### Measurement model

The camera detects an ArUco marker at the known world position (m_x, m_y). The measurement model predicts the planar range ρ and relative bearing β to the marker from the robot's position and heading. Both are measured from the robot's base origin (Part B converts the camera's reading to that point).

```math
z = h(\mathbf{x}) = \begin{bmatrix} \rho \\ \beta \end{bmatrix} = \begin{bmatrix} \sqrt{(m_x - x)^2 + (m_y - y)^2} \\ \mathrm{atan2}(m_y - y,\; m_x - x) - \theta \end{bmatrix}
```

The EKF needs the Jacobian H of h with respect to the state. It describes how small changes in the estimated position and heading change the predicted range and bearing.

```math
H = \frac{\partial h}{\partial \mathbf{x}} = \begin{bmatrix} -\dfrac{m_x - x}{\rho} & -\dfrac{m_y - y}{\rho} & 0 \\ \dfrac{m_y - y}{\rho^2} & -\dfrac{m_x - x}{\rho^2} & -1 \end{bmatrix}
```

The first row describes changes in range: moving towards the marker decreases it, moving away increases it. The second row describes changes in bearing caused by sideways movement relative to the marker or by rotation. Its last entry is −1 because turning the robot anticlockwise decreases the marker's relative bearing by the same angle.

The prediction is `h()` and its Jacobian is `jacobian_H()`, both in Part C's `ekf.py`, where the measurement update uses them.

### Controllability

Controllability asks whether the inputs can move the tracking error from any initial value to any desired value, including zero. For this three-state model the controllability matrix must have rank 3.

```math
\mathcal{C} = \begin{bmatrix} B & AB & A^2B \end{bmatrix}
```

| Case | rank | Controllable? |
|---|---|---|
| v_ref = 0.2 m/s | 3 | yes |
| v_ref = 0 | 2 | no |

B never acts on e_y directly, because its middle row is zero. When the reference speed is zero, the A[1,2] term that couples heading error into lateral error vanishes as well, so e_y stays where it is in the stationary linearised model. Only two independent error directions can be controlled, giving rank 2.

For docking, this means an LQR designed around a stationary reference cannot correct a remaining sideways offset. The robot should correct its lateral alignment during the approach and stop only once it reaches the docking pose, which is what Part D's controller does (it docks while still moving at 0.05 m/s). It does not mean the robot must move continuously: if it stops with a sideways offset, it can still turn and drive again.

### Observability

Observability asks whether the states can be worked out from the measurements and known inputs over time. For the three-state pose model, rank 3 means all three states are locally observable.

```math
\mathcal{O} = \begin{bmatrix} C \\ CF \\ CF^2 \end{bmatrix}
```

Along a moving path F changes from step to step, so the self-check stacks H₀, H₁F₀ and H₂F₁F₀ over three steps.

| Measurement | C | rank | Observable? |
|---|---|---|---|
| Encoders only | 0 (nothing measures x, y, θ) | 0 | no |
| Camera full pose | I₃ | 3 | yes |
| Camera range + bearing, parked | H | 2 | no: one direction unobservable |
| Camera range + bearing, robot moving | H along the path | 2 | no: one direction unobservable |
| Camera range + bearing, turning on the spot | H along the path | 2 | no: one direction unobservable |

Encoders measure wheel rotation, not absolute position or heading. Without an external pose measurement, errors build up as odometry propagates the estimate. F[0,2] and F[1,2] show how heading uncertainty turns into position uncertainty while the robot moves.

Range and bearing to one known point do not fully fix the robot's world pose. Moving the robot around a circle centred on the marker, while turning it by the same angle, gives exactly the same range and bearing. That ambiguity stays during motion, so the rank stays 2. Using the marker's orientation (a full-pose measurement), or seeing a second known landmark, would resolve it. In practice the EKF still works, because it starts from a known placement and odometry ties consecutive poses together, so it only needs local corrections.

When the marker leaves the camera's view, the EKF keeps predicting the pose from wheel odometry with no camera correction, and its uncertainty grows as motion errors build up. When a valid marker measurement comes back, the EKF resumes measurement updates and corrects the estimate. Part E's robustness study measures what that costs when the camera is lost for 10 s up to the rest of the run.

### Model check against MuJoCo

`validatemodel.py` drives the MuJoCo robot with fixed wheel speeds for three manoeuvres and integrates the model (wheels_to_body, then f) from the wheel encoders every 2 ms physics step. At the end it compares the model's pose with the simulator's true pose, which is read only to score the model.

| Robot model | Manoeuvre | Model: distance, turn | True: distance, turn | Position error | Heading error |
|---|---|---|---|---|---|
| exactly as the xml loads | straight, 7.5 s | 0.980 m, 0.0° | 0.829 m, −0.7° | 15.0 cm | +0.7° |
| exactly as the xml loads | spin on the spot, 9 s | 0.000 m, 348.9° | 0.052 m, 242.9° | 5.2 cm | +106.0° |
| exactly as the xml loads | arc (left 2, right 4 rad/s), 10 s | 0.779 m, 130.6° | 0.747 m, 72.0° | 38.1 cm | +58.5° |
| with the contact correction | straight, 7.5 s | 0.985 m, 0.0° | 0.981 m, 0.0° | 0.4 cm | 0.0° |
| with the contact correction | spin on the spot, 9 s | 0.000 m, 351.9° | 0.000 m, 350.8° | 0.0 cm | +1.0° |
| with the contact correction | arc (left 2, right 4 rad/s), 10 s | 0.782 m, 131.1° | 0.782 m, 130.1° | 0.8 cm | +0.9° |

With the robot exactly as the xml loads, the no-slip assumption fails: it travels 15 % less than its wheels say and turns about 30 % less, so the model is off by up to 38 cm and 106°. Part D traced this to the chassis collision shape touching the floor and to the caster friction (`PART_D_CONTROL.md`, section 3.1). The correction is applied when the scene loads (`sim.load()`), and every part uses it. With it, the model matches MuJoCo to within 1 cm and 1° over about 1 m of travel or a full turn. The model is fed the exact encoder angles here, so what is left (under 1 % of the distance or the turn) is a little creep in MuJoCo's soft wheel contacts. The EKF's process noise Q covers it.
