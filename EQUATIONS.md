# Lane Keeping Assist: Equations and Parameters

This document describes the mathematical model implemented in `LKA_Model.slx` and the parameters used by the MATLAB scripts.

## 1. Kinematic Bicycle Model

The vehicle is represented by a kinematic bicycle model with three state variables:

| Symbol | Name | Unit | Meaning |
|---|---|---:|---|
| $e_y$ | Lateral position error | $\mathrm{m}$ | Lateral distance from the vehicle center of gravity to the road centerline |
| $e_\phi$ | Heading angle error | $\mathrm{rad}$ | Difference between the vehicle heading and the road tangent |
| $\delta$ | Front-wheel steering angle | $\mathrm{rad}$ | Actual steering angle produced by the EPS actuator |

The model assumes constant longitudinal speed, a rigid vehicle, no tire slip, and small steering and heading angles.

## 2. Lateral Position Dynamics

$$
\boxed{\dot{e}_y = v e_\phi + d_y}
$$

The lateral error increases when the vehicle heading differs from the road direction. $d_y$ is an external lateral-error-rate disturbance.

| Symbol | Meaning | Unit |
|---|---|---:|
| $v$ | Longitudinal vehicle speed | $\mathrm{m/s}$ |
| $e_\phi$ | Heading angle error | $\mathrm{rad}$ |
| $d_y$ | Lateral disturbance | $\mathrm{m/s}$ |

## 3. Heading Angle Dynamics

$$
\boxed{\dot{e}_\phi = \frac{v}{L}\delta - v\kappa_r + d_\phi}
$$

| Symbol | Meaning | Unit |
|---|---|---:|
| $v$ | Longitudinal vehicle speed | $\mathrm{m/s}$ |
| $L$ | Wheelbase | $\mathrm{m}$ |
| $\delta$ | Actual steering angle | $\mathrm{rad}$ |
| $\kappa_r$ | Reference road curvature | $\mathrm{m^{-1}}$ |
| $d_\phi$ | Heading-rate disturbance | $\mathrm{rad/s}$ |

The term $\frac{v}{L}\delta$ is the vehicle yaw rate for a bicycle model. The term $v\kappa_r$ represents the angular rate required to follow the reference road. Their difference changes the heading error.

## 4. Road Curvature

Curvature is the inverse of the road radius:

$$
\boxed{\kappa_r = \frac{1}{R}}
$$

- $\kappa_r = 0.005\,\mathrm{m^{-1}}$ for the constant-curvature road.
- $\kappa_r = 0$ for the straight road.
- A positive or negative curvature identifies the direction of curvature.

### Constant-curvature scenario

$$
\kappa_r(t) = 0.005
$$

### Winding-road scenario

$$
\kappa_r(t) = A_\kappa \sin\left(\frac{2\pi t}{T_\kappa}\right)
$$

The current tested and final project values in `RoadScenario.m` are:

| Parameter | Value | Meaning |
|---|---:|---|
| $A_\kappa$ | $0.085\,\mathrm{m^{-1}}$ | Maximum curvature; larger values produce tighter bends |
| $T_\kappa$ | $4\,\mathrm{s}$ | Curvature repetition period; smaller values create bends more frequently |

## 5. EPS Steering Actuator

The steering actuator is modeled as a first-order system:

$$
\boxed{\dot{\delta} = \frac{K_s u-\delta}{\tau_s}}
$$

| Symbol | Meaning | Unit | Project value |
|---|---|---:|---:|
| $u$ | Controller steering command | $\mathrm{rad}$ | State-dependent |
| $\delta$ | Actual steering angle | $\mathrm{rad}$ | State |
| $K_s$ | Actuator gain | dimensionless | $1$ |
| $\tau_s$ | Actuator time constant | $\mathrm{s}$ | $0.10$ |
| $\delta_{\max}$ | Maximum steering angle | $\mathrm{rad}$ | $0.5$ |

$\tau_s$ controls actuator delay: a smaller time constant makes the steering angle approach the command faster. The steering angle is saturated so that:

$$
\delta_{\mathrm{sat}} =
\begin{cases}
\delta_{\max}, & \delta>\delta_{\max},\\
\delta, & -\delta_{\max}\leq\delta\leq\delta_{\max},\\
-\delta_{\max}, & \delta<-\delta_{\max}.
\end{cases}
$$

## 6. LKA State-Feedback Controller

The controller uses proportional negative feedback on lateral and heading errors:

$$
\boxed{u=a_c\left(-K_y e_y-K_\phi e_\phi\right)}
$$

| Symbol | Meaning | Unit | Project value |
|---|---|---:|---:|
| $K_y$ | Lateral-error gain | dimensionless | $0.5$ |
| $K_\phi$ | Heading-error gain | dimensionless | $3$ |
| $e_y$ | Lateral position error | $\mathrm{m}$ | State |
| $e_\phi$ | Heading angle error | $\mathrm{rad}$ | State |
| $a_c$ | Controller enable signal | $0$ or $1$ | $0$ or $1$ |

- $a_c=1$: the controller is active.
- $a_c=0$: the controller command is zero.

The negative sign provides correction: for example, a positive lateral error produces a steering command in the opposite direction.

## 7. Complete Closed-Loop State Equations

The state vector is:

$$
\mathbf{x}=
\begin{bmatrix}
e_y\\
e_\phi\\
\delta
\end{bmatrix}
$$

Without the controller switch, the system equations are:

$$
\boxed{
\begin{aligned}
\dot{e}_y &= v e_\phi+d_y,\\
\dot{e}_\phi &= \frac{v}{L}\delta-v\kappa_r+d_\phi,\\
\dot{\delta} &= \frac{K_s u-\delta}{\tau_s}.
\end{aligned}
}
$$

After substituting the state-feedback controller:

$$
\boxed{
\begin{aligned}
\dot{e}_y &= v e_\phi+d_y,\\
\dot{e}_\phi &= \frac{v}{L}\delta-v\kappa_r+d_\phi,\\
\dot{\delta} &= \frac{-K_s a_c K_y e_y-K_s a_c K_\phi e_\phi-\delta}{\tau_s}.
\end{aligned}
}
$$

## 8. State-Space Form

The controller-off/no-disturbance linear part can be written as:

$$
\dot{\mathbf{x}}=A\mathbf{x}+B u+E\mathbf{w}
$$

with:

$$
\mathbf{x}=
\begin{bmatrix}
e_y\\
e_\phi\\
\delta
\end{bmatrix},
\qquad
\mathbf{w}=
\begin{bmatrix}
\kappa_r\\
d_y\\
d_\phi
\end{bmatrix}.
$$

Using the feedback $u=a_c(-K_y e_y-K_\phi e_\phi)$, the combined closed-loop matrix has the following structure:

$$
A_{\mathrm{closed}}=
\begin{bmatrix}
0 & v & 0\\
0 & 0 & \dfrac{v}{L}\\
-\dfrac{K_s a_c K_y}{\tau_s} &
-\dfrac{K_s a_c K_\phi}{\tau_s} &
-\dfrac{1}{\tau_s}
\end{bmatrix}.
$$

The road-curvature and disturbance inputs enter the second and other state equations as modeled above.

## 9. External Disturbances

The simulation generates both a lateral disturbance and a heading-rate disturbance as pulse signals.

$$
d_y(t)=
\begin{cases}
D_y, & t_s\leq t\leq t_e,\\
0, & \text{otherwise},
\end{cases}
$$

$$
d_\phi(t)=
\begin{cases}
D_\phi, & t_s\leq t\leq t_e,\\
0, & \text{otherwise}.
\end{cases}
$$

| Symbol | Meaning | Default value |
|---|---|---:|
| $t_s$ | Disturbance start | $5.0\,\mathrm{s}$ |
| $t_e$ | Disturbance end | $5.1\,\mathrm{s}$ |
| $D_y$ | Lateral disturbance magnitude | $1.5\,\mathrm{m/s}$ |
| $D_\phi$ | Heading disturbance magnitude | $2.5\,\mathrm{rad/s}$ |

Both signals are zero when $\mathrm{disturbanceEnabled}=0$.

## 10. Main Baseline Parameters

| Symbol | Description | Value | Unit |
|---|---|---:|---:|
| $v$ | Vehicle speed | $30$ | $\mathrm{m/s}$ |
| $L$ | Vehicle wheelbase | $2.7$ | $\mathrm{m}$ |
| $K_s$ | Steering actuator gain | $1$ | dimensionless |
| $\tau_s$ | Actuator time constant | $0.10$ | $\mathrm{s}$ |
| $K_y$ | Lateral error feedback gain | $0.5$ | dimensionless |
| $K_\phi$ | Heading error feedback gain | $3$ | dimensionless |
| $\delta_{\max}$ | Maximum steering angle | $\pm 0.5$ | $\mathrm{rad}$ |
| $W_{\mathrm{lane}}$ | Lane width | $3.6$ | $\mathrm{m}$ |
| $T_{\mathrm{sim}}$ | Simulation duration | $10$ | $\mathrm{s}$ |
| $\Delta t$ | Road-signal time step | $0.01$ | $\mathrm{s}$ |

The wheelbase $L$ is inferred from the project equations and is not directly declared in `RoadScenario.m`. Verify or adjust it in the model workspace when tuning the vehicle.

## 11. Initial Conditions

$$
\begin{aligned}
e_y(0)&=e_{y0},\\
e_\phi(0)&=e_{\phi0},\\
\delta(0)&=\delta_0.
\end{aligned}
$$

| Symbol | Meaning |
|---|---|
| $e_{y0}$ | Initial lateral position error |
| $e_{\phi0}$ | Initial heading angle error |
| $\delta_0$ | Initial steering angle |

These integrator initial conditions should be checked in the model workspace. They determine the initial alignment of the vehicle before correction begins.

## 12. Simulation Output Signals

The Simulink model exports:

| Output | MATLAB name | Unit |
|---|---|---:|
| Lateral position error | `out.ey_out` | $\mathrm{m}$ |
| Heading angle error | `out.ephi_out` | $\mathrm{rad}$ |
| Steering angle | `out.delta_out` | $\mathrm{rad}$ |
| Controller command | `out.u_out` | $\mathrm{rad}$ |

## 13. Performance Measures

Maximum absolute lateral error:

$$
\boxed{e_{y,\max}=\max_t |e_y(t)|}
$$

Mean absolute lateral error:

$$
\boxed{\mathrm{MAE}=\frac{1}{N}\sum_{i=1}^{N}|e_y(t_i)|}
$$

Final lateral error:

$$
\boxed{e_{y,\mathrm{final}}=e_y(T_{\mathrm{sim}})}
$$

These measures describe road-tracking accuracy and disturbance rejection.

## 14. Model Limitations

This is a control-design and simulation model, not a full vehicle-dynamics model. It does not include tire forces, tire slip, suspension dynamics, aerodynamic effects, load transfer, or nonlinear vehicle dynamics. The controller is a proportional state-feedback law, and the road is represented by a prescribed curvature signal.
