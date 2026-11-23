# Lane Keeping Assist: Equations and Parameters

This document describes the mathematical model implemented in `LKA_Model.slx` and the parameters used by the MATLAB scripts.

## 1. Kinematic Bicycle Model

The vehicle is represented by a kinematic bicycle model with three state variables:

| Symbol | Name | Unit | Meaning |
|---|---|---:|---|
| `e_y` | Lateral position error | m | Lateral distance from the vehicle center of gravity to the road centerline |
| `e_phi` | Heading angle error | rad | Difference between the vehicle heading and the road tangent |
| `delta` | Front-wheel steering angle | rad | Actual steering angle produced by the EPS actuator |

The model assumes constant longitudinal speed, a rigid vehicle, no tire slip, and small steering and heading angles.

## 2. Lateral Position Dynamics

```text
dot(e_y) = v * e_phi + d_y
```

The lateral error increases when the vehicle heading differs from the road direction. `d_y` is an external lateral-error-rate disturbance.

| Symbol | Meaning | Unit |
|---|---|---:|
| `v` | Longitudinal vehicle speed | m/s |
| `e_phi` | Heading angle error | rad |
| `d_y` | Lateral disturbance | m/s |

## 3. Heading Angle Dynamics

```text
dot(e_phi) = (v / L) * delta - v * kappa_r + d_phi
```

| Symbol | Meaning | Unit |
|---|---|---:|
| `v` | Longitudinal vehicle speed | m/s |
| `L` | Wheelbase | m |
| `delta` | Actual steering angle | rad |
| `kappa_r` | Reference road curvature | 1/m |
| `d_phi` | Heading-rate disturbance | rad/s |

The term `v/L * delta` is the vehicle yaw rate for a bicycle model. The term `v*kappa_r` represents the angular rate required to follow the reference road. Their difference changes the heading error.

## 4. Road Curvature

Curvature is the inverse of the road radius:

```text
kappa_r = 1 / R
```

- `kappa_r = 0.005 1/m` for the constant-curvature road.
- `kappa_r = 0` for the straight road.
- A positive or negative curvature identifies the direction of curvature.

### Constant-curvature scenario

```text
kappa_r(t) = 0.005
```

### Winding-road scenario

```text
kappa_r(t) = curvatureAmplitude * sin(2*pi*t / curvaturePeriod)
```

The current tested and final project values in `RoadScenario.m` are:

| Parameter | Value | Meaning |
|---|---:|---|
| `curvatureAmplitude` | `0.085 1/m` | Maximum curvature; larger values produce tighter bends |
| `curvaturePeriod` | `4 s` | Curvature repetition period; smaller values create bends more frequently |

## 5. EPS Steering Actuator

The steering actuator is modeled as a first-order system:

```text
dot(delta) = (Ks * u - delta) / tau_s
```

| Symbol | Meaning | Unit | Project value |
|---|---|---:|---:|
| `u` | Controller steering command | rad | State-dependent |
| `delta` | Actual steering angle | rad | State |
| `Ks` | Actuator gain | dimensionless | `1` |
| `tau_s` | Actuator time constant | s | `0.10` |
| `delta_max` | Maximum steering angle | rad | `0.5` |

`tau_s` controls actuator delay: a smaller time constant makes the steering angle approach the command faster. The steering angle is saturated so that:

```text
delta_sat = clamp(delta, -delta_max, delta_max)
```

## 6. LKA State-Feedback Controller

The controller uses proportional negative feedback on lateral and heading errors:

```text
u = ac * (-Ky * e_y - Kphi * e_phi)
```

| Symbol | Meaning | Unit | Project value |
|---|---|---:|---:|
| `Ky` | Lateral-error gain | dimensionless | `0.5` |
| `Kphi` | Heading-error gain | dimensionless | `3` |
| `e_y` | Lateral position error | m | State |
| `e_phi` | Heading angle error | rad | State |
| `ac` | Controller enable signal | `0` or `1` | `0` or `1` |

- `ac = 1`: the controller is active.
- `ac = 0`: the controller command is zero.

The negative sign provides correction: for example, a positive lateral error produces a steering command in the opposite direction.

## 7. Complete Closed-Loop State Equations

The state vector is:

```text
x = [e_y; e_phi; delta]
```

Without the controller switch, the system equations are:

```text
dot(e_y)   = v * e_phi + d_y
dot(e_phi) = (v / L) * delta - v * kappa_r + d_phi
dot(delta) = (Ks * u - delta) / tau_s
```

After substituting the state-feedback controller:

```text
dot(e_y)   = v * e_phi + d_y
dot(e_phi) = (v / L) * delta - v * kappa_r + d_phi
dot(delta) = (-Ks * ac * Ky * e_y
              -Ks * ac * Kphi * e_phi
              -delta) / tau_s
```

## 8. State-Space Form

The controller-off/no-disturbance linear part can be written as:

```text
dot(x) = A*x + B*u + E*w
```

with:

```text
x = [e_y; e_phi; delta]

w = [kappa_r; d_y; d_phi]
```

Using the feedback `u = ac*(-Ky*e_y-Kphi*e_phi)`, the combined closed-loop matrix has the following structure:

```text
A_closed =
[ 0                         v            0
  0                         0            v/L
  -Ks*ac*Ky/tau_s          -Ks*ac*Kphi/tau_s  -1/tau_s ]
```

The road-curvature and disturbance inputs enter the second and other state equations as modeled above.

## 9. External Disturbances

The simulation generates both a lateral disturbance and a heading-rate disturbance as pulse signals.

```text
d_y(t) = Dy,     if ts <= t <= te
d_y(t) = 0,      otherwise

d_phi(t) = Dphi, if ts <= t <= te
d_phi(t) = 0,    otherwise
```

| Symbol | Meaning | Default value |
|---|---|---:|
| `ts` | Disturbance start | `5.0 s` |
| `te` | Disturbance end | `5.1 s` |
| `Dy` | Lateral disturbance magnitude | `1.5 m/s` |
| `Dphi` | Heading disturbance magnitude | `2.5 rad/s` |

Both signals are zero when `disturbanceEnabled = 0`.

## 10. Main Baseline Parameters

| Symbol | Description | Value | Unit |
|---|---|---:|---:|
| `v` | Vehicle speed | `30` | m/s |
| `L` | Vehicle wheelbase | `2.7` | m |
| `Ks` | Steering actuator gain | `1` | dimensionless |
| `tau_s` | Actuator time constant | `0.10` | s |
| `Ky` | Lateral error feedback gain | `0.5` | dimensionless |
| `Kphi` | Heading error feedback gain | `3` | dimensionless |
| `delta_max` | Maximum steering angle | `±0.5` | rad |
| `laneWidth` | Lane width | `3.6` | m |
| `StopTime` | Simulation duration | `10` | s |
| `dt` | Road-signal time step | `0.01` | s |

The wheelbase `L` is inferred from the project equations and is not directly declared in `RoadScenario.m`. Verify or adjust it in the model workspace when tuning the vehicle.

## 11. Initial Conditions

```text
e_y(0) = ey0
e_phi(0) = ephi0
delta(0) = delta0
```

| Symbol | Meaning |
|---|---|
| `ey0` | Initial lateral position error |
| `ephi0` | Initial heading angle error |
| `delta0` | Initial steering angle |

These integrator initial conditions should be checked in the model workspace. They determine the initial alignment of the vehicle before correction begins.

## 12. Simulation Output Signals

The Simulink model exports:

| Output | MATLAB name | Unit |
|---|---|---:|
| Lateral position error | `out.ey_out` | m |
| Heading angle error | `out.ephi_out` | rad |
| Steering angle | `out.delta_out` | rad |
| Controller command | `out.u_out` | rad |

## 13. Performance Measures

Maximum absolute lateral error:

```text
e_y,max = max(|e_y(t)|)
```

Mean absolute lateral error:

```text
MAE = (1/N) * sum(|e_y(t_i)|)
```

Final lateral error:

```text
e_y,final = e_y(T_sim)
```

These measures describe road-tracking accuracy and disturbance rejection.

## 14. Model Limitations

This is a control-design and simulation model, not a full vehicle-dynamics model. It does not include tire forces, tire slip, suspension dynamics, aerodynamic effects, load transfer, or nonlinear vehicle dynamics. The controller is a proportional state-feedback law, and the road is represented by a prescribed curvature signal.
