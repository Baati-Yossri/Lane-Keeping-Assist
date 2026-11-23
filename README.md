# Lane Keeping Assist (LKA) Simulation

MATLAB and Simulink project for simulating a Lane Keeping Assist system. The vehicle follows a simulated road while a controller regulates lateral position and heading errors. The project includes three road scenarios, independent controller and disturbance controls, and a visualization of the vehicle and road.

## Features

- Constant-curvature road
- Winding, S-shaped road
- Straight road
- Controller enabled or disabled
- Configurable lateral and heading disturbances
- Animated vehicle trajectory
- Disturbance and recovery highlighting with slow-motion playback
- Live animation panel showing simulation status and vehicle errors
- Performance plots for lateral error, heading error, steering angle, and controller command

For the full bicycle-model equations, controller derivation, disturbance model, state-space form, and parameter descriptions, see [EQUATIONS.md](EQUATIONS.md).

## Requirements

- MATLAB
- Simulink
- A MATLAB installation that supports the blocks used in `LKA_Model.slx`

No external MATLAB toolbox is explicitly required by the provided scripts.

## Project Structure

| File | Description |
|---|---|
| `run_LKA.m` | Main entry point. Configures and runs the simulation, then starts the animation. |
| `RoadScenario.m` | Generates road curvature and lateral disturbance signals. |
| `LKA_Model.slx` | Main Simulink vehicle and controller model. |
| `LKA_Animation_AllRoads.m` | Reconstructs the road and animates the simulated vehicle for all scenarios. |
| `LKA_Animation.m` | Animation script for the curved-road visualization. |
| `figure_export.m` | Creates performance plots from the simulation output. |
| `testing.slx` | Auxiliary testing model. |

## Quick Start

1. Open MATLAB.
2. Set the MATLAB current folder to the project directory.

   ```matlab
   cd("C:\Users\GIGABYTE\Desktop\Perso\ENISO\Semester 1\AUTOMOTIVE ADVANCED SIMULATION\PROJECT - Lane Keeping Assist")
   ```

3. Run a simulation with the controller and disturbance enabled:

   ```matlab
   run_LKA(1, 1, 1)
   ```

`run_LKA` accepts three arguments:

1. `scenario`: road selection (`1`, `2`, or `3`)
2. `controllerState`: controller state (`0` = OFF, `1` = ON)
3. `disturbanceState`: disturbance state (`0` = OFF, `1` = ON)

The controller and disturbance default to **ON** when their arguments are omitted.

## Road Scenarios

| Scenario | Description | Command |
|---|---|---|
| `1` | Constant-curvature road | `run_LKA(1, 1, 1)` |
| `2` | Winding S-shaped road | `run_LKA(2, 1, 1)` |
| `3` | Straight road | `run_LKA(3, 1, 1)` |

For controller-off comparison runs, use `0` as the second argument:

```matlab
run_LKA(2, 0, 1)
```

To disable the disturbance, use `0` as the third argument:

```matlab
run_LKA(2, 1, 0)
```

Calling `run_LKA` without arguments is not valid because the road scenario must be provided. For example, `run_LKA(2)` runs scenario 2 with both the controller and disturbance enabled by default.

## Simulation Settings

The default settings are defined in `RoadScenario.m`:

- Simulation duration: `10 s`
- Time step: `0.01 s`
- Vehicle speed: `30 m/s`
- Lane width: `3.6 m`
- Disturbance interval: `5.0 s` to `5.1 s`
- Lateral disturbance magnitude: `1.5 m/s`
- Heading disturbance magnitude: `2.5 rad/s`
- Animation recovery/highlight period: 2 seconds after the disturbance ends

The winding-road curvature is configurable through:

```matlab
curvatureAmplitude = 0.085;
curvaturePeriod = 4;
```

The vehicle speed can be changed through the `v` variable in `RoadScenario.m`. The same variable is used when reconstructing the animation.

The disturbance is generated only when `disturbanceEnabled` is `1`. The disturbance signals are exported as `dySignal` and `dphiSignal` for Simulink.

## Simulation Outputs

After running `run_LKA`, the simulation output is stored in the MATLAB base workspace as:

```matlab
out
```

The animation script expects the following signals:

- `out.ey_out` - lateral position error
- `out.ephi_out` - heading angle error
- `out.delta_out` - steering angle
- `out.u_out` - controller command

When disturbance mode is enabled, the animation also displays the disturbance value, highlights the vehicle and trajectory during the disturbance and recovery phases, and slows playback from `0.03 s` to `0.15 s` per animation step.

To generate the performance dashboard after a simulation:

```matlab
figure_export
```

## Running the Model Manually

The model can also be run directly in MATLAB:

```matlab
roadScenario = 2;
controllerEnabled = 1;
disturbanceEnabled = 1;
RoadScenario
load_system("LKA_Model")
out = sim("LKA_Model");
LKA_Animation_AllRoads
```

Alternatively, open `LKA_Model.slx` in Simulink and select **Run**. Run `RoadScenario.m` first so that `kappaSignal`, `dySignal`, `dphiSignal`, and the simulation variables exist in the MATLAB workspace. After a manual simulation, assign the simulation output to `out` before launching the animation.

## Adjusting the Model

To test controller or vehicle variations:

1. Open `LKA_Model.slx` in Simulink.
2. Modify the controller, vehicle, or disturbance blocks.
3. Update the parameters in `RoadScenario.m` if required.
4. Run the project through `run_LKA`.
5. Compare the plots and animation for controller-on and controller-off cases.

The Simulink cache and generated files in `slprj/`, `*.slxc`, and `tmwinternal/` are build artifacts. The normal source files needed to run the project are the `.m` files and `LKA_Model.slx`.
