# Lane Keeping Assist (LKA) Simulation

MATLAB and Simulink project for simulating a Lane Keeping Assist system. The vehicle follows a simulated road while a controller regulates lateral position and heading errors. The project includes three road scenarios, controller on/off modes, and a visualization of the vehicle and road.

## Features

- Constant-curvature road
- Winding, S-shaped road
- Straight road
- Controller enabled or disabled
- Lateral disturbance applied during the simulation
- Animated vehicle trajectory
- Performance plots for lateral error, heading error, steering angle, and controller command

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

3. Run a simulation with the controller enabled:

   ```matlab
   run_LKA(1, 1)
   ```

The first argument is the road scenario and the second argument enables or disables the controller.

## Road Scenarios

| Scenario | Description | Command |
|---|---|---|
| `1` | Constant-curvature road | `run_LKA(1, 1)` |
| `2` | Winding S-shaped road | `run_LKA(2, 1)` |
| `3` | Straight road | `run_LKA(3, 1)` |

For controller-off comparison runs, use `0` as the second argument:

```matlab
run_LKA(2, 0)
```

Calling `run_LKA` without arguments is not valid. The road scenario must be provided:

```matlab
run_LKA(2)
```

This runs scenario 2 with the controller enabled by default.

## Simulation Settings

The default settings are defined in `RoadScenario.m`:

- Simulation duration: `10 s`
- Time step: `0.01 s`
- Vehicle speed: `30 m/s`
- Lane width: `3.6 m`
- Lateral disturbance: `0.5 m/s` from `2.0 s` to `2.5 s`

The winding-road curvature is configurable through:

```matlab
curvatureAmplitude = 0.085;
curvaturePeriod = 4;
```

The vehicle speed can be changed through the `v` variable in `RoadScenario.m`. The same variable is used when reconstructing the animation.

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

To generate the performance dashboard after a simulation:

```matlab
figure_export
```

## Running the Model Manually

The model can also be run directly in MATLAB:

```matlab
roadScenario = 2;
controllerEnabled = 1;
RoadScenario
load_system("LKA_Model")
sim("LKA_Model")
LKA_Animation_AllRoads
```

Alternatively, open `LKA_Model.slx` in Simulink and select **Run**. Run `RoadScenario.m` first so that `kappaSignal`, `dySignal`, and the simulation variables exist in the MATLAB workspace.

## Adjusting the Model

To test controller or vehicle variations:

1. Open `LKA_Model.slx` in Simulink.
2. Modify the controller, vehicle, or disturbance blocks.
3. Update the parameters in `RoadScenario.m` if required.
4. Run the project through `run_LKA`.
5. Compare the plots and animation for controller-on and controller-off cases.

The Simulink cache and generated files in `slprj/`, `*.slxc`, and `tmwinternal/` are build artifacts. The normal source files needed to run the project are the `.m` files and `LKA_Model.slx`.
