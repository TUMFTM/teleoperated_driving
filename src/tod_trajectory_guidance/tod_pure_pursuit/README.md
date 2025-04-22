# TOD Pure Pursuit {#tod_pure_pursuit_doc}
====

## Overview

The TOD Pure Pursuit Package implements an adaptive pure pursuit controller where the look-ahead distance is proportional to the vehicle's velocity. The package handles trajectory following and simulates trajectories using a kinematic bicycle model for validation purposes. It includes coordinate system transformations and trajectory validation checks.

For theoretical details on the pure pursuit algorithm, see the reference paper by Coulter 1992 ["Implementation of the pure pursuit path tracking algorithm"](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf).

Content:
- **TOD Pure Pursuit** \ref tod_pure_pursuit
  - **Core Components**
    - pure_pursuit.hpp/cpp - Core controller implementation
    - path_tracking_control.hpp/cpp - Management node for controller
    - path_simulator.hpp/cpp - Simulation node of expected controller behavior for validation process
    - vehicle_model.hpp / helpers.hpp - Kinematic bicycle model for simulation

## Executables
- **Vehicle**:
  - **path_tracking_control**
  - **path_simulator**

## Subscribed Topics
- **PathTrackingControl**:
  - `input/odometry` - Vehicle odometry data
  - `input/trajectory` - Target trajectory to follow
  - `input/tod_status` - Vehicle status information
- **PathSimulator**
  - `input/validation_trajectory` - Trajectory for validation simulation

## Published Topics
- **PathTrackingControl**:
  - `output/primary_control_cmd` - Vehicle control commands
  - `output/ptc_logging` - Controller logging data
  - `output/path_array_for_rviz_from_controller` - Path visualization data
- **PathSimulator**
  - `output/simulated_trajectory` - Simulated trajectory results

## Parameters
- **PathTrackingControl**
  - `rear_axle_frame_id` (string, default: "base_link") - Frame ID for rear axle
  - `check_trajectory_outdated` (bool, default: false) - Enable trajectory age checking
  - `lookahead_ratio` (double, default: 0.3) - Ratio for adaptive lookahead distance
  - `min_lookahead_distance` (double, default: 2.0) - Minimum lookahead distance
  - `debug` (bool, default: false) - Enable debug output
  - `control_mode` (int) - Control mode setting
  - `config_path` (string) - Path to vehicle configuration
  - `simulation_step` (double, default: 0.1) - Time step for trajectory simulation


# Usage
The Pure Pursuit controller can be used for trajectory following within the teleoperation stack. It provides both real-time control and trajectory simulation capabilities.

## Build
```bash
colcon build --packages-up-to tod_pure_pursuit
```

## Launch
```bash
ros2 launch tod_pure_pursuit tod_pure_pursuit.launch.py
```

## Arguments
- `vehicle_config_path` (string) - Path to vehicle configuration file
- `vehicleID` (string, default: "edgar") - Vehicle identifier

## Doxygen stuff
\version 1.0
\author David Brecht, Hoffmann, Krauss
\defgroup tod_pure_pursuit TOD Pure Pursuit
\brief Adaptive Pure Pursuit controller implementation for trajectory following with simulation capabilities
