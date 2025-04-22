# TOD Vehicle Simulation {#tod_vehicle_sim_doc}
====

## Overview

The TOD Vehicle Simulation Package implements a simple kinematic bicycle model to simulate vehicle behavior based on received control commands. It provides an interface for testing and debugging vehicle dynamics, employing a PI controller to mimic EDGAR-like velocity control. The simulation node receives primary and secondary control commands, processes vehicle status messages, and publishes odometry and vehicle state information. This package is primarily used alongside visualization tools (e.g., tod_visual) for comprehensive vehicle behavior analysis.

Content:
- **TOD Vehicle Simulation** \ref tod_vehicle_sim
  - **Core Components**
    - **vehicle_sim_node.cpp** - Node implementing vehicle simulation using a kinematic bicycle model.
    - **vehicle_model.cpp** - Encapsulates the vehicle's dynamics and motion update functions.

## Executable
- **Vehicle Simulation Node**:
  - **VehicleSimNode** - The main node simulating vehicle motion and interfacing with control and status messages.

## Subscribed Topics
- **VehicleSimNode**:
  - `actuation/to_actuation/primary_control_cmd` - Receives primary control commands (steering and desired velocity).
  - `actuation/to_actuation/secondary_control_cmd` - Receives secondary control commands (e.g., honk, wiper, gear, indicator signals).
  - `input/tod_status` - Receives vehicle status messages to monitor connection and state changes.

## Published Topics
- **VehicleSimNode**:
  - `sensing/from_sensing/odometry` - Publishes odometry data based on the simulated vehicle pose and twist.
  - `actuation/from_actuation/primary_vehicle_state` - Publishes primary vehicle state data (velocity, acceleration, steering wheel angle).
  - `actuation/from_actuation/secondary_vehicle_state` - Publishes secondary vehicle state data (gear position, honk, wiper, head light, indicator, flash light).

## Parameters
- **VehicleSimNode**
  - `config_path` (string, default: "") - Path to the vehicle configuration directory.  
    *Used to load vehicle-specific parameters from the vehicle_config folder.*
  - `vehicleID` (string, default: "edgar") - Identifier for the vehicle instance.  
    *This parameter can be used to switch or identify different vehicle configurations.*

## Usage
The Vehicle Simulation controller is designed to mimic realistic vehicle motion using a simplified kinematic model and PI control for velocity adjustments. It is well-suited for testing control algorithms and validating behavior in simulation environments before deploying on physical vehicles.

## Build
```bash
colcon build --packages-up-to tod_vehicle_sim
```

## Launch
```bash
ros2 launch tod_vehicle_sim vehicle_sim.launch.py
```

## Arguments
- `config_path` (string) - Absolute path to the directory containing the vehicle configuration files.
- `vehicleID` (string, default: "edgar") - Identifier used to distinguish between multiple vehicle setups.

## Doxygen Documentation
\version 1.0  
\author TUMFTM  
\defgroup tod_vehicle_sim TOD Vehicle Simulation  
\brief Implements a kinematic bicycle model for vehicle simulation and debugging with control and status message integration.