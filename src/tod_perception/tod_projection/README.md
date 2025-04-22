# tod_projection

## Overview

This ROS2 package provides a node to project the vehicle's motion as lanes.

## Dependencies

> **Please note**  
> This Package was developed for ROS2 Humble.

Dependencies include:
  * C++17
  * tod package dependencies and external ROS2 Packages: see `package.xml`
  
## Nodes

- [LaneProjection](#laneprojection)

### LaneProjection
Publishes the set of vehicle lanes (front left/right, rear left/right) on every receive of a vehicle data message.
It is used to visualize the future path of the vehicle's edges if the vehicle was to stay on its current path given by the vehicle's current steering wheel angle.

**Subscribed Topics:**
- `input/primary_vehicle_state`
  ([tod_vehicle_msgs/msg/PrimaryVehicleState](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/VehicleData.msg))
  Contains current steering wheel angle of the vehicle that is used to calculate the lanes. 
- `input/secondary_vehicle_state`
  ([tod_vehicle_msgs/msg/SecondaryVehicleState](https://github.com/TUMFTM/tod_common/blob/master/tod_msgs/msg/VehicleData.msg))
  Contains the gear position to differentiate between the vehicle driving forwards / backwards. 

**Published Topics:**
- `output/vehicle_lane_*_**` ([nav_msgs/msg/Path](https://docs.ros.org/en/humble/p/nav_msgs/interfaces/msg/Path.html))
  Lane of the vehicle edges (\*: front/rear, \*\*: left/right).

**Parameters:**
- `vehicleID`: The ID of the vehicle is needed to load its parameters (e.g. geometric properties) for calculation of the lanes
- `config_path`: Path to folder containing config files. Default is tod_projection/config.


## Build the package
  * Build and source the workspace.
    ```bash
    colcon build --packages-select tod_projection && source install/setup.bash # or `setup.zsh`, depending on your shell
    ```

## Launch
  * Launch file to launch lane projection
    ```bash
    ros2 launch tod_projection tod_projection_launch.py
    ```
  * Vehicle lanes are published as soon as vehicle data is received. 
