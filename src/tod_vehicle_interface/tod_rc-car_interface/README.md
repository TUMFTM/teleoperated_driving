# tod_rc-car_interface

## Overview

The `tod_rc-car_interface` provides a specialized ROS2 interface for teleoperation and control of an RC car platform, primarily designed for integration and real-world testing of the TOD teleoperation software stack. The interface uses a modified F1Tenth vehicle equipped with a VESC motor controller for steering and velocity control. The software is designed to run dockerized on an NVIDIA Jetson device.

## Parameters

The following ROS2 parameters are available:

- `servo_min` (double, default: `0.15`): Minimum servo position.
- `servo_zero` (double, default: `0.4325`): Neutral servo position.
- `servo_max` (double, default: `0.7385`): Maximum servo position.
- `vehicleID` (string, default: `"rc-car"`): Identifier for the vehicle instance.
- `steering_wheel_topic` (string, default: `"/vesc/commands/servo/position"`): Topic for publishing steering commands.
- `engine_speed_topic` (string, default: `"/vesc/commands/motor/speed"`): Topic for publishing motor speed commands.
- `acceleration_topic` (string, default: `"/hedge_imu"`): Topic for receiving acceleration data.
- `parameter_folder` (string, default: `<package_share_directory>/config/`): Path to configuration files.
- `ackermann_command_topic` (string, default: `"/vesc/low_level/ackermann_cmd_mux/output"`): Topic for Ackermann drive commands.

## Subscribed Topics

- Ackermann commands: `/vesc/low_level/ackermann_cmd_mux/output` (`ackermann_msgs::msg::AckermannDrive`)

## Published Topics

- Steering commands: `/vesc/commands/servo/position`
- Speed commands: `/vesc/commands/motor/speed`

## Prerequisites / Dependencies

- rclcpp
- sensor_msgs
- nav_msgs
- ackermann_msgs

## TOD Package Dependencies

- tod_vehicle_msgs
- tod_generic_interface
- tod_core

## Build Instructions

To build the package, use:

```console
colcon build --packages-up-to tod_rc-car_interface
```

## Running the Package

The package can be launched using the provided launch files.

### Launch Files

- `rc-car_actuation_interface.launch.py`: Starts the actuation interface.
- `rc-car_sensing_interface.launch.py`: Starts the sensing interface.
- `tod_rc-car_interface.launch.py`: Starts the full RC-car interface including both actuation and sensing.

Example:

```console
ros2 launch tod_rc-car_interface tod_rc-car_interface.launch.py
```

## Configuration Files

The following configuration files are provided:

- **Package Configuration**:
  - `config/package_config/tod_rc-car_interface/params.yaml`

- **Vehicle Configuration**:
  - `config/vehicle_config/rc-car/vehicle-params.yaml`

Make sure to edit these files according to your hardware setup and vehicle-specific parameters before launching.
