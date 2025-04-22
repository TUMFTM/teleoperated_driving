# tod_network

## Overview

This package provides two communication components:
- Boost-based TCP sender & receiver
- TOD sevice forwarder & listener

## Nodes

This package serves as a library package which does not contain nodes or C++ executables


## Topics

The classes in this package subscribes to two ROS2 topics of type `tod_status_msgs::msg::Status`:

- `/vehicle/statemachine/vehicle_status`
- `/operator/statemachine/output/operator_status`


## Prerequisites

## TOD Package Dependencies

- `tod_status_msgs`
- `tod_network_protocols`

## Build

```
colcon build --packages-up-to tod_network && source install/setup.bash
```

## Run

This package serves as a library package and does not contain nodes or executables.

## Launch

This package serves as a library package and does not contain launch files.

## Configuration

This package serves as a library package and does not contain configurations.
