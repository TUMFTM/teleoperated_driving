# tod_network

## Overview

This package provides following communication components:
- base_protocol for sending data packets through network
- TCP sender/receiver
- UDP sender/receiver
- TOD sender/receiver which serializes ROS messages and utilizes the TCP/UDP sender/receiver to send the messages as data packages through network

## Nodes

This package serves as a library package which does not contain nodes or C++ executables


## Topics

The classes in this package subscribes to two ROS2 topics of type `tod_status_msgs::msg::Status`:

- `/vehicle/statemachine/vehicle_status`
- `/operator/statemachine/output/operator_status`

The TOD sender/receiver also publishes to ROS topics of the type "tod_network_msgs::msg::PaketInfo" to all forwarded ROS2 topics under the topic name `${TOPIC_NAME}/paket_info`


## Prerequisites

## TOD Package Dependencies

- `tod_status_msgs`
- `tod_network_msgs`

## Build

```
colcon build --packages-up-to tod_network_protocols && source install/setup.bash
```

## Run

This package serves as a library package and does not contain nodes or executables.

## Launch

This package serves as a library package and does not contain launch files.

## Configuration

This package serves as a library package and does not contain configurations.
