# TOD Dummy {#tod_dummy_doc}

## Overview
This packages contains nodes to publish messages for testing proposes.

## Nodes
- `joystick_dummy_pub`: Node to publish joystick messages. 
- `path_dummy_pub`: Node to publish path messages. 
- `status_dummy_pub`: Node to publish vehicle and operator status messages. 

## Subscribed Topics
- none
  
## Published Topics
- **Joystick Dummy:** `joystick_dummy_pub`
  - Joystick [sensor_msgs/msg/Joy]: `/dummy/output/joystick`
  
- **Path Dummy:** `path_dummy_pub`
  - Autoware path [autoware_auto_planning_msgs/msg/Path]: `/dummy/output/path`

- **Status Dummy:** `status_dummy_pub`
  - Operator status [tod_status_msgs/msg/Status]: `/dummy/output/operator_status`
  - Vehicle status [tod_status_msgs/msg/Status]: `/dummy/output/vehicle_status`

## Parameters
- none

## Build
```bash
colcon build --packages-up-to tod_dummy
```

## Launch Files
Remapping to desired topic names can be done in the launch file.
```bash
ros2 launch tod_dummy.launch.py
```

## Launch Arguments
- none

## Doxygen Documentation
\version 1.0  
\author TUMFTM  
\defgroup tod_dummy
\brief Dummy nodes to publish messages for testing.