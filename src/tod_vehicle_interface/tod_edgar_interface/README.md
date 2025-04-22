# TOD EDGAR Interface {#tod_edgar_interface_doc}

## Overview
This packages contains the interfaces for the research vehicle EDGAR without AV functionalities. It enables teleoperating EDGAR using Remote Driving concepts such as Direct Control and Trajectory Guidance. 

## Nodes
- `edgar_sensing_interface`: Interface to obtain data from EDGAR's sensor drivers. To minimize latency, camera and LiDAR data are not transmitted via this interface. Topics for this data are specified via EDGAR's vehicle config: `vehicle_config\edgar`. If you are looking for further information regarding camera and LiDAR see `tod_rtsp` and `tod_lidar`.
- `edgar_actuation_interface`: Interface to obtain states of EDGAR's actuators and secondary vehicle functions and to pass control commands them.

## Subscribed Topics
- **Sensing Interface:** `edgar_sensing_interface`
  - GNSS [sensor_msgs/msg/NavSatFix]: `/vehicle/sensor/fix`
  - IMU [sensor_msgs/msg/Imu]:  `/edgar/sensor/gnss/novatel/center/imu`
  - Odometry [nav_msgs/msg/Odometry]: `/edgar/sensor/gnss/novatel/center/odom`
  
- **Actuation Interface:** `edgar_actuation_interface`
  - See `tod_generic_interface/sensing_interface` for subscribers towards the TUM Teleoperation Software.
  - EDGAR's CAN interface states [tum_edgar_can_msgs/msg/TUMEdgarGatewayStates]: `/edgar/can/gateway_states`
  - EDGAR's CAN powertrain states [tum_edgar_can_msgs/msg/TUMEdgarPowertrain1]: `/edgar/can/powertrain1`
  - EDGAR's CAN secondary function states [tum_edgar_can_msgs/msg/TUMEdgarMisc1]: `/edgar/can/misc1`
  - EDGAR's CAN motion states [tum_edgar_can_msgs/msg/TUMEdgarMotion]: `/edgar/can/motion_vw`
  - EDGAR's CAN steering state [tum_edgar_can_msgs/msg/TUMEdgarSteering]: `/edgar/can/steering_vw`
  - EDGAR's Autoware interface gear states: [autoware_auto_vehicle_msgs/msg/GearReport]: `/vehicle/status/gear_status`
  - EDGAR's Autoware interface hazard light state: [autoware_auto_vehicle_msgs/msg/HazardLightsReport]: `/vehicle/status/hazard_lights_status`
  - EDGAR's Autoware interface head light state: [tum_autoware_vehicle_msgs/msg/HeadLightReport]: `/vehicle/status/head_light_status`
  - EDGAR's Autoware interface high beam state: [tum_autoware_vehicle_msgs/msg/HighBeamReport]: `/vehicle/status/high_beam_status`
  - EDGAR's Autoware interface honk states: [tum_autoware_vehicle_msgs/msg/HonkReport]: `/vehicle/status/honk_status`
  - EDGAR's Autoware interface steering: [autoware_auto_vehicle_msgs/msg/SteeringReport]: `/vehicle/status/steering_status`
  - EDGAR's Autoware interface turn indicator state: [autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport]: `/vehicle/status/turn_indicators_status`
  - EDGAR's Autoware interface velocity: [autoware_auto_vehicle_msgs/msg/VelocityReport]: `/vehicle/status/velocity_status`
  - EDGAR's Autoware interface wiper state: [tum_autoware_vehicle_msgs/msg/WiperReport]: `/vehicle/status/wiper_status`
  
## Published Topics
- **Sensing Interface:** `edgar_sensing_interface`
  - See `tod_generic_interface/sensing_interface` for publishers towards the TUM Teleoperation Software.
  
- **Actuation Interface:** `edgar_actuation_interface`
  - See `tod_generic_interface/sensing_interface` for publishers towards the TUM Teleoperation Software.
  - EDGAR's Autoware interface gear command [autoware_auto_vehicle_msgs/msg/GearCommand]: `/control/command/gear_cmd`
  - EDGAR's Autoware interface ackermann control command [autoware_auto_control_msgs/msg/AckermannControlCommand]: `/control/command/control_cmd`
  - EDGAR's Autoware interface hazard light command [autoware_auto_vehicle_msgs/msg/HazardLightsCommand]: `/control/command/hazard_lights_cmd`
  - EDGAR's Autoware interface indicator command [autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand]: `/control/command/turn_indicators_cmd`

## Parameters
All ROS parameters of the tod_edgar_interface are set via the file `tod_edgar_interface/config/package_config/tod_edgar_interface/params.yaml`. For each of the published and subscribed topics mentioned above that are not part of the `tod_generic_interface`, there are parameters to set topic names. Additional to that the tod_edgar_interface has the following ROS parameters:

- **Sensing Interface:** `edgar_sensing_interface`
  - none

- **Actuation Interface:** `edgar_actuation_interface`
  - **kp** [double]: P gain factor for the velocity controller.
  - **ki** [double]: I gain factor for the velocity controller.
  - **max_steer_wheel** [double]: Maximum steering wheel angle [rad] passed to EDGAR.
  - **wheel2tire_factor** [double]: Ratio of steering wheel angle to steering wheel tire angle. 

## Build
```bash
colcon build --packages-up-to tod_edgar_interface
```

## Launch Files
```bash
ros2 launch tod_edgar_interface.launch.py
```

## Launch Arguments
- `config_path` (string, default: "tod_edgar_interface/config") - Absolute path to the directory containing configuration files for the EDGAR interface. 
- `mode` (string, default: "vehicle") - EDGAR interface is only launched if mode equals to "vehicle".

## Doxygen Documentation
\version 1.0  
\author TUMFTM  
\defgroup tod_edgar_interface
\brief Interface for the research vehicle EDGAR without AV functionalities.