# TOD EDGAR AUTOWARE Interface {#tod_edgarautoware_interface_doc}

## Overview
This packages contains the interfaces for the research vehicle EDGAR with Autoware as AV stack. It enables teleoperating EDGAR using Remote Driving and Remote Assistance concepts using Autoware as autonomous driving system.

## Nodes
- `edgarautoware_sensing_interface`: Interface to obtain data from EDGAR's Autoware sensor interface. To minimize latency, camera and LiDAR data are not transmitted via this interface. Topics for this data are specified via EDGAR's vehicle config: `vehicle_config\edgar`. If you are looking for further information regarding camera and LiDAR see `tod_rtsp` and `tod_lidar`.
- `edgarautoware_actuation_interface`: Interface to Autoware's external command and vehicle interface. If required data is not part of the Autoware interfaces we directly subscribe to EDGAR's driver interface.
- `edgarautoware_automation_interface`: Interface to interact with Autoware.
- 
## Subscribed Topics
- **Sensing Interface:** `edgarautoware_sensing_interface`
  - GNSS [sensor_msgs/msg/NavSatFix]: `/vehicle/sensor/fix`
  - IMU [sensor_msgs/msg/Imu]:  `/vehicle/sensor/imu1`
  - Odometry [nav_msgs/msg/Odometry]: `/localization/kinematic_state`
  
- **Actuation Interface:** `edgarautoware_actuation_interface`
  - See `tod_generic_interface/sensing_interface` for subscribers towards the TUM Teleoperation Software.
  - EDGAR's CAN interface states [tum_edgar_can_msgs/msg/TUMEdgarGatewayStates]: `/edgar/can/gateway_states`
  - EDGAR's CAN motion states [tum_edgar_can_msgs/msg/TUMEdgarMotion]: `/edgar/can/motion_vw`
  - EDGAR's Autoware interface gear states: [autoware_auto_vehicle_msgs/msg/GearReport]: `/vehicle/status/gear_status`
  - EDGAR's Autoware interface hazard light state: [autoware_auto_vehicle_msgs/msg/HazardLightsReport]: `/vehicle/status/hazard_lights_status`
  - EDGAR's Autoware interface head light state: [tum_autoware_vehicle_msgs/msg/HeadLightReport]: `/vehicle/status/head_light_status`
  - EDGAR's Autoware interface high beam state: [tum_autoware_vehicle_msgs/msg/HighBeamReport]: `/vehicle/status/high_beam_status`
  - EDGAR's Autoware interface honk states: [tum_autoware_vehicle_msgs/msg/HonkReport]: `/vehicle/status/honk_status`
  - EDGAR's Autoware interface steering: [autoware_auto_vehicle_msgs/msg/SteeringReport]: `/vehicle/status/steering_status`
  - EDGAR's Autoware interface turn indicator state: [autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport]: `/vehicle/status/turn_indicators_status`
  - EDGAR's Autoware interface velocity: [autoware_auto_vehicle_msgs/msg/VelocityReport]: `/vehicle/status/velocity_status`
  - EDGAR's Autoware interface wiper state: [tum_autoware_vehicle_msgs/msg/WiperReport]: `/vehicle/status/wiper_status`

- **Automation Interface:** `edgarautoware_automation_interface`
  - Autoware system automation state: [tod_automation_msgs/msg/VehicleAutomationState]: `/vehicle/interface/automation/from_automation/automation_state`
  - Autoware perception predicted objects: [tod_automation_msgs/msg/PredictedObjects]: `/vehicle/interface/automation/from_automation/predicted_objects`
  - Autoware planning trajectory: [tod_automation_msgs/msg/Trajectory]: `/vehicle/interface/automation/from_automation/trajectory`

## Published Topics
- **Sensing Interface:** `edgarautoware_sensing_interface`
  - See `tod_generic_interface/sensing_interface` for publishers towards the TUM Teleoperation Software.
  
- **Actuation Interface:** `edgar_actuation_interface`
  - See `tod_generic_interface/sensing_interface` for publishers towards the TUM Teleoperation Software.
  - EDGAR's Autoware external interface gear command [autoware_auto_vehicle_msgs/msg/GearCommand]: `/external/selected/gear_cmd`
  - EDGAR's Autoware external interface ackermann control command [autoware_auto_control_msgs/msg/AckermannControlCommand]: `/external/selected/control_cmd`
  - EDGAR's Autoware external interface hazard light command [autoware_auto_vehicle_msgs/msg/HazardLightsCommand]: `/external/selected/hazard_lights_cmd`
  - EDGAR's Autoware external interface indicator command [autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand]: `/external/selected/turn_indicators_cmd`
  - EDGAR's Autoware external interface heartbeat [tier4_external_api_msgs/msg/Heartbeat]: `/external/selected/heartbeat`

- **Automation Interface:** `edgarautoware_automation_interface`
  - See `tod_generic_interface/sensing_interface` for publishers towards the TUM Teleoperation Software.
  
## Parameters
All ROS parameters of the tod_edgarautoware_interface are set via the file `tod_edgarautoware_interface/config/package_config/tod_edgar_interface/params.yaml`. For each of the published and subscribed topics mentioned above that are not part of the `tod_generic_interface`, there are parameters to set topic names. Additional to that the tod_edgarautoware_interface has the following ROS parameters:

- **Sensing Interface:** `edgarautoware_sensing_interface`
  - none

- **Actuation Interface:** `edgarautoware_actuation_interface`
  - **kp** [double]: P gain factor for the velocity controller.
  - **ki** [double]: I gain factor for the velocity controller.
  - **max_steer_wheel** [double]: Maximum steering wheel angle [rad] passed to EDGAR.
  - **wheel2tire_factor** [double]: Ratio of steering wheel angle to steering wheel tire angle. 
  
- **Automation Interface:** `edgarautoware_automation_interface`
  - none

## Build
```bash
colcon build --packages-up-to tod_edgarautoware_interface
```

## Launch Files
```bash
ros2 launch tod_edgarautoware_interface.launch.py
```

## Launch Arguments
- `config_path` (string, default: "tod_edgarautoware_interface/config") - Absolute path to the directory containing configuration files for the EDGAR AUTOWARE interface. 
- `mode` (string, default: "vehicle") - EDGAR AUTOWARE interface is only launched if mode equals to "vehicle".

## Doxygen Documentation
\version 1.0  
\author TUMFTM  
\defgroup tod_edgarautoware_interface
\brief Interface for the research vehicle EDGAR with Autoware as AV stack.