# tod_communication_interface
 
## Overview
 
This package defines the communication interface between the vehicle and the operator. More specifically, this packages defines the ROS topics and services that are mapped between the tod_vehicle and tod_operator.

## Topic Sending

## Service Forwarding
 This pacakge forwards ROS services by instantiating a `tod_network::ServiceForwarder` and a `tod_network::ServiceListener`. Currently, the service forwarding support the following service types:
- NetworkMonitorService (`tod_config_msgs::srv::NetworkMonitorService`)
- PacketCaptureService (`tod_config_msgs::srv::PacketCaptureService`)
- VideoConfig (`tod_config_msgs::srv::VideoConfig`)
- VideoParamService (`rcl_interfaces::srv::SetParameters`)

If another service needs to be forwarded, a new node with the service type needs to be implemented under this package and added to the launch file `tod_communication_interface.launch.py`. 

The service forwarding is controlled by the config file `services.yaml` from the package `tod_launch`. In the config file, the protocol, the service name at the vehicle side, and the service name at the operator side needs to be define.
 
 
## Nodes

### Service Forwarding
 
NetworkMonitorServiceForwarder
 
    Subscribed Topics:/Operator/Manager/status_msg (tod_status_msgs::msg::Status)
    Services: service name defined by config file (tod_network_monitoring_msgs::srv::NetworkMonitorService)
 
NetworkMonitorServiceListener
 
    Subscribed Topics: /Vehicle/Manager/status_msg (tod_status_msgs::msg::Status)
    Services: service name defined by config file (tod_network_monitoring_msgs::srv::NetworkMonitorService)

PacketCaptureServiceForwarder
 
    Subscribed Topics: /Operator/Manager/status_msg (tod_status_msgs::msg::Status)
    Services: service name defined by config file (tod_network_monitoring_msgs::srv::PacketCaptureService)

PacketCaptureServiceListener
 
    Subscribed Topics: /Vehicle/Manager/status_msg (tod_status_msgs::msg::Status)
    Services: service name defined by config file (tod_network_monitoring_msgs::srv::PacketCaptureService)

VideoConfigServiceForwarder
 
    Subscribed Topics: /Operator/Manager/status_msg (tod_status_msgs::msg::Status)
    Services: service name defined by config file (tod_config_msgs::srv::VideoConfig)

VideoConfigServiceListener
 
    Subscribed Topics: /Vehicle/Manager/status_msg (tod_status_msgs::msg::Status)
    Services: service name defined by config file (tod_config_msgs::srv::VideoConfig)

VideoParamServiceForwarder
 
    Subscribed Topics: /Operator/Manager/status_msg (tod_status_msgs::msg::Status)
    Services: service name defined by config file (rcl_interfaces::srv::SetParameters)

VideoParamServiceListener
 
    Subscribed Topics: /Vehicle/Manager/status_msg (tod_status_msgs::msg::Status)
    Services: service name defined by config file (rcl_interfaces::srv::SetParameters)
 
 
## Prerequisites / Dependencies
 
This Package was developed for ROS2 Humble.

Following packages are required:

- `rcl_interfaces`
 
## TOD Package dependencies
 
- `tod_config_msgs`
- `tod_status_msgs`
- `tod_network_monitoring_msgs`
- `tod_network_protocols`
- `tod_network`

 
## Build
 
```console
colcon build --packages-up-to tod_communication_interface
```
 
## Running the Package

Run the service forwarder and listener:

```console
ros2 run tod_communication_interface <service_forwarder/listener_node> <protocol> <service_name> <forwarder_port> <listener_port>
```

for instance:

```console
ros2 run tod_communication_interface VideoConfigServiceForwarder TCP /Vehicle/Network/Config/ToVehicle/VideoConfig 60100 60101
```
 
## Launch Files
 
```console
ros2 launch tod_communication_interface tod_communication_interface.launch.py
```
 
## Configuration Files
 
- `services.yaml` from package `tod_launch/config/package_config/tod_communication_interface`
