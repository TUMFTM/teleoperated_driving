# tod_generic_interface
 
## Overview
 
This package aims at providing a standardized interface for the ros2 tod software to/from the individual driving platform. This packages doesn't host nodes but rather provides a library for **your** platform interface. It consists of four mayor components:
- **generic_interface::BaseInterface** class provides basic functionalities to predefine any tod_specific ros2 message interface in this package while allowing straight forward configuration of the platform message interface later on. Core of the class is an attribute list that serves as a common ground between platform and tod system for data exchange and
is configured in the following child classes.
- **generic_interface::ActuationInterface** class preconfigures the message interface to and from the actuation.
- **generic_interface::SensingInterface** class preconfigures some message forwarder to provide the sensor data in the right namespace tod_system wide
- **generic_interface::AutomationInterface** class preconfigures the message interface to and from the automation

 
## Classes

**generic_interface::BaseInterface**
    
    - void add_attribute(const std::string& attribute_name, T attribute_value): called when creating a new common attribute with a specific type T, e.g. ```add_attribute<float>("PrimaryCtrl_SteeringWheelAngle", 0.0)```
    - T get_attribute(const std::string& attribute_name): fetches the attribute value for you if the type matches, e.g. ```get_attribute<float>("PrimaryCtrl_SteeringWheelAngle")```
    - update_attribute(const std::string& attribute_name, T attribute_value): updated an attribute value if you want to configure a subscription, automatically assignes the current time to the attribute, e.g. ```update_attribute("PrimaryCtrl_SteeringWheelAngle", static_cast<float>(0.0))```
    - add_publisher: constructs a publisher on the given topic namespace that checks every refresh_time if at least one of the attributes in attribute_names changed, and then constructs the message to be published using the provided message_builder. Inside the message_builder you would then use the get_attribute function to collect the required data for your message from the common attributes.
    - add_subscriber: constructs a subscriber to a given namespace that hands the message over to a provided message_handler. Inside the message handler you would update common attributes using update_attribute, sometimes conversions are necessary prior to updating
    - add_forwarder: constructs a subscriber and publisher pair that simply publishes a message from an input_topic on the output_topic without modifying it.

**generic_interface::ActuationInterface**

    common attributes defined:
        - PrimaryCtrl_SteeringWheelAngle (float)
        - PrimaryCtrl_SteeringTireAngle (float)
        - PrimaryCtrl_Velocity (float)
        - PrimaryCtrl_Acceleration (float)
        - SecondaryCtrl_Indicator (int8_t)
        - SecondaryCtrl_Gear (int8_t)
        - SecondaryCtrl_Honk (int8_t)
        - SecondaryCtrl_Wiper (int8_t)
        - SecondaryCtrl_HeadLight (int8_t)
        - SecondaryCtrl_FlashLight (int8_t)
        - PrimaryVehicleData_SteeringWheelAngle (float)
        - PrimaryVehicleData_SteeringTireAngle (float)
        - PrimaryVehicleData_Velocity (float)
        - PrimaryVehicleData_Acceleration (float)
        - SecondaryVehicleData_Indicator (int8_t)
        - SecondaryVehicleData_Gear (int8_t)
        - SecondaryVehicleData_Honk (int8_t)
        - SecondaryVehicleData_Wiper (int8_t)
        - SecondaryVehicleData_HeadLight (int8_t)
        - SecondaryVehicleData_FlashLight (int8_t)
        - SafetyDriverStatus_Long (int8_t)
        - SafetyDriverStatus_Lat (int8_t)
        - SafetyDriverStatus_EmergencyStopReleased (int8_t)

    Subscribed Topics: 
        - /Vehicle/Interface/Actuation/ToActuation/primary_control_cmd (tod_vehicle_msgs::msg::PrimaryControlCmd)
        - /Vehicle/Interface/Actuation/ToActuation/secondary_control_cmd (tod_vehicle_msgs::msg::SecondaryControlCmd)
    Published Topics: 
        - /Vehicle/Interface/Actuation/FromActuation/primary_vehicle_state (tod_vehicle_msgs::msg::PrimaryVehicleState)
        - /Vehicle/Interface/Actuation/FromActuation/secondary_vehicle_state (tod_vehicle_msgs::msg::SecondaryVehicleState)
        - /Vehicle/Interface/Actuation/FromActuation/safety_driver_status (tod_vehicle_msgs::msg::SafetyDriverStatus)
    Services: none
    Parameters: none

**generic_interface::SensingInterface**

    common attributes defined: none
    Subscribed Topics: None (configured by the platform interface to forward to the publish topics)
    Published Topics: 
        - /Vehicle/Interface/Sensing/GNSS/fix (sensor_msgs::msg::NavSatFix)
        - /Vehicle/Interface/Sensing/IMU/imu (sensor_msgs::msg::Imu)
        - /Vehicle/Interface/Sensing/Odometry/odom (nav_msgs::msg::Odometry)
        - /Vehicle/Interface/Sensing/Camera/frontcenter/image (sensor_msgs::msg::Image)
    Services: none
    Parameters: none
 
 
## Prerequisites / Dependencies
 
rclcpp
sensor_msgs
nav_msgs
tod_vehicle_messages
 
## TOD Package dependencies
 
None
 
## Build
 
```console
colcon build --packages-up-to tod_generic_interface
```
 
## Running the Package
 
Not possible, because this is only a library
 
## Launch Files

None
 
## Configuration Files
 
None