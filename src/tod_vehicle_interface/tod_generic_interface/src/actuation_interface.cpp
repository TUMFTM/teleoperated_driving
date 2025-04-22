/**
 * @file actuation_interface.cpp
 * @brief Generic actuation interface.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_generic_interface
 */

#include "tod_generic_interface/actuation_interface.hpp"

#include "tod_vehicle_msgs/msg/primary_control_cmd.hpp"
#include "tod_vehicle_msgs/msg/secondary_control_cmd.hpp"
#include "tod_vehicle_msgs/msg/primary_vehicle_state.hpp"
#include "tod_vehicle_msgs/msg/secondary_vehicle_state.hpp"
#include "tod_vehicle_msgs/msg/safety_driver_status.hpp"

namespace tod_generic_interface {

ActuationInterface::ActuationInterface(rclcpp::Node::SharedPtr node)
    : BaseInterface(node) 
{

    // Initialize attributes using add_attribute
    add_attribute<float>("PrimaryCtrl_SteeringWheelAngle", 0.0);
    add_attribute<float>("PrimaryCtrl_SteeringTireAngle", 0.0);
    add_attribute<float>("PrimaryCtrl_Velocity", 0.0);
    add_attribute<float>("PrimaryCtrl_Acceleration", 0.0);
    add_attribute<int8_t>("SecondaryCtrl_Indicator", 0);
    add_attribute<int8_t>("SecondaryCtrl_Gear", 0);
    add_attribute<int8_t>("SecondaryCtrl_Honk", 0);
    add_attribute<int8_t>("SecondaryCtrl_Wiper", 0);
    add_attribute<int8_t>("SecondaryCtrl_HeadLight", 0);
    add_attribute<int8_t>("SecondaryCtrl_FlashLight", 0);
    add_attribute<float>("PrimaryVehicleData_SteeringWheelAngle", 0.0);
    add_attribute<float>("PrimaryVehicleData_SteeringTireAngle", 0.0);
    add_attribute<float>("PrimaryVehicleData_Velocity", 0.0);
    add_attribute<float>("PrimaryVehicleData_Acceleration", 0.0);
    add_attribute<int8_t>("SecondaryVehicleData_Indicator", 0);
    add_attribute<int8_t>("SecondaryVehicleData_Gear", 0);
    add_attribute<int8_t>("SecondaryVehicleData_Honk", 0);
    add_attribute<int8_t>("SecondaryVehicleData_Wiper", 0);
    add_attribute<int8_t>("SecondaryVehicleData_HeadLight", 0);
    add_attribute<int8_t>("SecondaryVehicleData_FlashLight", 0);
    add_attribute<int8_t>("SafetyDriverStatus_Long", 0);
    add_attribute<int8_t>("SafetyDriverStatus_Lat", 0);
    add_attribute<int8_t>("SafetyDriverStatus_EmergencyStopReleased", 0);

    // Set up subscriptions
    add_subscriber<tod_vehicle_msgs::msg::PrimaryControlCmd>(
        "to_actuation/primary_control_cmd",
        [this](const tod_vehicle_msgs::msg::PrimaryControlCmd& msg) {
            update_attribute("PrimaryCtrl_SteeringWheelAngle", static_cast<float>(msg.steering_wheel_angle));
            update_attribute("PrimaryCtrl_SteeringTireAngle", static_cast<float>(msg.steering_tire_angle));
            update_attribute("PrimaryCtrl_Velocity", static_cast<float>(msg.velocity));
            update_attribute("PrimaryCtrl_Acceleration", static_cast<float>(msg.acceleration));
        });

    add_subscriber<tod_vehicle_msgs::msg::SecondaryControlCmd>(
        "to_actuation/secondary_control_cmd",
        [this](const tod_vehicle_msgs::msg::SecondaryControlCmd& msg) {
            //only update bools if it changed to avoid that the controll commands to the vehicle cans are spammed for secondary control commands
            if(this->get_attribute<int8_t>("SecondaryCtrl_Indicator") != static_cast<int8_t>(msg.indicator))
                update_attribute("SecondaryCtrl_Indicator", static_cast<int8_t>(msg.indicator));
            if(this->get_attribute<int8_t>("SecondaryCtrl_Gear") != static_cast<int8_t>(msg.gear_position))
                update_attribute("SecondaryCtrl_Gear", static_cast<int8_t>(msg.gear_position));
            if(this->get_attribute<int8_t>("SecondaryCtrl_Honk") != static_cast<int8_t>(msg.honk))
                update_attribute("SecondaryCtrl_Honk", static_cast<int8_t>(msg.honk));
            if(this->get_attribute<int8_t>("SecondaryCtrl_Wiper") != static_cast<int8_t>(msg.wiper))
                update_attribute("SecondaryCtrl_Wiper", static_cast<int8_t>(msg.wiper));
            if(this->get_attribute<int8_t>("SecondaryCtrl_HeadLight") != static_cast<int8_t>(msg.head_light))
            update_attribute("SecondaryCtrl_HeadLight", static_cast<int8_t>(msg.head_light));
            if(this->get_attribute<int8_t>("SecondaryCtrl_FlashLight") != static_cast<int8_t>(msg.flash_light))
            update_attribute("SecondaryCtrl_FlashLight", static_cast<int8_t>(msg.flash_light));
        });

    // Set up publishers
    add_publisher<tod_vehicle_msgs::msg::PrimaryVehicleState>(
        "from_actuation/primary_vehicle_state",
        [this]() {
            tod_vehicle_msgs::msg::PrimaryVehicleState output = tod_vehicle_msgs::msg::PrimaryVehicleState();
            output.header.stamp = get_clock().now();
            output.steering_wheel_angle = get_attribute<float>("PrimaryVehicleData_SteeringWheelAngle");
            output.steering_tire_angle = get_attribute<float>("PrimaryVehicleData_SteeringTireAngle");
            output.velocity = get_attribute<float>("PrimaryVehicleData_Velocity");
            output.acceleration = get_attribute<float>("PrimaryVehicleData_Acceleration");
            return output;
        },
        {"PrimaryVehicleData_SteeringWheelAngle", "PrimaryVehicleData_SteeringTireAngle", "PrimaryVehicleData_Velocity",
            "PrimaryVehicleData_Acceleration"},
        100);

    add_publisher<tod_vehicle_msgs::msg::SecondaryVehicleState>(
        "from_actuation/secondary_vehicle_state",
        [this]() {
            tod_vehicle_msgs::msg::SecondaryVehicleState output =tod_vehicle_msgs::msg::SecondaryVehicleState();
            output.header.stamp = get_clock().now();
            output.indicator = get_attribute<int8_t>("SecondaryVehicleData_Indicator");
            output.gear_position = get_attribute<int8_t>("SecondaryVehicleData_Gear");
            output.honk = get_attribute<int8_t>("SecondaryVehicleData_Honk");
            output.wiper = get_attribute<int8_t>("SecondaryVehicleData_Wiper");
            output.head_light = get_attribute<int8_t>("SecondaryVehicleData_HeadLight");
            output.flash_light = get_attribute<int8_t>("SecondaryVehicleData_FlashLight");
            return output;
        },
        {"SecondaryVehicleData_Indicator", "SecondaryVehicleData_Gear", "SecondaryVehicleData_Honk", 
            "SecondaryVehicleData_Wiper", "SecondaryVehicleData_HeadLight", "SecondaryVehicleData_FlashLight"},
        100);

    add_publisher<tod_vehicle_msgs::msg::SafetyDriverStatus>(
        "from_actuation/safety_driver_status",
        [this]() {
            tod_vehicle_msgs::msg::SafetyDriverStatus output = tod_vehicle_msgs::msg::SafetyDriverStatus();
            output.vehicle_long_approved = get_attribute<int8_t>("SafetyDriverStatus_Long");
            output.vehicle_lat_approved = get_attribute<int8_t>("SafetyDriverStatus_Lat");
            output.vehicle_emergency_stop_released = get_attribute<int8_t>("SafetyDriverStatus_EmergencyStopReleased");
            return output;
        },
        {"SafetyDriverStatus_Long", "SafetyDriverStatus_Lat", "SafetyDriverStatus_EmergencyStopReleased"},
        100);
}

} // namespace tod_generic_interface