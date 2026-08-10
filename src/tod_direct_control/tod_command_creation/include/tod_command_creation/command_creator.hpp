// Copyright TUM-FTM

/**
 * @file CommandCreator.hpp
 * @brief Defines the CommandCreator class that inherits from the Node class for managing
 * joystick-based vehicle control. Computes a PrimaryControlCmd and SecondaryControlCmd  
 * based on the inputdevice input from the node in tod_input_devices
 */


#pragma once
#include "sensor_msgs/msg/joy.hpp"
#include "tod_status_msgs/msg/status.hpp"
#include "tod_vehicle_msgs/msg/primary_control_cmd.hpp"
#include "tod_vehicle_msgs/msg/secondary_control_cmd.hpp"
#include "tod_vehicle_msgs/VehicleEnums.h"
#include "tod_operator_msgs/joystickConfig.h"
#include "tod_command_creation/gear_selector.hpp"
#include <tod_helper/vehicle/Model.h>
#include <utility>
#include <stdio.h>
#include <vector>
#include <map>
#include <algorithm>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "tod_core/param_set/param_sets.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace tod_command_creation {
/**
 * @class CommandCreator
 * @brief A ROS2 node to process joystick inputs and generate primary and secondary control commands.
 *
 * The CommandCreator class subscribes to joystick and status messages, processes the input data,
 * and publishes appropriate control commands for a vehicle.
 */
class CommandCreator : public rclcpp::Node
{
public:
    /**
     * @brief Constructs the CommandCreator node.
     *
     * Initializes subscriptions, publishers, parameters, and control messages.
     */
    CommandCreator();

private:
   
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joystickSubs;
    rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr _statusSubs;
    rclcpp::Publisher<tod_vehicle_msgs::msg::PrimaryControlCmd>::SharedPtr _primaryControlPub;
    rclcpp::Publisher<tod_vehicle_msgs::msg::SecondaryControlCmd>::SharedPtr _secondaryControlPub;
    std::shared_ptr<rclcpp::ParameterEventHandler> _param_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> _cb_handle;

    std::map<joystick::ButtonPos, int> _prevButtonState;                    ///< Previous states of joystick buttons.
    uint8_t _status{tod_status_msgs::msg::Status::TOD_STATUS_IDLE};         ///< Current system status.
    tod_vehicle_msgs::msg::PrimaryControlCmd _primaryControlMsg;     ///< Primary control message.
    tod_vehicle_msgs::msg::SecondaryControlCmd _secondaryControlMsg; ///< Secondary control message.
    std::unique_ptr<tod_core::param_set::Vehicle> vehicleParamHandler_;     ///< Vehicle parameter handler.
    std::unique_ptr<GearSelector> _gearSelector;

    bool _constraintSteeringRate{false};
    bool _invertSteeringInGearReverse{false};
    float _maxSpeedms{10};
    float _maxAcceleration{4};
    float _maxDeceleration{9};
    double _maxSteeringWheelAngleRate{15.0};
    bool _joystickInputSet{false};
    bool _inputDeviceHasSeparateBrakingAxis{true};
    std::string _vehicleID{"edgar"};
    rclcpp::TimerBase::SharedPtr _timer;                                    ///< Timer for cyclic msg publishing
    size_t _count;

    /**
     * @brief Initializes the control messages with default values.
     */
    void init_control_messages();
    
    /**
     * @brief Callback for handling incoming joystick messages.
     * @param msg The received joystick message.
     */
    void callback_joystick_msg(const sensor_msgs::msg::Joy &msg);

    /**
     * @brief Callback for handling incoming status messages.
     * @param msg The received status message.
     */
    void callback_status_msg(const tod_status_msgs::msg::Status &msg);

    /**
     * @brief Calculates the desired steering wheel angle.
     * @param out The primary control command to populate.
     * @param axes The joystick axes values.
     */
    void calculate_steering_wheel_angle(tod_vehicle_msgs::msg::PrimaryControlCmd &out, const std::vector<float> &axes);

    /**
     * @brief Calculates the desired velocity based on joystick input and gear.
     * @param out The primary control command to populate.
     * @param msg The joystick message.
     * @param gear The current gear.
     */
    void calculate_desired_velocity(tod_vehicle_msgs::msg::PrimaryControlCmd &out, const sensor_msgs::msg::Joy &msg,
            const int gear);

    /**
     * @brief Sets the gear based on joystick button states.
     * @param out The secondary control command to populate.
     * @param buttonState The joystick button states.
     * @param currentVelocity The current vehicle velocity.
     */
    void set_gear(tod_vehicle_msgs::msg::SecondaryControlCmd &out, const std::vector<int> &buttonState,
            const float &currentVelocity);

    /**
     * @brief Sets the indicator state based on joystick button states.
     * @param out The secondary control command to populate.
     * @param buttonState The joystick button states.
     */
    void set_indicator(tod_vehicle_msgs::msg::SecondaryControlCmd &out, const std::vector<int> &buttonState);

    /**
     * @brief Sets the light state based on joystick button states.
     * @param out The secondary control command to populate.
     * @param buttonState The joystick button states.
     */
    void set_light(tod_vehicle_msgs::msg::SecondaryControlCmd &out, const std::vector<int> &buttonState);
    
    /**
     * @brief Sets the honk state based on joystick button states.
     * @param out The secondary control command to populate.
     * @param buttonState The joystick button states.
     */
    void set_honk(tod_vehicle_msgs::msg::SecondaryControlCmd &out, const std::vector<int> &buttonState);
    
    /**
     * @brief Timer callback for periodic message publishing.
     */
    void timer_callback();

    /**
     * @brief Callback for parameter changes.
     * @param p The changed parameter.
     */
    void cb_change_param(const rclcpp::Parameter & p);
};
} // namespace
