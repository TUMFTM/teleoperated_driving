/**
 * @file ros_interface.cpp
 * @brief ROS Abstraction for usb input devices
 * @copyright 2025 TUMFTM
 **/


#pragma once
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <memory>
#include <utility>
#include "virtual_input_device.hpp"
#include "tod_input_devices/usb_input_device/usb_input_device.hpp"
#if SENSO_MACRO 
#include "senso_input_device.h"
#endif
#include "tod_input_devices/ros_interface.hpp"
#include "tod_operator_msgs/joystickConfig.h"

#include "sensor_msgs/msg/joy.hpp"
#include <filesystem>
#include "tod_operator_msgs/srv/input_device.hpp"

namespace tod_input_device {

/**
 * @struct AxisItem
 * @brief Represents an axis configuration item, including position and inversion status.
 */
struct AxisItem {
    AxisItem(joystick::AxesPos pos, bool inv) : position(pos), invert_axes(inv) {}
    joystick::AxesPos position;
    bool invert_axes;
};

/**
 * @class InputDeviceController
 * @brief Manages input devices, their configurations, and mappings in a ROS2 environment.
 * 
 * This class is responsible for initializing, configuring, and managing multiple input devices.
 * It handles callbacks for input events, error reporting, and service requests to change devices.
 */
class InputDeviceController {
public:
    /**
     * @brief Constructor for the InputDeviceController class.
     * @param argc Number of command-line arguments.
     * @param argv Pointer to an array of command-line arguments.
     * 
     * Initializes ROS2 node, declares parameters, and sets up input devices.
     */
    explicit InputDeviceController(int argc, char** argv);
    
    std::string inputDeviceType;

    /**
     * @brief Destructor for the InputDeviceController class.
     * 
     * Ensures proper termination of ROS2 node and all managed input devices.
     */
    ~InputDeviceController();

    /**
     * @brief Terminates all active input devices.
     */
    void terminate();

private:
    // Input Device Configuration
    std::map<std::string, std::shared_ptr<MyInputDevice>> _input_devices;    ///< Map of input device instances.
    std::map<int, joystick::ButtonPos> _button_mapping;                     ///< Mapping of hardware buttons to joystick positions.
    std::map<int, AxisItem> _axis_mapping;                                  ///< Mapping of hardware axes to joystick positions.

    // Input Devices Callback
    /**
     * @brief Callback for when an axis value changes.
     * @param axis The hardware axis index.
     * @param value The new value of the axis.
     */
    void callback_axis_changed(const int axis, const double value);

    /**
     * @brief Callback for when a button state changes.
     * @param button The hardware button index.
     * @param state The new state of the button (pressed/released).
     */
    void callback_button_changed(const int button, const int state);

    /**
     * @brief Callback for reporting errors from input devices.
     * @param errorMsg The error message to report.
     */
    void callback_error(const std::string& errorMsg);

    // Service Callback
    /**
     * @brief Callback for handling requests to change the active input device.
     * @param request The request containing the new input device configuration.
     * @param response The response indicating whether the change was successful.
     * @return `true` if the device change was successful, `false` otherwise.
     */
    bool callback_change_device_request(
        const std::shared_ptr<tod_operator_msgs::srv::InputDevice::Request> request,
        const std::shared_ptr<tod_operator_msgs::srv::InputDevice::Response> response);

    // Other Member Variables
    bool _configurationMode{false};     ///< Indicates whether the controller is in configuration mode.
    bool _debug{false};                 ///< Indicates whether debug mode is enabled.
    std::shared_ptr<RosInterface> _ros; ///< Pointer to the ROS interface instance.

    // Other Member Functions
    /**
     * @brief Changes the active input device based on current configuration.
     */
    void change_input_device();

    /**
     * @brief Updates the button and axis mappings from the parameter workspace.
     */
    void update_mapping_from_param_workspace();

    /**
     * @brief Parses a YAML file and converts it into a list of ROS2 parameters.
     * @param path_to_yaml The file path to the YAML configuration.
     * @return A vector of parsed ROS2 parameters.
     */
    std::vector<rclcpp::Parameter> Yayp_te(std::string path_to_yaml);
};

} // namespace tod_input_device
