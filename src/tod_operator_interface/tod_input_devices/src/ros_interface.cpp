/**
 * @file ros_interface.cpp
 * @brief ROS Abstraction for usb input devices
 * @copyright 2025 TUMFTM
 **/
#include "tod_input_devices/ros_interface.hpp"
#include "tod_input_devices/input_device_controller.hpp"
#include <QMainWindow>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace tod_input_device {

RosInterface::RosInterface(int argc, char** pArgv, InputDeviceController* parent)
    : Node("InputDevice"), _init_argc(argc), _p_init_argv(pArgv), _parent(parent) {

    this->declare_parameter<std::string>("type");
    this->declare_parameter<int>("button_config.IndicatorLeft");
    this->declare_parameter<int>("button_config.IndicatorRight");
    this->declare_parameter<int>("button_config.FlashLight");
    this->declare_parameter<int>("button_config.FrontLight");
    this->declare_parameter<int>("button_config.Honk");
    this->declare_parameter<int>("button_config.IncreaseSpeed");
    this->declare_parameter<int>("button_config.DecreaseSpeed");
    this->declare_parameter<int>("button_config.IncreaseGear");
    this->declare_parameter<int>("button_config.DecreaseGear");
    this->declare_parameter<int>("axis_config.Steering");
    this->declare_parameter<int>("axis_config.Throttle");
    this->declare_parameter<int>("axis_config.Brake");
    this->declare_parameter<bool>("invert_axis.Steering");
    this->declare_parameter<bool>("invert_axis.Throttle");
    this->declare_parameter<bool>("invert_axis.Brake");
    this->declare_parameter<bool>("input_device_has_separate_braking_axis");
    this->declare_parameter<std::string>("correction");
    clear_joystick_msg();
}

RosInterface::~RosInterface() {
    _ros_thread.join();
    rclcpp::shutdown();
}

void RosInterface::set_debug_mode() {
    auto ret = rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    RCLCPP_INFO(this->get_logger(), "Setting severity threshold to DEBUG");
    if (ret != RCUTILS_RET_OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to set logger level.");
    }
}

bool RosInterface::init() {
    node = shared_from_this();

    _joystick_msg_publisher = this->create_publisher<sensor_msgs::msg::Joy>("output/joystick", 100);
    _ros_thread = std::thread([=] { run(); });
    return true;
}

bool RosInterface::clear_joystick_msg() {
    _operator_msg.buttons.assign(31, 0);
    _operator_msg.axes.assign(4, 0);
    return true;
}

void RosInterface::run() {
    rclcpp::Rate r(100);

    while (rclcpp::ok() && _active) {
        _operator_msg.header.stamp = this->get_clock()->now();
        _joystick_msg_publisher->publish(_operator_msg);
        rclcpp::spin_some(get_node_base_interface());
        r.sleep();
    }
    if (_parent != nullptr) {
        _parent->terminate();
    }
}

std::string RosInterface::get_node_name() {
    return this->get_name();
}

std::string RosInterface::get_package_path() {
    return ament_index_cpp::get_package_share_directory("tod_input_devices");
}

void RosInterface::set_button(const int &button, const int &state) {
    if (static_cast<size_t>(button) > _operator_msg.buttons.size() - 1) {
        RCLCPP_ERROR_STREAM(this->get_logger(), get_node_name() << ": tried to access button " << button
            << " only buttons between 0 and " << _operator_msg.buttons.size() - 1
            << " possible!");
        return;
    }
    _operator_msg.buttons.at(button) = state;
}

void RosInterface::set_axis(const int &axis, const float &value) {
    if (static_cast<size_t>(axis) > _operator_msg.axes.size() - 1) {
        RCLCPP_ERROR_STREAM(this->get_logger(), get_node_name() << ": tried to access axis " << axis
            << " only axes between 0 and " << _operator_msg.axes.size() - 1
            << " possible!");
        return;
    }
    _operator_msg.axes.at(axis) = value;
}

} // namespace tod_input_device
