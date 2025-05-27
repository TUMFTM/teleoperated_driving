/**
 * @file input_device_component.hpp
* @brief Service to change the input device before starting the teleoperation
 * @ingroup tod_gl_ros_interface
* @copyright 2024 TUMFTM
 **/
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "tod_operator_msgs/srv/input_device.hpp"

namespace tod_gl {

class InputDeviceComponent{
  public:
    InputDeviceComponent(std::shared_ptr<rclcpp::Node> subNode);
    void publish_input_device(const std::string& inputDevice);

  private:
    std::shared_ptr<rclcpp::Client<tod_operator_msgs::srv::InputDevice>> _client;
  };

} // namespace tod_gl