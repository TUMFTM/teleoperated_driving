/**
 * @file manager_button_status_component.cpp
 * @brief Component to communicate in the manager to the state machine which buttons have been clicked by the operator, 
 * so that it can initiate appropriate actions for connection or disconnection and start or end of teleoperation
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/publishing_components/manager_button_status_component.hpp"

namespace tod_gl {

ManagerButtonStatusComponent::ManagerButtonStatusComponent(std::shared_ptr<rclcpp::Node> subNode) {
    _publisher = subNode->create_publisher<tod_status_msgs::msg::ManagerButtonStatus>("output/button_status", 10);
}

void ManagerButtonStatusComponent::publish_button_status(const tod_status_msgs::msg::ManagerButtonStatus& msg) {
    _publisher->publish(msg);
}

}  // namespace tod_gl
