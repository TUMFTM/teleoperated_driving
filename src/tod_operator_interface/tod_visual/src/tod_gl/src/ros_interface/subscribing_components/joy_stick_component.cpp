/**
 * @file joy_stick_component.cpp
 * @brief Joystick component that manages the subscription and the data for joy topics .
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/subscribing_components/joy_stick_component.hpp"

namespace tod_gl {

void JoyStickComponent::cb_message(const sensor_msgs::msg::Joy::SharedPtr msg) {
    axes_ = msg->axes;
    buttons_ = msg->buttons;
}

}  // namespace tod_gl