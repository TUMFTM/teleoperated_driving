/**
 * @file joy_stick_component.hpp
 * @brief Joystick component that manages the subscription and the data for joy topics.
 * @copyright TUMFTM 2024
 **/

#pragma once

#include <vector>

#include "tod_gl/ros_interface/subscribing_component_base.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"

namespace tod_gl {
  
class JoyStickComponent : public SubscribingComponent<sensor_msgs::msg::Joy> {
  public:
    explicit JoyStickComponent(std::shared_ptr<rclcpp::Node> sub_node)
      : SubscribingComponent(sub_node, "input/joystick"),
        axes_(),
        buttons_()
    {}
    
    std::vector<float> get_axes() const { return axes_; }
    std::vector<int32_t> get_buttons() const { return buttons_; }
    
  private:
    void cb_message(const sensor_msgs::msg::Joy::SharedPtr msg) override;
    std::vector<float> axes_;
    std::vector<int32_t> buttons_;
};

}  // namespace tod_gl