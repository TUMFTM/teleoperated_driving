/**
 * @file manager_button_status_component.hpp
 * @ingroup tod_gl_ros_interface
 * @brief Component for managing button status publishing.
 *
 * This file contains the declaration of the VisualKeyPressedComponent class, which provides functionality 
 * to publish button status messages for state machine.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "tod_operator_msgs/msg/key_press.hpp"

namespace tod_gl {
  /**
 * @class VisualKeyPressedComponent
 * @brief Handles publishing visual messages.
 *
 * The VisualKeyPressedComponent provides an interface for publishing key messages 
 * of type `tod_operator_msgs::msg::KeyPress` to a ROS topic.
 */

class VisualKeyPressedComponent {
  public:
    VisualKeyPressedComponent(std::shared_ptr<rclcpp::Node> subNode);
      void publish_key_press(const tod_operator_msgs::msg::KeyPress& msg);



  private:
    rclcpp::Publisher<tod_operator_msgs::msg::KeyPress>::SharedPtr _publisher;
};

}  // namespace tod_gl