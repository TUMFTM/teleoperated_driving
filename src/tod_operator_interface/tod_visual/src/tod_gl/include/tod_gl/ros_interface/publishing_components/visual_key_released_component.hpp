/**
 * @file manager_button_status_component.hpp
 * @ingroup tod_gl_ros_interface
 * @brief Component for managing button status publishing.
 *
 * This file contains the declaration of the VisualKeyReleasedComponent class, which provides functionality 
 * to publish button status messages for state machine.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "tod_operator_msgs/msg/key_press.hpp"

namespace tod_gl {
  /**
 * @class VisualKeyReleasedComponent
 * @brief Handles publishing manager button status messages.
 *
 * The VisualKeyReleasedComponent provides an interface for publishing button status messages 
 * of type `tod_status_msgs::msg::ManagerButtonStatus` to a ROS topic.
 */

class VisualKeyReleasedComponent {
  public:
    VisualKeyReleasedComponent(std::shared_ptr<rclcpp::Node> subNode);
      void publish_key_release(const tod_operator_msgs::msg::KeyPress& msg);



  private:
    rclcpp::Publisher<tod_operator_msgs::msg::KeyPress>::SharedPtr _publisher;
};

}  // namespace tod_gl