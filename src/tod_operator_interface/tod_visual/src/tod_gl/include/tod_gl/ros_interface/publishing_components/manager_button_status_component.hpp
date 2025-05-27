/**
 * @file manager_button_status_component.hpp
 * @ingroup tod_gl_ros_interface
 * @brief Component for managing button status publishing.
 *
 * This file contains the declaration of the ManagerButtonStatusComponent class, which provides functionality 
 * to publish button status messages for state machine.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "tod_status_msgs/msg/manager_button_status.hpp"

namespace tod_gl {
  /**
 * @class ManagerButtonStatusComponent
 * @brief Handles publishing manager button status messages.
 *
 * The ManagerButtonStatusComponent provides an interface for publishing button status messages 
 * of type `tod_status_msgs::msg::ManagerButtonStatus` to a ROS topic.
 */

class ManagerButtonStatusComponent {
  public:
    ManagerButtonStatusComponent(std::shared_ptr<rclcpp::Node> subNode);
      void publish_button_status(const tod_status_msgs::msg::ManagerButtonStatus& msg);



  private:
    rclcpp::Publisher<tod_status_msgs::msg::ManagerButtonStatus>::SharedPtr _publisher;
};

}  // namespace tod_gl