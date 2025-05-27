/**
 * @file visual_mouse_moved_component.hpp
 * @ingroup tod_gl_ros_interface
 * @brief Component for managing button status publishing.
 *
 * This file contains the declaration of the VisualMousePressedComponent class, which provides functionality 
 * to publish button status messages for state machine.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"

namespace tod_gl {
  /**
 * @class VisualMousePressedComponent
 * @brief Handles publishing manager button status messages.
 *
 * The VisualMousePressedComponent provides an interface for publishing mouse moved messages 
 * of type `geometry_msgs::msg::PointStamped` to a ROS topic.
 */

class VisualMousePressedComponent {
  public:
    VisualMousePressedComponent(std::shared_ptr<rclcpp::Node> subNode);
      void publish_mouse_position(const geometry_msgs::msg::PointStamped& msg);

  private:
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr _publisher;
};

}  // namespace tod_gl