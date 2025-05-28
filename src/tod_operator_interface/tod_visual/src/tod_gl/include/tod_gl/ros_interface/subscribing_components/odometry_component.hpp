/**
 * @file odometry_component.hpp
 * @brief holds the odometry information of the vehicle in the world - the initial position is offset such that the 
 * Scene only renders relative update to prevent transformation render artifacts
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "tod_gl/ros_interface/subscribing_component_base.hpp"

#include "rclcpp/rclcpp.hpp"
#include <glm/glm.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tod_gl/systems/transform_system.hpp"

#include "nav_msgs/msg/odometry.hpp"

namespace tod_gl {
  
class OdometryComponent : public SubscribingComponent<nav_msgs::msg::Odometry> {
  public:
    explicit OdometryComponent(std::shared_ptr<rclcpp::Node> sub_node)
      : SubscribingComponent(sub_node, "input/odom"),
      position_(0.f, 0.f, 0.f),
      orientation_(0.f, 0.f, 0.f, 1.f), // Init only with valid rotation otherwise transforms result in NaN
      linear_velocities_(0.f, 0.f, 0.f),
      angular_velocities_(0.f, 0.f, 0.f),
      has_received_data_(false)
    {}
    
    glm::vec3 get_position() const { return position_; }
    tf2::Quaternion get_orientation() const { return orientation_; }
    glm::vec3 get_linear_velocities() const { return linear_velocities_; }
    glm::vec3 get_angular_velocities() const { return angular_velocities_; }
    bool has_received_data() const { return has_received_data_; }

  private:
    void cb_message(const nav_msgs::msg::Odometry::SharedPtr msg) override;
    // Pose
    glm::vec3 position_;
    tf2::Quaternion orientation_;
    // Twist
    glm::vec3 linear_velocities_;
    glm::vec3 angular_velocities_;
    // Data state
    bool has_received_data_;
};

}  // namespace tod_gl

