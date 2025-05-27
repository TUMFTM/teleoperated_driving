/**
 * @file primary_control_component.hpp
 * @brief PrimaryControlCommandComponent component that manages the subscription and the data for PrimaryControl topics for visualization for the operator 
 * @copyright 2024 TUMFTM
 **/
#pragma once

#include "tod_gl/ros_interface/subscribing_component_base.hpp"

#include "rclcpp/rclcpp.hpp"
#include <glm/glm.hpp>

#include "tod_vehicle_msgs/msg/primary_control_cmd.hpp"

namespace tod_gl {
  
class PrimaryControlCommandComponent : public SubscribingComponent<tod_vehicle_msgs::msg::PrimaryControlCmd> {
  public:
    explicit PrimaryControlCommandComponent(std::shared_ptr<rclcpp::Node> sub_node)
      : SubscribingComponent(sub_node, "input/primary_control_command"),
        steering_wheel_angle_(0.f),
        steering_tire_angle_(0.f),
        velocity_(0.f),
        acceleration_(0.f)
    {}

    float get_steering_wheel_angle() const { return steering_wheel_angle_; };
    float get_tire_wheel_angle() const { return steering_tire_angle_; };
    float get_velocity() const { return velocity_; };
    float get_acceleration() const { return acceleration_; };

  private:
    void cb_message(const tod_vehicle_msgs::msg::PrimaryControlCmd::SharedPtr msg) override;
    float steering_wheel_angle_;
    float steering_tire_angle_;
    float velocity_;
    float acceleration_; 
};

}