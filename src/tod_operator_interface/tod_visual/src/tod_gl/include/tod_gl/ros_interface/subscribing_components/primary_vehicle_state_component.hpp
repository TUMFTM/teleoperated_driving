/**
 * @file primary_vehicle_state_component.hpp
 * @brief PrimaryVehicleState component that manages the subscription and the data for PrimaryControl topics for primary vehicle state coming from the vehicle 
 * @copyright TUMFTM
 **/

#pragma once

#include "tod_gl/ros_interface/subscribing_component_base.hpp"

#include "rclcpp/rclcpp.hpp"
#include <glm/glm.hpp>

#include "tod_vehicle_msgs/msg/primary_vehicle_state.hpp"

namespace tod_gl {
  
class PrimaryVehicleStateComponent : public SubscribingComponent<tod_vehicle_msgs::msg::PrimaryVehicleState> {
  public:
    explicit PrimaryVehicleStateComponent(std::shared_ptr<rclcpp::Node> sub_node)
      : SubscribingComponent(sub_node, "input/primary_vehicle_state"),
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
    void cb_message(const tod_vehicle_msgs::msg::PrimaryVehicleState::SharedPtr msg) override;
    float steering_wheel_angle_;
    float steering_tire_angle_;
    float velocity_;
    float acceleration_; 
};

}