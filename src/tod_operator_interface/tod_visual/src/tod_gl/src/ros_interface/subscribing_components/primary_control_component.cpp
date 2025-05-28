/**
 * @file primary_control_component.cpp
 * @brief PrimaryControlCommandComponent component that manages the subscription and the data for PrimaryControl topics for visualization for the operator 
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/subscribing_components/primary_control_component.hpp"

namespace tod_gl {

void PrimaryControlCommandComponent::cb_message(const tod_vehicle_msgs::msg::PrimaryControlCmd::SharedPtr msg){
    steering_wheel_angle_ = msg->steering_wheel_angle;
    steering_tire_angle_ = msg->steering_tire_angle;
    velocity_ = msg->velocity;
    acceleration_ = msg->acceleration;
}

} // namespace tod_gl