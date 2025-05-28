/**
 * @file primary_vehicle_state_component.cpp
 * @briefPrimaryVehicleState component that manages the subscription and the data for PrimaryControl topics for primary vehicle state coming from the vehicle 
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/subscribing_components/primary_vehicle_state_component.hpp"

namespace tod_gl {

void PrimaryVehicleStateComponent::cb_message(const tod_vehicle_msgs::msg::PrimaryVehicleState::SharedPtr msg){
    steering_wheel_angle_ = msg->steering_wheel_angle;
    steering_tire_angle_ = msg->steering_tire_angle;
    velocity_ = msg->velocity;
    acceleration_ = msg->acceleration;
}

} // namespace tod_gl