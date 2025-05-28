/**
 * @file WheelController.cpp
 * @brief Turns the wheels while driving for the immersion
 * @copyright 2024 TUMFTM
**/

#include "tod_dynamic_entities/wheel_controller.hpp"

#include "tod_gl/ros_interface/subscribing_components/primary_vehicle_state_component.hpp"
#include "tod_gl/scene/components.hpp"

namespace TodDynamicEntities {

void WheelController::on_update(float ts) 
{
    auto entity = this->get_bounded_scene().find_entity_with_tag("SubscriptionManager");
    if (!entity.has_component<tod_gl::PrimaryVehicleStateComponent>()) {
        return;
    }
    auto &primary_vehicle_state = entity.get_component<tod_gl::PrimaryVehicleStateComponent>();
    this->get_component<tod_gl::TransformComponent>().rotation.y += primary_vehicle_state.get_velocity() * ts * _turning_speed;
}

void FrontWheelController::on_update(float ts) 
{
        WheelController::on_update(ts);
        auto entity = this->get_bounded_scene().find_entity_with_tag("SubscriptionManager");
        if (!entity.has_component<tod_gl::PrimaryVehicleStateComponent>()) {
            return;
        }
        auto &primary_vehicle_state = entity.get_component<tod_gl::PrimaryVehicleStateComponent>();
        this->get_component<tod_gl::TransformComponent>().rotation.z = primary_vehicle_state.get_tire_wheel_angle();
}

}  // namespace TodDynamicEntities