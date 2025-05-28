/**
 * @file camera.cpp
 * @brief Camera Entity that just holds the camera component 
 * @copyright 2024 TUMFTM
 **/

#include "tod_static_entities/camera.hpp"

#include "tod_gl/scene/components.hpp"

#include "tod_vehicle_msgs/VehicleEnums.h"

namespace TodStaticEntities {

tod_gl::Entity Camera::create(std::shared_ptr<tod_gl::Scene> scene, std::string name, tod_gl::Entity parent) {
    tod_gl::Entity camera = scene->create_entity(name);
    camera.add_component<tod_gl::CameraComponent>(
        glm::perspective(glm::radians(45.0f), (float)1280 / (float)720, 0.1f, 50.0f), true);
    camera.get_component<tod_gl::TransformComponent>().set_parent(parent);
    return camera;
}

void Camera::onGearUpdate(const tod_vehicle_msgs::msg::SecondaryVehicleState::ConstSharedPtr &msg, tod_gl::Entity &entity) {
    static eGearPosition previousGear{eGearPosition::GEARPOSITION_PARK};
    if (msg->gear_position == eGearPosition::GEARPOSITION_REVERSE &&
        previousGear != eGearPosition::GEARPOSITION_REVERSE) {
        entity.get_component<tod_gl::CameraComponent>().yaw = 180.0f;
        entity.get_component<tod_gl::CameraComponent>().radius = 0.0f;
    }
    if (msg->gear_position != eGearPosition::GEARPOSITION_REVERSE &&
        previousGear == eGearPosition::GEARPOSITION_REVERSE) {
        entity.get_component<tod_gl::CameraComponent>().yaw = 0.0f;
        entity.get_component<tod_gl::CameraComponent>().radius = 1.5f;
    }
    previousGear = static_cast<eGearPosition>(msg->gear_position);
}

};  // namespace TodStaticEntities
