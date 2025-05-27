/**
 * @file position_controller.cpp
 * @brief PositionController performs the odometry movement in the world for the transformation system @ref TransformationSystem
 * @copyright 2024 TUMFTM 
 **/

// Copyright 2023 TUMFTM
#include "tod_dynamic_entities/position_controller.hpp"

#include "tod_gl/ros_interface/subscribing_components/odometry_component.hpp"
#include "tod_gl/scene/components.hpp"
#include "tod_gl/systems/transform_system.hpp"

namespace TodDynamicEntities {

void PositionController::on_update(float ts) 
{
    auto entity = this->get_bounded_scene().find_entity_with_tag("SubscriptionManager");
    if (!entity.has_component<tod_gl::OdometryComponent>()) {
        return;
    }
    auto &odometry = entity.get_component<tod_gl::OdometryComponent>();
    glm::vec3 position = odometry.get_position();
    tf2::Quaternion orientation = odometry.get_orientation();
    
    if(odometry.has_received_data() && !tod_gl::TransformSystem::get_instance()->is_world_offset_set()){
        std::cout << "World to Game offset set to " << std::to_string(position.x) << "  " << std::to_string(position.y) << std::endl;
        tod_gl::TransformSystem::get_instance()->set_world_offset(position);
    }

    if(odometry.has_received_data()){
        // Set translation
        glm::vec3 gamePosition = tod_gl::TransformSystem::get_instance()->to_game_coordinates(position);
        this->get_component<tod_gl::TransformComponent>().set_translation(
        glm::vec3(gamePosition.x, gamePosition.y, 0.0f));
    }
    else{
        this->get_component<tod_gl::TransformComponent>().set_translation(
        glm::vec3(0.0f, 0.0f, 0.0f));
    }
    
    if(odometry.has_received_data()){
        // Set rotation
        double r, p, y;
        tf2::Matrix3x3(orientation).getRPY(r, p, y);
        this->get_component<tod_gl::TransformComponent>().set_rotation(glm::vec3(0.0f, 0.0f, y));
    }
    else{
        this->get_component<tod_gl::TransformComponent>().set_rotation(glm::vec3(0.0f, 0.0f, 0.0f));
    }
}

} //namespace TodDynamicEntities