/**
 * @file transform_system.cpp
 * @brief Transformation system for the scene's entities using the @ref TransformComponent transform information
 * @copyright 2024 TUMFTM
 **/

#include "tod_gl/systems/transform_system.hpp"

#include "entt/entt.hpp"

namespace tod_gl {

glm::mat4 TransformSystem::local_to_world( TransformComponent &transform) {
    glm::mat4 localToWorld;
    if (transform.parent_entity.get_handle() == entt::null) {
        localToWorld = local_to_parent(transform);
    } else {
         auto &parentTransform = transform.parent_entity.get_component<TransformComponent>();
        localToWorld = local_to_world(parentTransform) * local_to_parent(transform);
    }
    return localToWorld;
}

glm::mat4 TransformSystem::local_to_parent(TransformComponent &transform) {
    glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), transform.rotation.z, {0, 0, 1}) *
                         glm::rotate(glm::mat4(1.0f), transform.rotation.y, {0, 1, 0}) *
                         glm::rotate(glm::mat4(1.0f), transform.rotation.x, {1, 0, 0});

    return glm::translate(glm::mat4(1.0f), transform.translation) * rotation *
           glm::scale(glm::mat4(1.0f), transform.scale);
}

glm::mat4 TransformSystem::local_to_world_rotation(TransformComponent &transform) {
    if (transform.parent_entity.get_handle() == entt::null) {
        return local_to_parent_rotation(transform);
    } else {
        return local_to_world_rotation(transform.parent_entity.get_component<TransformComponent>()) *
               local_to_parent_rotation(transform);
    }
}

glm::mat4 TransformSystem::local_to_parent_rotation(TransformComponent &transform) {
    glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), transform.rotation.x, {1, 0, 0}) *
                         glm::rotate(glm::mat4(1.0f), transform.rotation.y, {0, 1, 0}) *
                         glm::rotate(glm::mat4(1.0f), transform.rotation.z, {0, 0, 1});

    return rotation;
}
// From a to be
glm::mat4 TransformSystem::get_transform_between_entities( TransformComponent& source, TransformComponent& target) {


    glm::mat4 t_source_world = TransformSystem::local_to_world(source);
    glm::mat4 t_target_world = TransformSystem::local_to_world(target);

    return glm::inverse(t_target_world) * t_source_world;
}


glm::vec3 TransformSystem::extract_rotation_euler( glm::mat4 &transform) {

    glm::mat3 rotMatrix = glm::mat3(transform);
    glm::quat rotationQuat = glm::quat_cast(rotMatrix);
    return glm::eulerAngles(rotationQuat);
}

glm::vec3 TransformSystem::extract_translation( glm::mat4 &transform) {
    return glm::vec3(transform[3]);
}

TransformSystem* TransformSystem::instance_ = nullptr;

} // namespace tod_gl