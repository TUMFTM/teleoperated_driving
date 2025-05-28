/**
 * @file camera_system.cpp
 * @brief Helpers for the scene's camera 
 * @copyright 2024 TUMFTM
 **/

#include "tod_gl/systems/camera_system.hpp"

#include "tod_gl/systems/transform_system.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"

namespace tod_gl {

void CameraSystem::calc_view_matrix(CameraComponent& camera, TransformComponent& transform) {
    // TODO: (Simon): only call this function [CameraController::updatePosition(camera)] on user
    // input (it does only need an update, when the
    // relative position between vehicle and camera is changed AND
    // on application start)
    glm::mat4 CameraTransform = tod_gl::TransformSystem::get_instance()->local_to_world(transform);
    glm::mat4 CameraRotation = tod_gl::TransformSystem::get_instance()->local_to_world_rotation(transform);
    if (camera.rotate == true) {
        update_position_from_look_at_and_radius(camera);
    }
    glm::vec3 viewPosition = glm::vec3(CameraTransform * glm::vec4(camera.position, 1.0));
    glm::vec3 viewlookAt = glm::vec3(CameraTransform * glm::vec4(camera.lookat, 1.0));
    glm::vec3 viewUp = glm::vec3(CameraRotation * glm::vec4(camera.up, 1.0));  // only rotation required
    camera.view = glm::lookAt(viewPosition, viewlookAt, viewUp);
}

void CameraSystem::on_window_size_changed(CameraComponent& camera, int width, int height) {
    camera.projection = glm::perspective(glm::radians(camera.field_of_view), (float)width / (float)height,
                                        camera.near_plane, camera.far_plane);
}

void CameraSystem::update_position_from_look_at_and_radius(CameraComponent& camera) {
    camera.position.x = camera.lookat.x - (camera.radius + camera.lookat.x) * glm::cos(glm::radians(camera.yaw));
    camera.position.y = (camera.radius + camera.lookat.x) * glm::sin(glm::radians(camera.yaw));
    glm::vec3 cameraLookDirection = glm::normalize(camera.position - camera.lookat);
    glm::vec3 worldUp{0.0f, 0.0f, 1.0f};
    glm::vec3 cameraRight = glm::normalize(glm::cross(worldUp, cameraLookDirection));
    camera.up = glm::normalize(glm::cross(cameraLookDirection, cameraRight));
}

} // namespace tod_gl