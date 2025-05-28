/**
 * @file cursor_position.cpp
 * @brief TODO: Add brief
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/core/cursor_position.hpp"

#include "tod_gl/systems/transform_system.hpp"

namespace tod_gl {
geometry_msgs::msg::PointStamped CursorPosition::get_real_world_coordinates(
    const glm::mat4& modelViewMatrix, const glm::mat4& projectionMatrix, float mouseX, float mouseY,
    unsigned int viewportWidth, unsigned int viewportHeight, const TransformComponent& floor_transform) {
    geometry_msgs::msg::PointStamped tmpMousePosition;

    double xNDC = (2.0f * mouseX) / viewportWidth - 1.0f;
    double yNDC = 1.0f - (2.0f * mouseY) / viewportHeight;

    glm::mat4 invVP = glm::inverse(projectionMatrix * modelViewMatrix);
    glm::vec4 nearPoint = invVP * glm::vec4(xNDC, yNDC, -1.0f, 1.0f);
    glm::vec4 farPoint = invVP * glm::vec4(xNDC, yNDC, 1.0f, 1.0f);

    nearPoint /= nearPoint.w;
    farPoint /= farPoint.w;

    glm::vec3 rayDir = glm::normalize(glm::vec3(farPoint - nearPoint));

    auto floorPoint = floor_transform.translation;
    auto floorNormal = glm::normalize(glm::mat3(floor_transform.get_transform()) * glm::vec3(0.0f, 0.0f, 1.0f));

    // glm::vec3 floorNormal(0.0f, 0.0f, 1.0f);
    // glm::vec3 floorPoint(0.0f, 0.0f, 0.0f);

    float t = glm::dot(floorPoint - glm::vec3(nearPoint), floorNormal) / glm::dot(rayDir, floorNormal);
    glm::vec3 intersectionPoint = glm::vec3(nearPoint) + t * rayDir;

    // Transform Cursor Positoin Into World Space
    intersectionPoint = tod_gl::TransformSystem::get_instance()->to_world_coordinates(intersectionPoint);

    tmpMousePosition.point.x = intersectionPoint.x;
    tmpMousePosition.point.y = intersectionPoint.y;
    tmpMousePosition.point.z = 0.0;
    return tmpMousePosition;
}

bool CursorPosition::intersect_floor(const glm::vec3& rayOrigin, const glm::vec3& rayDirection,
                                    glm::vec3& intersectionPoint) {
    glm::vec3 planeNormal = glm::vec3(0, 0, 0.1);
    float planeD = 0;
    float denom = glm::dot(planeNormal, rayDirection);
    if (std::abs(denom) > 0.0001f) {
        float t = -(glm::dot(planeNormal, rayOrigin) + planeD) / denom;
        if (t >= 0) {
            intersectionPoint = rayOrigin + t * rayDirection;
            return true;
        }
    }
    return false;
}

GLint CursorPosition::convert_y_pixel_coord_from_GLFW_to_openGL(const unsigned int window_height, const float yPosInPixelGLFW) {
    GLint yPosInOpenGLConvention = (GLint)(window_height - 1 - yPosInPixelGLFW);
    return yPosInOpenGLConvention;
}

bool CursorPosition::cursor_hovered_over_object(const GLfloat& depth) {
    return (depth > 0.0f && depth < 1.0f);
}

void CursorPosition::convert_mat_4_to_gl_double_array(const glm::mat4& glmMat, GLdouble array[16]) {
    const float* pSource = (const float*)glm::value_ptr(glmMat);
    for (int i = 0; i < 16; ++i) {
        array[i] = pSource[i];
    }
}

void CursorPosition::print_tmp_mouse_position_for_debugging(const MouseMovedEvent& mwe,
                                                       const geometry_msgs::msg::PointStamped& tmpMousePosition) {
    printf("mwe_x: %f mwe_y: %f,  wh: %i xposP: %f yposP: %f x: %f y: %f z: %f\n",

           mwe.get_x_position(), mwe.get_y_position(), mwe.window_height, mwe.x_pos_in_pixel, mwe.y_pos_in_pixel, tmpMousePosition.point.x,
           tmpMousePosition.point.y, tmpMousePosition.point.z);
}
}  // namespace tod_gl