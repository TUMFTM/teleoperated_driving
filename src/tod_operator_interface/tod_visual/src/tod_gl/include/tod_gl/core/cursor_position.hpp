/**
 * @file cursor_position.hpp
 * @brief Manages the tracking and mapping of the cursor position within the 3D scene as well as the translation between scene and map space
 * @copyright 2024 TUMFTM
 */

#pragma once

#include <mutex>

#include "tod_gl/events/mouse_events.hpp"
#include "tod_gl/scene/components.hpp"

#include <glad/glad.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "GL/glu.h"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace tod_gl {

class CursorPosition {
  public:
    static CursorPosition& get() {
        static CursorPosition instance;
        return instance;
    }

    void set_mouse_position(float x, float y) {
        std::lock_guard<std::mutex> lock(_mutex);
        _mouse_x = x;
        _mouse_y = y;
    }

    void set_mouse_position(float x, float y, bool isValid) {
        std::lock_guard<std::mutex> lock(_mutex);
        _mouse_x = x;
        _mouse_y = y;
        _is_position_valid = isValid;
        if (isValid) {
            _last_valid_x = x;
            _last_valid_y = y;
        }
    }

    std::pair<float, float> get_mouse_position() {
        std::lock_guard<std::mutex> lock(_mutex);
        return {_mouse_x, _mouse_y};
    }

    std::pair<float, float> get_viewport_dimensions() {
        std::lock_guard<std::mutex> lock(_mutex);
        return {_viewport_width, _viewport_height};
    }

    bool is_position_valid() {
        std::lock_guard<std::mutex> lock(_mutex);
        return _is_position_valid;
    }

    bool isOverViewport = false;

    void set_viewport(float width, float height) {
        std::lock_guard<std::mutex> lock(_mutex);
        _viewport_width = width;
        _viewport_height = height;
    }

    static geometry_msgs::msg::PointStamped get_real_world_coordinates(const glm::mat4& modelViewMatrix,
                                                                    const glm::mat4& projectionMatrix, float _mouse_x,
                                                                    float _mouse_y, unsigned int _viewport_width,
                                                                    unsigned int _viewport_height,
                                                                    const TransformComponent& floor_transform);

  private:
    CursorPosition() = default;
    CursorPosition(const CursorPosition&) = delete;
    CursorPosition& operator=(const CursorPosition&) = delete;

    float _mouse_x = 0.0f;
    float _mouse_y = 0.0f;
    std::mutex _mutex;
    unsigned int _viewport_width = 0;
    unsigned int _viewport_height = 0;

    bool _is_position_valid = false;
    float _last_valid_x = 0.0f;
    float _last_valid_y = 0.0f;

    static void convert_mat_4_to_gl_double_array(const glm::mat4& glmMat, GLdouble array[16]);
    static bool cursor_hovered_over_object(const GLfloat& depth);
    static bool intersect_floor(const glm::vec3& rayOrigin, const glm::vec3& rayDirection, glm::vec3& intersectionPoint);
    static GLint convert_y_pixel_coord_from_GLFW_to_openGL(const unsigned int window_height, const float yPosInPixelGLFW);
    static void print_tmp_mouse_position_for_debugging(const MouseMovedEvent& mwe,
                                                  const geometry_msgs::msg::PointStamped& tmpClick);
};
}  // namespace tod_gl