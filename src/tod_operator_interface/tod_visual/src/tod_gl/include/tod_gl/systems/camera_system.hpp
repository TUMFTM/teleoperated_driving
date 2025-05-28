/**
 * @file camera_system.hpp
 * @brief Helpers for the scene's camera 
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "tod_gl/scene/components.hpp"

namespace tod_gl {

class CameraSystem {
  public:
    ~CameraSystem() = default;
    static void calc_view_matrix(CameraComponent& camera, TransformComponent& transform);
    static void on_window_size_changed(CameraComponent& camera, int width, int height);

  private:
    CameraSystem();
    static void update_position_from_look_at_and_radius(CameraComponent& camera);
};

} // namespace tod_gl