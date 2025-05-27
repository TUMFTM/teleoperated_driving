/**
 * @file camera_controller.cpp
 * @brief Camera controller for the camera entity, contains options for UI based click / key based camera movement, bird's eye view and return to vehicle functions
 * @copyright 2024 TUMFTM
 **/

#include <cmath>
#include <glm/fwd.hpp>

#include "tod_gl/scene/camera_controller.hpp"



namespace tod_gl {


void CameraController::on_event(Event &e, CameraComponent &camera,
                               float deltaTime) {
  EventDispatcher dispatcher(e);
  auto &cursorPosition = CursorPosition::get();
  auto &keyManager = KeyStateManager::get_instance();

  if (!cursorPosition.isOverViewport) {
    on_cursor_leave_viewport();
    return;
  }

  if (!_target_position_initialized) {
    target_position = camera.position;
    target_lookat = camera.lookat;
    _target_position_initialized = true;
  }

  if (std::isnan(target_position.x) || std::isnan(target_position.y) ||
      std::isnan(target_position.z) || std::isnan(target_lookat.x) ||
      std::isnan(target_lookat.y) || std::isnan(target_lookat.z)) {
    target_position = camera.initial.position;
    target_lookat = camera.initial.lookat;
    camera.position = camera.initial.position;
    camera.lookat = camera.initial.lookat;
    camera.up = camera.initial.up;
  }

  if (e.get_event_type() == EventType::KeyPressed) {
    auto &keyEvent = static_cast<KeyPressedEvent &>(e);
    KeyCode keyCode = keyEvent.get_key_code();
    keyManager.set_key_state(keyCode, true);

    switch (keyCode) {
    case KeyCode::Tab:
      if (!is_transitioning) {
        switch_view(camera);
      }
      break;
    case KeyCode::O:
      if (!is_transitioning) {
        back_to_car(camera);
      }
      break;
    case KeyCode::P:
      print_camera(camera);
      break;
    case KeyCode::U:
      switch_is_move_camera();
      break;
    default:
      break;
    }
    // } else {
    //     handleMovement(e, camera, deltaTime);    }
  } else if (e.get_event_type() == EventType::KeyReleased) {
    auto &keyEvent = static_cast<KeyReleasedEvent &>(e);
    keyManager.set_key_state(keyEvent.get_key_code(), false);
  }

  if (keyManager.is_any_movement_key_pressed()) {
    handle_continuous_movement(camera, deltaTime);
  }

  if (cursorPosition.isOverViewport) {
    auto mousePos = cursorPosition.get_mouse_position();
    auto viewports = cursorPosition.get_viewport_dimensions();

    if (e.get_event_type() == EventType::MouseButtonPressed) {
      auto &mousePressed = static_cast<MouseButtonPressedEvent &>(e);
      if (mousePressed.get_mouse_button() == MouseCode::ButtonRight &&
          isMoveCamera) {
        _is_right_mouse_pressed = true;
        _last_mouse_x = mousePos.first;
        _last_mouse_y = mousePos.second;

        _orbital_point_set = true;
        _current_view_mode = ViewMode::Normal;
      } else if (mousePressed.get_mouse_button() == MouseCode::ButtonLeft &&
          isMoveCamera) {
        _is_left_mouse_pressed = true;
        _last_mouse_x = mousePos.first;
        _last_mouse_y = mousePos.second;
      } else if (mousePressed.get_mouse_button() == MouseCode::ButtonMiddle &&
          isMoveCamera) {
        _is_middle_mouse_pressed = true;
        _last_mouse_x = mousePos.first;
        _last_mouse_y = mousePos.second;
      }


    } else if (e.get_event_type() == EventType::MouseButtonReleased) {
      auto &mouseReleased = static_cast<MouseButtonReleasedEvent &>(e);
      if (mouseReleased.get_mouse_button() == MouseCode::ButtonRight) {
        _is_right_mouse_pressed = false;
        _orbital_point_set = false;
      }
      if (mouseReleased.get_mouse_button() == MouseCode::ButtonLeft) {
        _is_left_mouse_pressed = false;
      }
      if (mouseReleased.get_mouse_button() == MouseCode::ButtonMiddle) {
        _is_middle_mouse_pressed = false;
      }
    }

    if (e.get_event_type() == EventType::MouseMoved && isMoveCamera) {
      if (_is_left_mouse_pressed && keyManager.is_key_pressed(KeyCode::LeftShift) && _current_view_mode == ViewMode::Normal)  {
        handle_forward_drag(glm::vec2(mousePos.first, mousePos.second), camera,
                      deltaTime);
      } else if (_is_left_mouse_pressed) {
        if (_current_view_mode == ViewMode::TopView) {
          handle_top_down_translation(glm::vec2(mousePos.first, mousePos.second), camera,
                      deltaTime);
        } else {
          handle_panning(glm::vec2(mousePos.first, mousePos.second), camera,
                      deltaTime);
        }
           
      } else if (_is_right_mouse_pressed) {
        handle_orbital_rotation(glm::vec2(mousePos.first, mousePos.second),
                              camera, deltaTime);
      } else if (_is_middle_mouse_pressed) {
        handle_orbital_point_move(glm::vec2(mousePos.first, mousePos.second),
                              camera, deltaTime);
      }
    }

    if (e.get_event_type() == EventType::MouseScrolled) {
      auto &mouseScroll = static_cast<MouseScrolledEvent &>(e);
      auto scrollAmount = static_cast<float>(mouseScroll.get_y_offset());

      glm::vec3 direction = glm::normalize(camera.lookat - camera.position);
      float zoomAmount = scrollAmount * _zoom_sensitivity;
      float currentDistance = glm::length(target_position - target_lookat);
      float newDistance = currentDistance - zoomAmount;

      if (newDistance > 0.1f && newDistance < 100.0f) {
        glm::vec3 newPosition = target_position + direction * zoomAmount;
        if (newPosition.z >= 0.1f) {
          target_position = newPosition;
          is_transitioning = true;
        }
      }
    }
  } else {
    _is_right_mouse_pressed = false;
    _is_left_mouse_pressed = false;
    _orbital_point_set = false;
  }

  target_position = glm::vec3(
      std::isnan(target_position.x) ? camera.position.x : target_position.x,
      std::isnan(target_position.y) ? camera.position.y : target_position.y,
      std::isnan(target_position.z) ? camera.position.z : target_position.z);

  target_lookat =
      glm::vec3(std::isnan(target_lookat.x) ? camera.lookat.x : target_lookat.x,
                std::isnan(target_lookat.y) ? camera.lookat.y : target_lookat.y,
                std::isnan(target_lookat.z) ? camera.lookat.z : target_lookat.z);
}

void CameraController::transition(CameraComponent &camera, float deltaTime) {
  if (!is_transitioning) {
    return;
  }

  camera.position = glm::mix(camera.position, target_position,
                             1.0f - std::exp(-_position_lerp_speed * deltaTime));
  camera.lookat = glm::mix(camera.lookat, target_lookat,
                           1.0f - std::exp(-_lookat_lerp_speed * deltaTime));

  float positionDist = glm::length(target_position - camera.position);
  float lookAtDist = glm::length(target_lookat - camera.lookat);

  if (positionDist < EPSILON && lookAtDist < EPSILON) {
    is_transitioning = false;
    camera.position = target_position;
    camera.lookat = target_lookat;
  }
  camera.view = glm::lookAt(camera.position, camera.lookat, camera.up);
}

void CameraController::handle_continuous_movement(CameraComponent &camera,
                                                float deltaTime) {
  auto &keyManager = KeyStateManager::get_instance();

  float cameraSpeed = 3.0f * deltaTime; // Increase for faster movement

  glm::vec3 Front = glm::normalize(camera.lookat - camera.position);
  glm::vec3 Right = glm::normalize(glm::cross(Front, camera.up));
  glm::vec3 Up = camera.up;
  auto Forward = glm::vec3(1.0f, 0.0f, 0.0f);
  glm::vec3 newTargetPosition = target_position;
  glm::vec3 newTargetLookAt = target_lookat;

  if (keyManager.is_key_pressed(KeyCode::Up)) {
    move_up(deltaTime);
  }
  if (keyManager.is_key_pressed(KeyCode::Down)) {
    move_down(deltaTime);
  }
  if (keyManager.is_key_pressed(KeyCode::Left)) {
    move_left(camera, deltaTime);
  }
  if (keyManager.is_key_pressed(KeyCode::Right)) {
    move_right(camera, deltaTime);
  }

  if (keyManager.is_key_pressed(KeyCode::I)) {
    pitch_forward(camera, deltaTime);
  }
  if (keyManager.is_key_pressed(KeyCode::K)) {
    pitch_backward(camera, deltaTime);
  }

  if (keyManager.is_key_pressed(KeyCode::PageUp)) {
    float newZ = target_position.z + Up.z * cameraSpeed;
    if (newZ <= 100.f) {
      target_position += Up * cameraSpeed;
      target_lookat += Up * cameraSpeed;
      is_transitioning = true;
    }
  }
  if (keyManager.is_key_pressed(KeyCode::PageDown)) {
    float newZ = target_position.z - Up.z * cameraSpeed;
    if (newZ >= 0.1f) {
      target_position -= Up * cameraSpeed;
      target_lookat -= Up * cameraSpeed;
      is_transitioning = true;
    }
  }

  if (keyManager.is_key_pressed(KeyCode::J)) {
    rotate_camera_lookat(deltaTime, camera, 1.0f);
    is_transitioning = true;
  }
  if (keyManager.is_key_pressed(KeyCode::L)) {
    rotate_camera_lookat(deltaTime, camera, -1.0f);
    is_transitioning = true;
  }
  if (keyManager.is_key_pressed(KeyCode::N)) {
    rotate_camera_translation(22.5f, camera, 1.0f);
    is_transitioning = true;
  }
  if (keyManager.is_key_pressed(KeyCode::M)) {
    rotate_camera_translation(22.5f, camera, -1.0);
    is_transitioning = true;
  }
}

void CameraController::pitch_forward(CameraComponent &camera, float timestamp) {
  float pitchSpeed = 1.f * timestamp;

  glm::vec3 direction = glm::normalize(camera.lookat - camera.position);
  glm::vec3 right = glm::normalize(glm::cross(direction, camera.up));

  glm::mat4 pitchMatrix = glm::rotate(glm::mat4(1.0f), pitchSpeed, right);
  glm::vec3 newDirection = glm::vec3(pitchMatrix * glm::vec4(direction, 0.f));

  float radius = glm::length(camera.lookat - camera.position);
  target_lookat = camera.position + (newDirection * radius);
  is_transitioning = true;
}

void CameraController::pitch_backward(CameraComponent &camera, float timestamp) {
  float pitchSpeed = -1.f * timestamp;

  glm::vec3 direction = glm::normalize(camera.lookat - camera.position);
  glm::vec3 right = glm::normalize(glm::cross(direction, camera.up));

  glm::mat4 pitchMatrix = glm::rotate(glm::mat4(1.0f), pitchSpeed, right);
  glm::vec3 newDirection = glm::vec3(pitchMatrix * glm::vec4(direction, 0.f));

  float radius = glm::length(camera.lookat - camera.position);
  target_lookat = camera.position + (newDirection * radius);
  is_transitioning = true;
}

void CameraController::move_down(float timestamp) {
  float cameraSpeed = 3.f * timestamp;
  auto Forward = glm::vec3(1.0f, 0.0f, 0.0f);
  target_position -= Forward * cameraSpeed;
  target_lookat -= Forward * cameraSpeed;
  is_transitioning = true;
}

void CameraController::move_up(float timestamp) {
  float cameraSpeed = 3.f * timestamp;
  auto Forward = glm::vec3(1.0f, 0.0f, 0.0f);
  target_position += Forward * cameraSpeed;
  target_lookat += Forward * cameraSpeed;
  is_transitioning = true;
}

void CameraController::move_left(CameraComponent &camera, float timestamp) {
  glm::vec3 Front = glm::normalize(camera.lookat - camera.position);
  glm::vec3 Right = glm::normalize(glm::cross(Front, camera.up));

  float cameraSpeed = 3.f * timestamp;

  target_position -= Right * cameraSpeed;
  target_lookat -= Right * cameraSpeed;
  is_transitioning = true;
}

void CameraController::move_right(CameraComponent &camera, float timestamp) {
  glm::vec3 Front = glm::normalize(camera.lookat - camera.position);
  glm::vec3 Right = glm::normalize(glm::cross(Front, camera.up));

  float cameraSpeed = 3.f * timestamp;
  target_position += Right * cameraSpeed;
  target_lookat += Right * cameraSpeed;
  is_transitioning = true;
}

void CameraController::handle_panning(const glm::vec2 &mousePos,
                                     CameraComponent &camera, float deltaTime) {
  float deltaX = mousePos.x - _last_mouse_x;
  float deltaY = mousePos.y - _last_mouse_y;
  _last_mouse_x = mousePos.x;
  _last_mouse_y = mousePos.y;

  float movement_threshold = 2.0f;
  if (std::abs(deltaX) < movement_threshold &&
      std::abs(deltaY) < movement_threshold) {
    return;
  }

  float distance = glm::length(camera.position - camera.lookat);
  float adjustedPanSpeed = _pan_speed * distance * 0.001f;

  glm::vec3 Front = glm::normalize(camera.lookat - camera.position);
  glm::vec3 Right = glm::normalize(glm::cross(Front, camera.up));
  glm::vec3 Up = camera.up;

  glm::vec3 panOffset = (-Right * deltaX + Up * deltaY) * adjustedPanSpeed;

  if (!std::isnan(panOffset.x) && !std::isnan(panOffset.y) &&
      !std::isnan(panOffset.z)) {
    target_position += panOffset;
    target_lookat += panOffset;

    target_position.z = glm::max(target_position.z, 0.15f);
    target_lookat.z = glm::max(target_lookat.z, 0.15f);

    is_transitioning = true;
  }
}




void CameraController::handle_orbital_rotation(const glm::vec2 &mousePos,
                                             CameraComponent &camera,
                                             float deltaTime) {
  if (!_orbital_point_set)
    return;

  float deltaX = mousePos.x - _last_mouse_x;
  float deltaY = mousePos.y - _last_mouse_y;
  _last_mouse_x = mousePos.x;
  _last_mouse_y = mousePos.y;

  glm::vec3 direction = camera.position - camera.orbit_point.position;
  float radius = glm::length(direction);
  if (radius < EPSILON) 
      return;

  float currentPitch = glm::degrees(std::asin(direction.z / radius));
  float currentYaw = glm::degrees(std::atan2(direction.y, direction.x));

  float newPitch = glm::clamp(currentPitch - deltaY, 10.f, MAX_PITCH);
  float newYaw = currentYaw + deltaX;

  float pitchRad = glm::radians(newPitch);
  float yawRad = glm::radians(newYaw);

  glm::vec3 newPosition;
  newPosition.x = camera.orbit_point.position.x + camera.orbit_point.orbit_distance * std::cos(pitchRad) * std::cos(yawRad);
  newPosition.y = camera.orbit_point.position.y + camera.orbit_point.orbit_distance * std::cos(pitchRad) * std::sin(yawRad);
  newPosition.z = camera.orbit_point.position.z + camera.orbit_point.orbit_distance * std::sin(pitchRad);

  newPosition.z = glm::max(newPosition.z, 0.1f);

  if (!std::isnan(newPosition.x) && !std::isnan(newPosition.y) &&
      !std::isnan(newPosition.z)) {
    camera.position = newPosition;

    camera.lookat = camera.orbit_point.position;
    target_position = newPosition;
    target_lookat = camera.orbit_point.position;
    //     is_transitioning = true;
  }

  // camera.up = glm::normalize(glm::cross(right, direction));
}


void CameraController::handle_top_down_translation(const glm::vec2 &mousePos, CameraComponent &camera, float deltaTime) {
  float deltaX = mousePos.x - _last_mouse_x;
  float deltaY = mousePos.y - _last_mouse_y;
  _last_mouse_x = mousePos.x;
  _last_mouse_y = mousePos.y;

  float translationFactor = _pan_speed * deltaTime;
  glm::vec3 translation(deltaY * translationFactor, deltaX * translationFactor, 0.0f);
  target_position += translation;
  target_lookat += translation;
  is_transitioning = true;
}

void CameraController::handle_forward_drag(const glm::vec2 &mousePos, CameraComponent &camera, float deltaTime) {
  float deltaX = mousePos.x - _last_mouse_x;
  float deltaY = mousePos.y - _last_mouse_y;
  _last_mouse_x = mousePos.x;
  _last_mouse_y = mousePos.y;

  float dragFactor = _move_speed * deltaTime;
  glm::vec3 forward = glm::normalize(camera.lookat - camera.position);
  glm::vec3 right = glm::normalize(glm::cross(forward, camera.up));
  
  glm::vec3 dragOffset = forward * (deltaY * dragFactor) + right * (deltaX * dragFactor);
  target_position += dragOffset;
  target_lookat += dragOffset;
  is_transitioning = true;
}

void CameraController::handle_orbital_point_move(const glm::vec2 &mousePos, CameraComponent &camera, float deltaTime) {
  float deltaX = mousePos.x - _last_mouse_x;
  float deltaY = mousePos.y - _last_mouse_y;
  _last_mouse_x = mousePos.x;
  _last_mouse_y = mousePos.y;

   auto movement_penalty_factor = 0.1f; // TODO: Make Constexpr
  camera.orbit_point.position.x = camera.orbit_point.position.x - deltaY * movement_penalty_factor;
  camera.orbit_point.position.y = camera.orbit_point.position.y - deltaX * movement_penalty_factor;
  handle_top_down_translation(mousePos, camera,deltaTime);
  is_transitioning = true;

}


void CameraController::print_camera(CameraComponent &camera) {
  std::cout << "Camera Position: " << glm::to_string(camera.position)
            << std::endl;
  std::cout << "Camera LookAt: " << glm::to_string(camera.lookat) << std::endl;
  std::cout << "Camera Up: " << glm::to_string(camera.up) << std::endl;
  std::cout << "Camera Yaw: " << camera.yaw << std::endl;
  std::cout << "Camera Radius: " << camera.radius << std::endl;
  std::cout << "Camera Projection: " << glm::to_string(camera.projection)
            << std::endl;
  std::cout << "Camera View: " << glm::to_string(camera.view) << std::endl;
}

void CameraController::rotate_camera_lookat(float deltaTime, CameraComponent &camera, float rotation_direction) {
  glm::vec3 direction = camera.position - camera.lookat;

  float angle = rotation_direction * glm::radians(90.0f) * deltaTime * 30;
  glm::mat3 rotation = glm::rotate(glm::mat4(1.0f), angle, camera.up);
  direction = rotation * direction;
  glm::vec3 newPosition = camera.lookat + direction;
  newPosition.z = camera.position.z;
  target_position = newPosition;
  is_transitioning = true;
}


void CameraController::rotate_camera_translation(float angleStepDegrees, CameraComponent &camera, float translation_direction) {
    glm::vec3 offset = (std::isfinite(target_position.x) && std::isfinite(target_lookat.x))
        ? (target_position - target_lookat)
        : (camera.position - camera.lookat);

    float distance = glm::length(offset);
    if (distance < 1e-6f)
        return; // Avoid dividing by near-zero lengths
    float angleRad = glm::radians(angleStepDegrees * translation_direction);
    glm::quat rotationQ = glm::angleAxis(angleRad, glm::normalize(camera.up)); 
    offset = rotationQ * offset;
    offset = glm::normalize(offset) * distance;
    target_position = camera.lookat + offset;
    target_lookat = camera.lookat;
        is_transitioning = true;
}


void CameraController::zoom_in(CameraComponent &camera, float timestamp) {
    float cameraSpeed = 3.f * timestamp;
    glm::vec3 Front = glm::normalize(camera.lookat - camera.position);

    target_position += Front * cameraSpeed;
    target_lookat += Front * cameraSpeed;
    is_transitioning = true;
}

void CameraController::zoom_out(CameraComponent &camera, float timestamp) {
    float cameraSpeed = 3.f * timestamp;
    glm::vec3 Front = glm::normalize(camera.lookat - camera.position);

    target_position -= Front * cameraSpeed;
    target_lookat -= Front * cameraSpeed;
    is_transitioning = true;
}

CameraController::ViewState CameraController::get_direct_control_state() const {
  return ViewState{
      glm::vec3(1.307014, 0.000000, 1.432), // position
      glm::vec3(10.652516, 0.000000, -0.131046),  // lookAt
      glm::vec3(0.000000, 0.000000, 1.000000),  // up
      0.0f,                                     // yaw
      1.5f                                      // radius
  };
}

CameraController::ViewState
CameraController::get_initial_state(const CameraComponent &camera) const {
  return ViewState{camera.initial.position, camera.initial.lookat,
                   camera.initial.up, camera.initial.yaw,
                   camera.initial.radius};
}

void CameraController::update_top_view_state(CameraComponent &camera) {
  _top_view_mode.position = glm::vec3(-4.18,  0.0, 47.0);
  _top_view_mode.lookat = glm::vec3(.0, 0, .0);
  _top_view_mode.up = camera.initial.up;
  _top_view_mode.yaw = camera.initial.yaw;
  _top_view_mode.radius = 1.5f;

  target_position = _top_view_mode.position;
  target_lookat = _top_view_mode.lookat;
  camera.up = _top_view_mode.up;
  camera.yaw = _top_view_mode.yaw;
  camera.radius = _top_view_mode.radius;
}

void CameraController::update_normal_view_state(CameraComponent &camera) {
  StateManager &stateManager = StateManager::get_instance();
  ViewState newState;

  if (stateManager.get_current_mode() ==
      tod_status_msgs::msg::Status::CONTROL_MODE_DIRECT) {
    newState = get_direct_control_state();
  } else {
    newState = get_initial_state(camera);
  }

  _normal_view_mode = newState;

  target_position = _normal_view_mode.position;
  target_lookat = _normal_view_mode.lookat;
  camera.up = _normal_view_mode.up;
  camera.yaw = _normal_view_mode.yaw;
  camera.radius = _normal_view_mode.radius;
}

void CameraController::switch_to_normal_view(CameraComponent &camera) {
  if (_current_view_mode == ViewMode::Normal)
    return;

  target_position = _normal_view_mode.position;
  target_lookat = _normal_view_mode.lookat;
  camera.up = _normal_view_mode.up;
  camera.yaw = _normal_view_mode.yaw;
  camera.radius = _normal_view_mode.radius;

  _current_view_mode = ViewMode::Normal;
  is_transitioning = true;
}

void CameraController::switch_to_top_view(CameraComponent &camera) {
  if (_current_view_mode == ViewMode::TopView)
    return;

  // Save current normal view state
  _normal_view_mode = ViewState{camera.position, camera.lookat, camera.up,
                              camera.yaw, camera.radius};

  // Switch to top view
  update_top_view_state(camera);
  _current_view_mode = ViewMode::TopView;
  is_transitioning = true;
}


void CameraController::back_to_car(CameraComponent &camera) {
  if (_current_view_mode == ViewMode::Normal) {
    update_normal_view_state(camera);
  } else {
    update_top_view_state(camera);
  }
  camera.orbit_point.position = glm::vec3(0.0f);

  is_transitioning = true;
}

void CameraController::switch_view(CameraComponent &camera) {
  if (_current_view_mode == ViewMode::Normal) {
    switch_to_top_view(camera);
  } else {
    switch_to_normal_view(camera);
  }
}

bool CameraController::get_top_view_on() {
  return _current_view_mode == ViewMode::TopView;
}

} // namespace tod_gl