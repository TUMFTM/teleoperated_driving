/**
 * @file camera_controller.hpp
 * @brief Camera controller for the camera entity, contains options for UI based click / key based camera movement, bird's eye view and return to vehicle functions
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "glm/gtx/string_cast.hpp"

#include "tod_gl/scene/components.hpp"
#include "tod_gl/events/event.hpp"
#include "tod_gl/events/key_codes.hpp"
#include "tod_gl/events/key_event.hpp"
#include "tod_gl/events/key_codes.hpp"
#include "tod_gl/events/key_event.hpp"
#include "tod_gl/events/mouse_codes.hpp"
#include "tod_gl/events/mouse_events.hpp"
#include "tod_gl/core/state_manager.hpp"
#include "tod_gl/core/cursor_position.hpp"

namespace tod_gl {

/**
 * @brief Manages the key states of the keys that are used to control the scene camera and to handle continuous movement
 **/
class KeyStateManager {
public:
  static KeyStateManager &get_instance() {
    static KeyStateManager instance;
    return instance;
  }

  void set_key_state(KeyCode key, bool pressed) { keyStates[key] = pressed; }

  bool is_key_pressed(KeyCode key) const {
    auto it = keyStates.find(key);
    return it != keyStates.end() && it->second;
  }

  void clear_all_states() { keyStates.clear(); }

  void handle_key_press(KeyPressedEvent &e) { set_key_state(e.get_key_code(), true); }

  void handle_key_release(KeyReleasedEvent &e) {
    set_key_state(e.get_key_code(), false);
  }

  bool is_any_movement_key_pressed() const {
    return is_key_pressed(KeyCode::Up) || is_key_pressed(KeyCode::Down) ||
           is_key_pressed(KeyCode::Left) || is_key_pressed(KeyCode::Right) ||
           is_key_pressed(KeyCode::PageUp) || is_key_pressed(KeyCode::PageDown) ||
           is_key_pressed(KeyCode::I) || is_key_pressed(KeyCode::K) ||
           is_key_pressed(KeyCode::J) || is_key_pressed(KeyCode::L) ||
           is_key_pressed(KeyCode::N) || is_key_pressed(KeyCode::M);
  }

private:
  KeyStateManager() = default;
  std::unordered_map<KeyCode, bool> keyStates;
};

/**
 * @brief The camera controller modifies the 3D scenes camera in a transitory system defining current position and lookat as well as target position and lookat which is transitioned to between the speed based on the lerp speed

 * The CameraController Supports, 2D translation, 3D translation via panning and orbital roation around a controllable orbit point both with mouse and key commands
 * Based on the tod control mode different camera movements are enabled e.g. first person in direct mode or bird's eye view in trajectory guidance
 **/
class CameraController {
public:
  static CameraController &get_instance() {
    static CameraController instance;
    return instance;
  }
  void on_event(Event &e, CameraComponent &camera, float delta_time);

  float get_mouse_pressed() const { return _is_left_mouse_pressed; }
  float get_mouse_moved() const { return is_mouse_moved; }
  bool get_is_move_camera() const { return isMoveCamera; }

  void switch_is_move_camera() { isMoveCamera = get_is_move_camera() ? false : true; }
  void handle_continuous_movement(CameraComponent &camera, float deltaTime);

  void switch_view(CameraComponent &camera);
  void back_to_car(CameraComponent &camera);
  bool get_top_view_on();
  void rotate_camera_lookat(float deltaTime, CameraComponent &camera, float rotation_direction);
  void rotate_camera_translation(float angleStepDegrees, CameraComponent &camera, float translation_direction);
  void move_up(float deltaTime);
  void move_down(float deltaTime);
  void move_right(CameraComponent &camera, float deltaTime);
  void move_left(CameraComponent &camera, float deltaTime);
  void print_camera(CameraComponent &camera);
  bool isMoveCamera = true;
  void pitch_forward(CameraComponent &camera, float deltaTime);
  void pitch_backward(CameraComponent &camera, float deltaTime);

  void zoom_in(CameraComponent &camera, float timestamp);
  void zoom_out(CameraComponent &camera, float timestamp);

  void handle_top_down_translation(const glm::vec2 &mousePos, CameraComponent &camera, float deltaTime);
  void handle_forward_drag(const glm::vec2 &mousePos, CameraComponent &camera, float deltaTime);
  void on_cursor_leave_viewport() {
    _is_right_mouse_pressed = false;
    _is_left_mouse_pressed = false;
    _orbital_point_set = false;
  }

  enum class ViewMode { Normal, TopView };

  struct ViewState {
    glm::vec3 position;
    glm::vec3 lookat;
    glm::vec3 up;
    float yaw;
    float radius;
  };
  void switch_to_top_view(CameraComponent &camera);
  void switch_to_normal_view(CameraComponent &camera);
  void transition(CameraComponent &camera, float deltaTime);

  bool is_transitioning = false;
  glm::vec3 target_position = glm::vec3(0.0f);
  glm::vec3 target_lookat = glm::vec3(0.0f);

private:
  CameraController() = default;

  // FIXME: Should be dependent on the TOD Status
  ViewMode _current_view_mode = ViewMode::Normal;
  ViewState _normal_view_mode;
  ViewState _top_view_mode;
  ViewState get_direct_control_state() const;
  ViewState get_initial_state(const CameraComponent &camera) const;
  void update_top_view_state(CameraComponent &camera);
  void update_normal_view_state(CameraComponent &camera);
  void handle_orbital_rotation(const glm::vec2 &mousePos, CameraComponent &camera,
                             float deltaTime);
  void handle_panning(const glm::vec2 &mousePos, CameraComponent &camera,
                     float deltaTime);
  void handle_orbital_point_move(const glm::vec2 &mousePos, CameraComponent &camera, float deltaTime);


  KeyStateManager &keyManager = KeyStateManager::get_instance();

  // Mouse input state
  bool _is_right_mouse_pressed = false;
  bool _is_left_mouse_pressed = false;
  bool _is_middle_mouse_pressed = false;
  bool is_mouse_moved = false;

  double _last_mouse_x = 0.0;
  double _last_mouse_y = 0.0;

  bool _orbital_point_set = false;
  bool _target_position_initialized = false;

  const double movement_threshold = 0.5;
  const float MAX_PITCH = 85.0f;

  glm::vec3 _orbit_point = glm::vec3(0.0f);

  // Movement smoothing
  float _position_lerp_speed = 6.0f;
  float _lookat_lerp_speed = 6.0f;
  const float EPSILON = 0.001f;


  // Input sensitivity
  float _mouse_sensitivity = 1.0f;
  float _zoom_sensitivity = 2.0f;
  float _pan_speed = 1.f;

  float _move_speed = 10.0f;

};

} // namespace tod_gl

