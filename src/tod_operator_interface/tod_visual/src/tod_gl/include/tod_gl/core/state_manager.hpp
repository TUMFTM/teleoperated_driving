/**
 * @file state_manager.hpp
 * @brief The state manager modifies the render state based on the current tod_control mode and enables/disables both the layers as well as entities registired for the control concept 
 * @copyright 2024 TUMFTM
 */

#pragma once

#include <unordered_map>
#include <string>

#include "tod_status_msgs/msg/status.hpp"

namespace tod_gl {
/*
* @brief the Statemanager can be used to register entities and layers of the scene to only render when in a specific driving mode i.e. when we are in 
* DirectControl we disable most of the UI Elements and enable the rendering of the sperical video projections and when switching to TrajectoryGuidance 
* we enable the VideoLayers and start rendering the environment model of the autonomous vehicle
*/
class StateManager {
  public:
    static StateManager& get_instance() {
        static StateManager instance;
        return instance;
    }
    StateManager(const StateManager&) = delete;
    StateManager& operator=(const StateManager&) = delete;

    void set_state(uint8_t newState) { _current_mode = newState; }

    void set_toggle_setting(const std::string& tag, bool enabled) { _toggle_settings[tag] = std::move(enabled); }
    std::unordered_map<std::string, bool> get_toggle_settings() const { return _toggle_settings;}
    bool get_toggle_setting(const std::string& tag) const { 
      auto it = _toggle_settings.find(tag); 
      if (it != _toggle_settings.end()) {
        return it->second; 
      }
      return false;
    }

    uint8_t get_current_mode() const { return _current_mode; }

    void register_entity(const std::string& tag, std::vector<uint8_t> states) { _entity_states[tag] = std::move(states); }

    void register_layer(const std::string& layerName, std::vector<uint8_t> states) {
        _layer_states[layerName] = std::move(states);
    }

    bool contains_entity(const std::string& tag) const { return (_entity_states.count(tag) > 0); }

    bool contains_layer(const std::string& layerName) const { return (_layer_states.count(layerName) > 0); }

    bool should_render_entity(const std::string& tag) const {
        auto it = _entity_states.find(tag);
        return it != _entity_states.end() &&
               std::find(it->second.begin(), it->second.end(), _current_mode) != it->second.end();
    }

    bool should_render_layer(const std::string& layerName) const {
        auto it = _layer_states.find(layerName);
        return it != _layer_states.end() &&
               std::find(it->second.begin(), it->second.end(), _current_mode) != it->second.end();
    }

  private:
    StateManager() = default;
    uint8_t _current_mode = tod_status_msgs::msg::Status::CONTROL_MODE_NONE;
    std::unordered_map<std::string, std::vector<uint8_t>> _entity_states;
    std::unordered_map<std::string, std::vector<uint8_t>> _layer_states;
    std::unordered_map<std::string, bool> _toggle_settings;
};
}  // namespace tod_gl