/**
 * @file automation_status_component.cpp
 * @brief Handles subscribing to vehicle automation state and provides access to the current automation status in string form.
 *        Maps internal numeric states to human-readable strings such as "DISENGAGED", "AUTO", and "REMOTE".
 *        Useful for UI or logic components that need to display or act upon the current automation state.
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/subscribing_components/automation_status_component.hpp"

namespace tod_gl {

void AutomationStatusComponent::cb_message(const tod_automation_msgs::msg::VehicleAutomationState::SharedPtr msg) {
    automation_status_ = msg->vehicle_automation_state;
}

const std::string& AutomationStatusComponent::map_states_to_string(const std::unordered_map<u_int8_t, std::string>& map, 
                                                                   const u_int8_t& state) const {
    auto iter = map.find(state);
    if (iter != map.end()) {
        return iter->second;
    }
    else {
        return default_string_;
    }
}

}  // namespace tod_gl