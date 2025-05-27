/**
 * @file automation_status_component.hpp
 * @brief Handles subscribing to vehicle automation state and provides access to the current automation status in string form.
 *        Maps internal numeric states to human-readable strings such as "DISENGAGED", "AUTO", and "REMOTE".
 *        Useful for UI or logic components that need to display or act upon the current automation state.
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <string>
#include <map>

#include "tod_gl/ros_interface/subscribing_component_base.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tod_automation_msgs/msg/vehicle_automation_state.hpp"

namespace tod_gl {
  
class AutomationStatusComponent : public SubscribingComponent<tod_automation_msgs::msg::VehicleAutomationState> {
  public:
    explicit AutomationStatusComponent(std::shared_ptr<rclcpp::Node> sub_node)
      : SubscribingComponent(sub_node, "input/automation_status"),
        default_string_("UNKOWN"),
        automation_status_(0)
    {}
    
    u_int8_t get_automation_status() const { return automation_status_; }
    const std::string& get_automation_status_string() const { return map_states_to_string(automation_status_map_, automation_status_); };
    
  private:
    void cb_message(const tod_automation_msgs::msg::VehicleAutomationState::SharedPtr msg) override;
    const std::string& map_states_to_string (const std::unordered_map<u_int8_t, std::string>& map, const u_int8_t& state) const;

    std::unordered_map<u_int8_t, std::string> automation_status_map_{
        {tod_automation_msgs::msg::VehicleAutomationState::AUTOMATION_STATE_DISENGAGED, "DISENGAGED"},
        {tod_automation_msgs::msg::VehicleAutomationState::AUTOMATION_STATE_ENGAGED, "AUTO"},
        {tod_automation_msgs::msg::VehicleAutomationState::AUTOMATION_STATE_REMOTEOPERATION, "REMOTE"}
    };
    
    std::string default_string_;

    u_int8_t automation_status_;
};

}  // namespace tod_gl