/**
 * @file edgarautoware_automation_interface.hpp
 * @brief Automation interface for the research vehicle EDGAR using Autoware as AV stack.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_edgarautoware_interface
 */

#pragma once

#include "tod_generic_interface/automation_interface.hpp"

#include "tod_automation_msgs/msg/vehicle_automation_state.hpp"

#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

namespace tod_edgarautoware_interface {
/**
 * @ingroup tod_edgarautoware_interface
 * @brief Interfaces for the research vehicle EDGAR using Autoware as AV stack.
 */

/**
 * @brief Automation interface for the research vehicle EDGAR using Autoware as AV stack.
 */
class AutomationInterface : public rclcpp::Node
{
    public:
        AutomationInterface();
        ~AutomationInterface();
        void run();
    private:    
        std::shared_ptr<tod_generic_interface::AutomationInterface> generic_automation_interface_;

        // Subscriptions
        // Conversion from incoming Autoware messages to tod_automation_msgs
        void aw_vehicle_automation_state_handler(const autoware_adapi_v1_msgs::msg::OperationModeState &msg);
        void aw_predicted_objects_handler(const autoware_auto_perception_msgs::msg::PredictedObjects &msg);
        void aw_trajectory_handler(const autoware_auto_planning_msgs::msg::Trajectory &msg);

        
        std::unordered_map<uint8_t, uint8_t> aw_to_tod_automation_state_map_
        {
            {autoware_adapi_v1_msgs::msg::OperationModeState::UNKNOWN, tod_automation_msgs::msg::VehicleAutomationState::AUTOMATION_STATE_UNKNOWN},
            {autoware_adapi_v1_msgs::msg::OperationModeState::STOP, tod_automation_msgs::msg::VehicleAutomationState::AUTOMATION_STATE_DISENGAGED},
            {autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS, tod_automation_msgs::msg::VehicleAutomationState::AUTOMATION_STATE_ENGAGED},
            {autoware_adapi_v1_msgs::msg::OperationModeState::REMOTE,  tod_automation_msgs::msg::VehicleAutomationState::AUTOMATION_STATE_REMOTEOPERATION}
        };
};

} // namespace tod_edgarautoware_interface