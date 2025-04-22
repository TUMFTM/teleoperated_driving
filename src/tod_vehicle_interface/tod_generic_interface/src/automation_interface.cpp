/**
 * @file automation_interface.cpp
 * @brief Generic automation interface.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_generic_interface
 */

#include "tod_generic_interface/automation_interface.hpp"

#include <vector>
#include "tod_automation_msgs/msg/vehicle_automation_state.hpp"
#include "tod_automation_msgs/msg/predicted_objects.hpp"
#include "tod_automation_msgs/msg/trajectory.hpp"
#include "tod_automation_msgs/msg/trajectory_point.hpp"

namespace tod_generic_interface {

AutomationInterface::AutomationInterface(rclcpp::Node::SharedPtr node)
    : BaseInterface(node) 
{

    // Initialize attributes using add_attribute
    add_attribute<int8_t>("VehicleAutomationState_VehicleAutomationState", 0);
    add_attribute<std::vector<tod_automation_msgs::msg::PredictedObject>>("PredictedObjects_Objects", 
                                                                          std::vector<tod_automation_msgs::msg::PredictedObject>());
    add_attribute<std::string>("Trajectory_ChildFrameID", "base_link");
    add_attribute<std::vector<tod_automation_msgs::msg::TrajectoryPoint>>("Trajectory_Points", 
                                                                          std::vector<tod_automation_msgs::msg::TrajectoryPoint>());
    
    // Set up publishers
    add_publisher<tod_automation_msgs::msg::VehicleAutomationState>(
        "from_automation/automation_state",
        [this]() {
            tod_automation_msgs::msg::VehicleAutomationState output = tod_automation_msgs::msg::VehicleAutomationState();
            output.vehicle_automation_state = get_attribute<int8_t>("VehicleAutomationState_VehicleAutomationState");
            return output;
        },
        {"VehicleAutomationState_VehicleAutomationState"},
        10);
    
    add_publisher<tod_automation_msgs::msg::Trajectory>(
        "from_automation/trajectory",
        [this]() {
            tod_automation_msgs::msg::Trajectory output = tod_automation_msgs::msg::Trajectory();
            output.header.stamp = get_clock().now();
            output.child_frame_id = get_attribute<std::string>("Trajectory_ChildFrameID");
            output.points = get_attribute<std::vector<tod_automation_msgs::msg::TrajectoryPoint>>("Trajectory_Points");
            return output;
        },
        {"Trajectory_ChildFrameID", "Trajectory_Points"},
        10);
    
    add_publisher<tod_automation_msgs::msg::PredictedObjects>(
        "from_automation/predicted_objects",
        [this]() {
            tod_automation_msgs::msg::PredictedObjects output = tod_automation_msgs::msg::PredictedObjects();
            output.header.stamp = get_clock().now();
            output.objects = get_attribute<std::vector<tod_automation_msgs::msg::PredictedObject>>("PredictedObjects_Objects");
            return output;
        },
        {"PredictedObjects_Objects"},
        10);
}

} // namespace generic_interface