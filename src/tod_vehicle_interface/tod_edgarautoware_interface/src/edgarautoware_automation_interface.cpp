/**
 * @file edgarautoware_automation_interface.cpp
 * @brief Automation interface for the research vehicle EDGAR using Autoware as AV stack.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_edgarautoware_interface
 */

#include "tod_edgarautoware_interface/edgarautoware_automation_interface.hpp"

#include "tod_automation_msgs/msg/trajectory_point.hpp"
#include "tod_automation_msgs/msg/predicted_object.hpp"

namespace tod_edgarautoware_interface {

AutomationInterface::AutomationInterface() : rclcpp::Node("edgarautoware_automation_interface")
{
    this->declare_parameter<std::string>("aw_vehicle_automation_state_topic", "default_vehicle_automation_state_topic");
    this->declare_parameter<std::string>("aw_predicted_objects_topic", "default_predicted_objects_topic");
    this->declare_parameter<std::string>("aw_trajectory_topic", "default_trajectory_topic");
}

AutomationInterface::~AutomationInterface() 
{
    RCLCPP_INFO(this->get_logger(), "AutomationInterface shutting down");
}

void AutomationInterface::run()
{
    this->generic_automation_interface_ = std::make_shared<tod_generic_interface::AutomationInterface>(
            std::dynamic_pointer_cast<rclcpp::Node>(shared_from_this())
    );

    std::string aw_vehicle_automation_state_topic = this->get_parameter("aw_vehicle_automation_state_topic").as_string();
    std::string aw_predicted_objects_topic = this->get_parameter("aw_predicted_objects_topic").as_string();
    std::string aw_trajectory_topic = this->get_parameter("aw_trajectory_topic").as_string();

    // Subscriptions - Add subscriber
    if (aw_vehicle_automation_state_topic != "none") {
        this->generic_automation_interface_->add_subscriber<autoware_adapi_v1_msgs::msg::OperationModeState>(
            aw_vehicle_automation_state_topic,
            [this](const autoware_adapi_v1_msgs::msg::OperationModeState &msg) {
                this->aw_vehicle_automation_state_handler(msg);
            });
    }

    if (aw_predicted_objects_topic != "none") {
        this->generic_automation_interface_->add_subscriber<autoware_auto_perception_msgs::msg::PredictedObjects>(
            aw_predicted_objects_topic,
            [this](const autoware_auto_perception_msgs::msg::PredictedObjects &msg) {
                this->aw_predicted_objects_handler(msg);
            });
    }

    if (aw_trajectory_topic != "none") {
        this->generic_automation_interface_->add_subscriber<autoware_auto_planning_msgs::msg::Trajectory>(
            aw_trajectory_topic,
            [this](const autoware_auto_planning_msgs::msg::Trajectory &msg) {
                this->aw_trajectory_handler(msg);
            });
    }

    RCLCPP_INFO(this->get_logger(), "AutomationInterface initialized");
}

 // Subscriptions - Message Handler
void AutomationInterface::aw_vehicle_automation_state_handler(const autoware_adapi_v1_msgs::msg::OperationModeState &msg)
{
    uint8_t mode = this->aw_to_tod_automation_state_map_.find(msg.mode)->second;
    this->generic_automation_interface_->update_attribute("VehicleAutomationState_VehicleAutomationState", static_cast<int8_t>(mode));
}

void AutomationInterface::aw_predicted_objects_handler(const autoware_auto_perception_msgs::msg::PredictedObjects &msg)
{
    auto tod_predicted_objects = std::vector<tod_automation_msgs::msg::PredictedObject>();
    
    for (const auto& aw_predicted_object : msg.objects) {
            tod_automation_msgs::msg::PredictedObject tod_predicted_object;
            
            // Pose and Bounding Box
            tod_predicted_object.pose = aw_predicted_object.kinematics.initial_pose_with_covariance.pose;          
            tod_predicted_object.dimensions = aw_predicted_object.shape.dimensions;
            
            // Classification 
            tod_predicted_object.classification = tod_automation_msgs::msg::PredictedObject::UNKNOWN;
            if (aw_predicted_object.classification.size() != 0){ 
                auto classification = *aw_predicted_object.classification.begin();
                tod_predicted_object.classification = classification.label;
            }

            tod_predicted_objects.push_back(tod_predicted_object);
    }

    this->generic_automation_interface_->update_attribute("PredictedObjects_Objects", 
                                                         static_cast<std::vector<tod_automation_msgs::msg::PredictedObject>>(
                                                            tod_predicted_objects));
}

void AutomationInterface::aw_trajectory_handler(const autoware_auto_planning_msgs::msg::Trajectory &msg)
{
    this->generic_automation_interface_->update_attribute("Trajectory_ChildFrameID", static_cast<std::string>(msg.header.frame_id));

    std::vector<tod_automation_msgs::msg::TrajectoryPoint> tod_trajectory_points;
    for (auto aw_point : msg.points) {
        tod_automation_msgs::msg::TrajectoryPoint tod_point;

        tod_point.pose.pose = aw_point.pose;
        tod_point.twist.twist.linear.x = aw_point.longitudinal_velocity_mps;
        tod_point.twist.twist.linear.y = aw_point.lateral_velocity_mps;
        tod_point.twist.twist.angular.z = aw_point.heading_rate_rps;

        tod_trajectory_points.push_back(tod_point);
    }

    this->generic_automation_interface_->update_attribute("Trajectory_Points", 
                                                         static_cast<std::vector<tod_automation_msgs::msg::TrajectoryPoint>>(tod_trajectory_points));                                                   
}

} // namespace tod_edgarautoware_interface