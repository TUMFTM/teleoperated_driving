/**
 * @file trajectory_control_component.hpp
 * @brief Trajectory Control Component manages the subscription and data of the control commands send during the trajectory guidance control concept 
 * @copyright 2024 TUMFTM
 **/


#pragma once


#include "rclcpp/rclcpp.hpp"

#include "tod_gl/ros_interface/subscribing_component_base.hpp"

#include "tod_trajectory_guidance_msgs/msg/trajectory_guidance_control.hpp"

namespace tod_gl {
class TrajectoryControlComponent : public SubscribingComponent<tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceControl> 
{
public:
    explicit TrajectoryControlComponent(std::shared_ptr<rclcpp::Node> sub_node) 
    : SubscribingComponent(sub_node, "input/trajectory_guidance/trajectory_guidance_control"), 
        drive_status_(false),
        target_velocity_operator_(0.0f)
    {}
    
    bool drive_status_;
    float target_velocity_operator_;

private:
    rclcpp::Subscription<tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceControl>::SharedPtr subscription_;
    void cb_message(const tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceControl::SharedPtr msg) override {

        drive_status_ = msg->drive;
        target_velocity_operator_ = msg->target_velocity_operator;
        
    }
};
}