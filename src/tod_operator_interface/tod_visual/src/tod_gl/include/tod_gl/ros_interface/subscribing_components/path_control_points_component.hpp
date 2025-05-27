/**
 * @file path_control_points_component.hpp
 * @brief The PathControlPoints are visualizations of the spline control points in the spline used to generate a path for the vehicle @ref PathCreator   
 * @copyright 2024 TUMFTM
 **/

#pragma once


#include <rclcpp/rclcpp.hpp>

#include "tod_gl/ros_interface/subscribing_component_base.hpp"

#include "tod_trajectory_guidance_msgs/msg/control_points.hpp"

namespace tod_gl {
class PathControlPointsComponent  : public SubscribingComponent<tod_trajectory_guidance_msgs::msg::ControlPoints>  {
  public:
    explicit PathControlPointsComponent(std::shared_ptr<rclcpp::Node> sub_node) 
    : SubscribingComponent(sub_node, "input/trajectory_guidance/path_control_points"),
      points_(),
      valid_data_(false)
     {}

    bool hasValidData() const { return valid_data_; }
    const tod_trajectory_guidance_msgs::msg::ControlPoints getPoints() const {
        return points_;
    }

private:
    rclcpp::Subscription<tod_trajectory_guidance_msgs::msg::ControlPoints>::SharedPtr subscription_;
    bool valid_data_;
    tod_trajectory_guidance_msgs::msg::ControlPoints points_;

    void cb_message(const tod_trajectory_guidance_msgs::msg::ControlPoints::SharedPtr msg) override {
        points_ = *msg;
        valid_data_ = true; 
    }
};
}  // namespace tod_gl