/**
 * @file trajectory_component.hpp
 * @brief TrajectoryComponent manages the subscription and data recieved from trajectories from the AV
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <vector>

#include "tod_gl/ros_interface/subscribing_component_base.hpp"
#include "tod_gl/renderer/data_container.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tod_automation_msgs/msg/trajectory.hpp"

namespace tod_gl {
  
class TrajectoryComponent : public SubscribingComponent<tod_automation_msgs::msg::Trajectory> {

  public:
    explicit TrajectoryComponent(std::shared_ptr<rclcpp::Node> sub_node)
      : SubscribingComponent(sub_node, "input/trajectory"),
        trajectory_()
    {}

    const std::vector<tod_gl::TrajectoryPoint>& get_trajectory() const { return trajectory_; }; 

  private:
    void cb_message(const tod_automation_msgs::msg::Trajectory::SharedPtr msg) override;
    std::vector<tod_gl::TrajectoryPoint> trajectory_;
};

}  // namespace tod_gl