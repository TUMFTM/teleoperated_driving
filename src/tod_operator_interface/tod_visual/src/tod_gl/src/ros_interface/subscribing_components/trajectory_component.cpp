/**
 * @file trajectory_component.cpp
 * @brief  TrajectoryComponent manages the subscription and data recieved from trajectories from the AV
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/subscribing_components/trajectory_component.hpp"

namespace tod_gl {

void TrajectoryComponent::cb_message(const tod_automation_msgs::msg::Trajectory::SharedPtr msg) {
    trajectory_.clear();
    for (const auto& point : msg->points) {
        trajectory_.emplace_back(
            TrajectoryPoint(
                glm::vec3(point.pose.pose.position.x, point.pose.pose.position.y, point.pose.pose.position.z),
                glm::quat(point.pose.pose.orientation.w, point.pose.pose.orientation.x,
                          point.pose.pose.orientation.y, point.pose.pose.orientation.z),
                glm::vec3(point.twist.twist.angular.x, point.twist.twist.angular.y, point.twist.twist.angular.z),
                glm::vec3(point.twist.twist.linear.x, point.twist.twist.linear.y, point.twist.twist.linear.z)
            )
        );
    }
}

}  // namespace tod_gl