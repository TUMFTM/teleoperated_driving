/**
 * @file odometry_component.cpp
 * @brief holds the odometry information of the vehicle in the world - the initial position is offset such that the 
 * Scene only renders relative update to prevent transformation render artifacts
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/subscribing_components/odometry_component.hpp"

namespace tod_gl {

void OdometryComponent::cb_message(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Pose with optional offset
    position_ = glm::vec3(msg->pose.pose.position.x, 
                          msg->pose.pose.position.y, 
                          msg->pose.pose.position.z);
    
    // Check if orientation is valid
    if (msg->pose.pose.orientation.x == 0.f && msg->pose.pose.orientation.y == 0.f && msg->pose.pose.orientation.z && 0.f, msg->pose.pose.orientation.w == 0.f) {
        orientation_ = tf2::Quaternion(0.f, 0.f, 0.f, 1.f);
    }
    else {
    orientation_ = tf2::Quaternion(msg->pose.pose.orientation.x, 
                                   msg->pose.pose.orientation.y,
                                   msg->pose.pose.orientation.z, 
                                   msg->pose.pose.orientation.w);
    }
    // Twist
    linear_velocities_ = glm::vec3(msg->twist.twist.linear.x, 
                                   msg->twist.twist.linear.y, 
                                   msg->twist.twist.linear.z);

    angular_velocities_ = glm::vec3(msg->twist.twist.angular.x, 
                                    msg->twist.twist.angular.y, 
                                    msg->twist.twist.angular.z);
    // Data state
    has_received_data_ = true;
}

}  // namespace tod_gl