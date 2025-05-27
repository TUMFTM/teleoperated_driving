/**
 * @file driving_lane_component.cpp
 * @brief  Contains data of the predicted path of the vehicle while driving in direct control mode.
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/subscribing_components/driving_lane_component.hpp"

namespace tod_gl {

void DrivingLaneComponent::cb_message(const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = *msg;
        has_received_path_ = true;
}

}  // namespace tod_gl