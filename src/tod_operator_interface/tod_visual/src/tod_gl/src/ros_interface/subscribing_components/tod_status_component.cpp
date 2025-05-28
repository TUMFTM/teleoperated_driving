/**
 * @file tod_status_component.cpp
 * @brief ToD Status component that manages the subscription and the data of the recieved tod status messages, such as current control mode and other information for the manager application
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/subscribing_components/tod_status_component.hpp"

namespace tod_gl {

void TodStatusComponent::cb_message(const tod_status_msgs::msg::Status::SharedPtr msg) {
    // General
    tod_status_ = msg->tod_status;
    // Operator
    operator_ip_address_ = msg->operator_ip_address;
    operator_control_mode_ = msg->operator_control_mode;
    operator_video_rate_mode_ = msg->operator_video_rate_mode;
    // Vehicle 
    vehicle_nav_status_ = msg->vehicle_nav_status;
    vehicle_gps_pos_type_ = msg->vehicle_gps_pos_type;
    vehicle_id_ = msg->vehicle_id;
    vehicle_ip_address_ = msg->vehicle_ip_address;
    tod_vehicle_status_ = msg->tod_vehicle_status;
    vehicle_control_mode_ = msg->vehicle_control_mode;
    vehicle_emergency_stop_released_ = msg->vehicle_emergency_stop_released;
    vehicle_long_approved_ = msg->vehicle_long_approved;
    vehicle_lat_approved_ = msg->vehicle_lat_approved;
}

const std::string& TodStatusComponent::map_states_to_string(const std::unordered_map<u_int8_t, std::string>& map, 
                                                     const u_int8_t& state) const {
    auto iter = map.find(state);
    if (iter != map.end()) {
        return iter->second;
    }
    else {
        return default_string_;
    }
}

}