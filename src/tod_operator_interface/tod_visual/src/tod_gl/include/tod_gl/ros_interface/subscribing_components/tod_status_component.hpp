/**
 * @file tod_status_component.hpp
 * @ingroup tod_gl_ros_interface
 * @brief ToD Status component that manages the subscription and the data of the recieved tod status messages, such as current control mode and other information for the manager application
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include <string>
#include <map>

#include "tod_gl/ros_interface/subscribing_component_base.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tod_status_msgs/msg/status.hpp"

namespace tod_gl {
  
class TodStatusComponent : public SubscribingComponent<tod_status_msgs::msg::Status> {
  public:
    explicit TodStatusComponent(std::shared_ptr<rclcpp::Node> sub_node)
      : SubscribingComponent(sub_node, "input/tod_status"),
        default_string_("UNKOWN"),
        // General
        tod_status_(0),
        // Operator
        operator_ip_address_(),
        operator_control_mode_(99),
        operator_video_rate_mode_(0),
        // Vehicle 
        vehicle_nav_status_(),
        vehicle_gps_pos_type_(),
        vehicle_id_(),
        vehicle_ip_address_(),
        tod_vehicle_status_(0),
        vehicle_control_mode_(99),
        vehicle_emergency_stop_released_(false),
        vehicle_long_approved_(false),
        vehicle_lat_approved_(false)
    {}

    // General
    uint8_t get_tod_status() const { return tod_status_; }
    const std::string& get_tod_status_string() const { return map_states_to_string(tod_status_map_, tod_status_); };

    // Operator
    uint8_t get_operator_control_mode() const { return operator_control_mode_; }
    const std::string& get_operator_control_mode_string() const { return map_states_to_string(control_mode_map_, operator_control_mode_); };

    std::string get_operator_ip_address() const { return operator_ip_address_; }

    // Vehicle
    uint8_t get_vehicle_control_mode() const { return vehicle_control_mode_; }
    const std::string& get_vehicle_control_mode_string() const { return map_states_to_string(control_mode_map_, vehicle_control_mode_); };

    uint8_t get_operator_video_rate_mode() const { return operator_video_rate_mode_; }
    const std::string& get_operator_video_rate_mode_string() const { return map_states_to_string(video_rate_map_, operator_video_rate_mode_); };
    
    std::string get_vehicle_ip_address() const { return vehicle_ip_address_; }
    std::string get_vehicle_nav_status() const { return vehicle_nav_status_; }
    std::string get_vehicle_gps_pos_type() const { return vehicle_gps_pos_type_; }
    std::string get_vehicle_id() const { return vehicle_id_; }

    bool is_emergency_stop_released() const { return vehicle_emergency_stop_released_; }
    bool is_long_approved() const { return vehicle_long_approved_; }
    bool is_lat_approved() const { return vehicle_lat_approved_; }

  private:
    void cb_message(const tod_status_msgs::msg::Status::SharedPtr msg) override;
    const std::string& map_states_to_string (const std::unordered_map<u_int8_t, std::string>& map, const u_int8_t& state) const;

    std::unordered_map<u_int8_t, std::string> control_mode_map_{
        {tod_status_msgs::msg::Status::CONTROL_MODE_NONE, "NONE"},
        {tod_status_msgs::msg::Status::CONTROL_MODE_DIRECT, "DIRECT CONTROL"},
        {tod_status_msgs::msg::Status::CONTROL_MODE_SHARED, "SHARED CONTROL"},
        {tod_status_msgs::msg::Status::CONTROL_MODE_WAYPOINT, "WAYPOINT GUIDANCE"},
        {tod_status_msgs::msg::Status::CONTROL_MODE_PATH_GUIDANCE, "TRAJECTORY GUIDANCE"}, // TODO: Correct status msg
        {tod_status_msgs::msg::Status::CONTROL_MODE_PERCEPTION_MODIFICATION, "PERCEPTION MODIFICATION"},
        {tod_status_msgs::msg::Status::CONTROL_MODE_SAFECORRIDOR, "SAFE CORRIDOR"} // TODO: Correct status msg
    };

    std::unordered_map<u_int8_t, std::string> tod_status_map_{
        {tod_status_msgs::msg::Status::TOD_STATUS_IDLE, "IDLE"},
        {tod_status_msgs::msg::Status::TOD_STATUS_UPLINK_ONLY, "UPLINK ONLY"},
        {tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION, "TELEOPERATION"}
    };

    std::unordered_map<u_int8_t, std::string> video_rate_map_{
        {tod_status_msgs::msg::Status::VIDEO_RATE_CONTROL_MODE_SINGLE, "SINGLE"},
        {tod_status_msgs::msg::Status::VIDEO_RATE_CONTROL_MODE_COLLECTIVE, "COLLECTIVE"},
        {tod_status_msgs::msg::Status::VIDEO_RATE_CONTROL_MODE_AUTOMATIC, "AUTOMATIC"}
    };

    std::string default_string_;

    // General
    uint8_t tod_status_;

    // Operator
    std::string operator_ip_address_;
    uint8_t operator_control_mode_;
    uint8_t operator_video_rate_mode_;

    // Vehicle
    std::string vehicle_nav_status_;
    std::string vehicle_gps_pos_type_;
    std::string vehicle_id_;
    std::string vehicle_ip_address_;
    uint8_t tod_vehicle_status_;
    uint8_t vehicle_control_mode_;
    bool vehicle_emergency_stop_released_;
    bool vehicle_long_approved_;
    bool vehicle_lat_approved_;
};

}  // namespace tod_gl
