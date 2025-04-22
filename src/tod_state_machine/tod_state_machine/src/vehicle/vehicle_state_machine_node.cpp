/**
 * @file vehicle_state_machine_node.cpp
 * @brief this file defines methods and variables used for the state machine
 * @copyright TUM-FTM
 */

#include "tod_state_machine/vehicle/vehicle_state_machine_node.hpp"

using std::placeholders::_1;
using namespace tod_vehicle_state_machine;

VehicleStateMachineNode::VehicleStateMachineNode() : Node ("vehicle_state_machine_node")
{
    sub_operator_status_ = this->create_subscription<tod_status_msgs::msg::Status>(
        "input/operator_status",
        1,
        std::bind(&VehicleStateMachineNode::callback_operator_status, this, _1));

    sub_safety_driver_status_ = this->create_subscription<tod_vehicle_msgs::msg::SafetyDriverStatus>(
        "input/safety_driver_status",
        1,
        std::bind(&VehicleStateMachineNode::callback_safety_driver_status, this, _1));

    pub_vehicle_status_ = this->create_publisher<tod_status_msgs::msg::Status>(
        "output/vehicle_status",
        10);
    
    set_vehicle_init_status();

    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            [this]() {pub_vehicle_status_->publish(tod_vehicle_status_);});
}

void VehicleStateMachineNode::callback_operator_status(const tod_status_msgs::msg::Status::SharedPtr msg)
{
    std::lock_guard lock(mutex_);
    switch (msg->tod_status)
    {
        case tod_status_msgs::msg::Status::TOD_STATUS_UPLINK_ONLY:
        case tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION:
            vsm_.process_connection_requested();
            tod_vehicle_status_.tod_status = msg->tod_status;
            tod_vehicle_status_.tod_vehicle_status = msg->tod_status;
            tod_vehicle_status_.operator_ip_address = msg->operator_ip_address;
            tod_vehicle_status_.operator_control_mode = msg->operator_control_mode;
            tod_vehicle_status_.operator_video_rate_mode = msg->operator_video_rate_mode;
            tod_vehicle_status_.vehicle_control_mode = msg->operator_control_mode;
            tod_vehicle_status_.vehicle_ip_address = msg->vehicle_ip_address;
            break;
        
        case tod_status_msgs::msg::Status::TOD_STATUS_IDLE:
            vsm_.process_disconnection_triggered();
            tod_vehicle_status_.tod_status = msg->tod_status;
            tod_vehicle_status_.tod_vehicle_status = tod_status_msgs::msg::Status::TOD_STATUS_IDLE;
            break;
    }
}

void VehicleStateMachineNode::callback_safety_driver_status(const tod_vehicle_msgs::msg::SafetyDriverStatus::SharedPtr msg)
{
    tod_vehicle_status_.vehicle_long_approved = msg->vehicle_long_approved;
    tod_vehicle_status_.vehicle_lat_approved = msg->vehicle_lat_approved;
    tod_vehicle_status_.vehicle_emergency_stop_released = msg->vehicle_emergency_stop_released;
}

void VehicleStateMachineNode::set_vehicle_init_status()
{
    tod_vehicle_status_.tod_status = tod_status_msgs::msg::Status::TOD_STATUS_IDLE;
    tod_vehicle_status_.tod_vehicle_status = tod_status_msgs::msg::Status::TOD_STATUS_IDLE;
    tod_vehicle_status_.vehicle_lat_approved = false;
    tod_vehicle_status_.vehicle_long_approved = false;
    tod_vehicle_status_.vehicle_emergency_stop_released = false;
    tod_vehicle_status_.operator_ip_address = "127.0.0.1";
    tod_vehicle_status_.vehicle_ip_address = "127.0.0.1";
    tod_vehicle_status_.operator_control_mode = tod_status_msgs::msg::Status::CONTROL_MODE_NONE;
    tod_vehicle_status_.operator_video_rate_mode = tod_status_msgs::msg::Status::VIDEO_RATE_CONTROL_MODE_SINGLE;
    tod_vehicle_status_.vehicle_id = "UNKNOWN";
}

int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tod_vehicle_state_machine::VehicleStateMachineNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}