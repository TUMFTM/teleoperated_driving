/**
 * @file operator_state_machine_node.cpp
 * @brief this file defines methods and variables used for the state machine on operator side
 * @copyright TUM-FTM
 */

#include "tod_state_machine/operator/operator_state_machine_node.hpp"

using std::placeholders::_1;
using namespace tod_operator_state_machine;

OperatorStateMachineNode::OperatorStateMachineNode() : Node ("operator_state_machine_node")
{
    sub_visual_status_ = this->create_subscription<tod_status_msgs::msg::ManagerButtonStatus>(
        "input/button_status",
        1,
        std::bind(&OperatorStateMachineNode::callback_visual_state, this, _1));
    
    sub_vehicle_status_ = this->create_subscription<tod_status_msgs::msg::Status>(
        "input/vehicle_status",
        1,
        std::bind(&OperatorStateMachineNode::callback_vehicle_state, this, _1));

    status_publisher_ = this->create_publisher<tod_status_msgs::msg::Status>(
        "output/operator_status", 
        10);

    set_operator_init_status();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), 
        [this]() {status_publisher_->publish(tod_operator_status_);});
        

}

void OperatorStateMachineNode::callback_visual_state(const tod_status_msgs::msg::ManagerButtonStatus::SharedPtr msg)
{
    tod_operator_status_.operator_control_mode = msg->vehicle_control_mode;
    switch (msg->clicked_button)
    {
        case 0:
            // if (tod_operator_status_.tod_vehicle_status == tod_status_msgs::msg::Status::TOD_STATUS_UPLINK_ONLY)
            // {
            tod_operator_status_.tod_status = tod_status_msgs::msg::Status::TOD_STATUS_UPLINK_ONLY;
            tod_operator_status_.operator_ip_address = msg->operator_ip_address;
            tod_operator_status_.vehicle_ip_address = msg->vehicle_ip_address;
            // }
            // else 
            // {
            //     RCLCPP_WARN(this->get_logger(), "No connection to vehicle. Remain in State IDLE.");
            // }
            break;
        case 2:
            if (tod_operator_status_.tod_vehicle_status == tod_status_msgs::msg::Status::TOD_STATUS_UPLINK_ONLY)
            {
                sm_.process_start_clicked();
                tod_operator_status_.tod_status = tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION;
            }
            else 
            {
                RCLCPP_WARN(this->get_logger(), "No connection to vehicle. Remain in State UPLINK.");
            }
            break;
        case 3:
            sm_.process_stop_clicked();
            tod_operator_status_.tod_status = tod_status_msgs::msg::Status::TOD_STATUS_UPLINK_ONLY;
            break;
        case 1:
            sm_.process_disconnect_clicked();
            tod_operator_status_.tod_status = tod_status_msgs::msg::Status::TOD_STATUS_IDLE;
            break;    
    }

    if (msg->clicked_button == 1)
    {
        // TODO: Check vehicle status
        sm_.process_disconnect_clicked();
        tod_operator_status_.tod_status = tod_status_msgs::msg::Status::TOD_STATUS_IDLE;
    }
}

void OperatorStateMachineNode::callback_vehicle_state(const tod_status_msgs::msg::Status::SharedPtr msg)
{
    std::lock_guard lock(mutex_);
    tod_operator_status_.tod_vehicle_status = msg->tod_vehicle_status;
    tod_operator_status_.vehicle_control_mode = msg->vehicle_control_mode;
    tod_operator_status_.vehicle_lat_approved = msg->vehicle_lat_approved;
    tod_operator_status_.vehicle_long_approved = msg->vehicle_long_approved;
    tod_operator_status_.vehicle_ip_address = msg->vehicle_ip_address;
    tod_operator_status_.vehicle_emergency_stop_released = msg->vehicle_emergency_stop_released;
    tod_operator_status_.vehicle_nav_status = msg->vehicle_nav_status;
    tod_operator_status_.vehicle_gps_pos_type = msg->vehicle_gps_pos_type;
    tod_operator_status_.vehicle_id = msg->vehicle_id;
}

void OperatorStateMachineNode::set_operator_init_status()
{
    tod_operator_status_.tod_status = tod_status_msgs::msg::Status::TOD_STATUS_IDLE;
    tod_operator_status_.vehicle_lat_approved = false;
    tod_operator_status_.vehicle_long_approved = false;
    tod_operator_status_.vehicle_emergency_stop_released = false;
    tod_operator_status_.operator_ip_address = "127.0.0.1";
    tod_operator_status_.vehicle_ip_address = "127.0.0.1";
    tod_operator_status_.operator_control_mode = tod_status_msgs::msg::Status::CONTROL_MODE_NONE;
    tod_operator_status_.operator_video_rate_mode = tod_status_msgs::msg::Status::VIDEO_RATE_CONTROL_MODE_SINGLE;
    tod_operator_status_.vehicle_id = "UNKNOWN";
}


int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tod_operator_state_machine::OperatorStateMachineNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}