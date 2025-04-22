// Copyright TUM-FTM
#include "tod_command_forwarder/forward_primary_ctrl_cmd.hpp"

using std::placeholders::_1;

uint8_t _ctrlMode = 99;

bool tod_command_forwarder::inDirectControlMode() { 
    return _ctrlMode == tod_status_msgs::msg::Status::CONTROL_MODE_DIRECT; 
}

tod_command_forwarder::ForwardPrimaryCtrlCmd::ForwardPrimaryCtrlCmd() : Node("forward_primary_ctrl_cmd") {
    publisher_ = this->create_publisher<tod_vehicle_msgs::msg::PrimaryControlCmd>("output/primary_control_cmd", 1);

    sub_primary_cmd_ = this->create_subscription<tod_vehicle_msgs::msg::PrimaryControlCmd>(
        "input/primary_control_cmd",
        1,
        std::bind(&ForwardPrimaryCtrlCmd::callback_direct_control, this, _1)
    );

    sub_status_ = this->create_subscription<tod_status_msgs::msg::Status>(
        "input/vehicle_status",
        1,
        std::bind(&ForwardPrimaryCtrlCmd::callback_status, this, _1)
    );

}

void tod_command_forwarder::ForwardPrimaryCtrlCmd::callback_direct_control(const tod_vehicle_msgs::msg::PrimaryControlCmd& msg) {
    if(inDirectControlMode()) {
        publisher_->publish(msg);
    }
}

void tod_command_forwarder::ForwardPrimaryCtrlCmd::callback_status(const tod_status_msgs::msg::Status& msg) {
    _ctrlMode = msg.vehicle_control_mode;
}