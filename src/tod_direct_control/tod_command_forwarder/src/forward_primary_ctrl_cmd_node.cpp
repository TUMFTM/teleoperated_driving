// Copyright TUM-FTM
#include <rclcpp/rclcpp.hpp>
#include "tod_command_forwarder/forward_primary_ctrl_cmd.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tod_command_forwarder::ForwardPrimaryCtrlCmd>());
    rclcpp::shutdown();
    return 0;
}