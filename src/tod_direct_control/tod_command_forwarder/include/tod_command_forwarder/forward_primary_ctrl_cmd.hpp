// Copyright TUM-FTM
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "tod_status_msgs/msg/status.hpp"
#include "tod_vehicle_msgs/msg/primary_control_cmd.hpp"

namespace tod_command_forwarder {
/**
 * @brief Checks if the current control mode is direct.
 * @return True if the control mode is direct, false otherwise.
 */
bool inDirectControlMode();

/**
 * @class ForwardPrimaryCtrlCmd 
 * @brief ROS2 Node for forwarding the primary control command given by the operator
 * on the vehicle's side
 * 
 * This node subscribes to the PrimaryControlCommand on the vehicle side and republishes
 * it in the direct control namespace id the direct control mode is selected by the 
 * remote operator
 */
class ForwardPrimaryCtrlCmd : public rclcpp::Node
{
    public:
        ForwardPrimaryCtrlCmd();    ///< Constructs the ForwardPrimaryCtrlCmd node

    private:
        rclcpp::Publisher<tod_vehicle_msgs::msg::PrimaryControlCmd>::SharedPtr publisher_;
        rclcpp::Subscription<tod_vehicle_msgs::msg::PrimaryControlCmd>::SharedPtr sub_primary_cmd_;
        rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr sub_status_;

        /**
         * @brief Callback for handling of the incoming control command
         * @param msg The incoming control command message 
         */
        void callback_direct_control(const tod_vehicle_msgs::msg::PrimaryControlCmd& msg);
        
        /**
         * @brief Callback for handling the incoming status message
         * @param msg The incoming status message 
         */
        void callback_status(const tod_status_msgs::msg::Status& msg);
};
} // namespace