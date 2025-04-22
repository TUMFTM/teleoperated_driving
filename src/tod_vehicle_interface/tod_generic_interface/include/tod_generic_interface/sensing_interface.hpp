/**
 * @file sensing_interface.hpp
 * @brief Generic sensing interface.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_generic_interface
 */

#pragma once

#include "tod_generic_interface/base_interface.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include <functional>

namespace tod_generic_interface {
/**
 * @ingroup tod_generic_interface
 * @brief Generic interfaces between vehicle platforms and the TUM Teleoperation software.
 */

/**
 * @brief Generic sensing interface.
 */
class SensingInterface : public BaseInterface 
{
    public:
        // these topics are manadatory for the stack to work
        SensingInterface(rclcpp::Node::SharedPtr node, std::string gnss_topic, std::string imu_topic, std::string odom_topic);
        virtual ~SensingInterface() = default;
    private:

        void odom_callback(const nav_msgs::msg::Odometry& msg) const;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;    
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
};

} // namespace tod_generic_interface