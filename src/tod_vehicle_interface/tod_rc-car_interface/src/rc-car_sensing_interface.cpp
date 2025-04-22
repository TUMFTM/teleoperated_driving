/**
 * @file rc-car_sensing_interface.cpp
 * @brief RC-Car sensing interface.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_rc-car_interface
 */

#include "tod_rc-car_interface/rc-car_sensing_interface.hpp"

namespace tod_rccar_interface {

SensingInterface::SensingInterface() : rclcpp::Node("sensing_interface_node")
{
    this->declare_parameter<std::string>("gnss_topic", "default_gnss_topic");
    this->declare_parameter<std::string>("imu_topic", "/hedge_imu");
    this->declare_parameter<std::string>("odom_topic", "/vesc/odom");
}

void SensingInterface::run(){
    std::string gnss_topic = this->get_parameter("gnss_topic").as_string();
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    this->generic_sensing_interface_ = std::make_shared<tod_generic_interface::SensingInterface>(
            std::dynamic_pointer_cast<rclcpp::Node>(shared_from_this()),
            gnss_topic, 
            imu_topic, 
            odom_topic
    );
    RCLCPP_INFO(this->get_logger(), "SensingInterface forwarder initialized");
}

} // namespace tod_rccar_interface