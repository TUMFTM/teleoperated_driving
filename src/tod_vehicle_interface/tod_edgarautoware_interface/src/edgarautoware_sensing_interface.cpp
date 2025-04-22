/**
 * @file edgarautoware_sensing_interface.cpp
 * @brief Sensing interface for the research vehicle EDGAR using Autoware as AV stack.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_edgarautoware_interface
 */

#include "tod_edgarautoware_interface/edgarautoware_sensing_interface.hpp"

namespace tod_edgarautoware_interface {

SensingInterface::SensingInterface() : rclcpp::Node("edgarautoware_sensing_interface")
{
    this->declare_parameter<std::string>("gnss_topic", "default_gnss_topic");
    this->declare_parameter<std::string>("imu_topic", "default_imu_topic");
    this->declare_parameter<std::string>("odom_topic", "default_odom_topic");
}

void SensingInterface::run()
{
    std::string gnss_topic = this->get_parameter("gnss_topic").as_string();
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    this->generic_sensing_interface_ = std::make_shared<tod_generic_interface::SensingInterface>(
            std::dynamic_pointer_cast<rclcpp::Node>(shared_from_this()),
            gnss_topic, 
            imu_topic, 
            odom_topic
    );
    RCLCPP_INFO(this->get_logger(), "SensingInterface initialized");
}

} // namespace tod_edgarautoware_interface