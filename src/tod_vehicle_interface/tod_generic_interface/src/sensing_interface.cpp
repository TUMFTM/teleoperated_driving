/**
 * @file sensing_interface.cpp
 * @brief Generic sensing interface.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_generic_interface
 */

#include "tod_generic_interface/sensing_interface.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

namespace tod_generic_interface {

SensingInterface::SensingInterface(rclcpp::Node::SharedPtr node, std::string gnss_topic, std::string imu_topic, std::string odom_topic)
        : BaseInterface(node) 
{
    add_forwarder(gnss_topic, "from_sensing/fix", "sensor_msgs/msg/NavSatFix");
    add_forwarder(imu_topic, "from_sensing/imu", "sensor_msgs/msg/Imu");
    odom_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>("from_sensing/odom", 1);
    odom_subscriber_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 1, std::bind(&SensingInterface::odom_callback, this, _1));
}

void SensingInterface::odom_callback(const nav_msgs::msg::Odometry& msg) const 
{
    nav_msgs::msg::Odometry new_msg = msg;
    new_msg.pose.pose.position.x = std::fmod(msg.pose.pose.position.x , 100000.0);
    new_msg.pose.pose.position.y = std::fmod(msg.pose.pose.position.y , 100000.0);
    this->odom_publisher_->publish(new_msg);
}

} //namespace tod_generic_interface