/**
 * @file input_device_component.hpp
 * @brief Provides functionality to send asynchronous requests to enable or disable packet capture
 *        on both operator and vehicle nodes via corresponding ROS services.
 *        Useful for network monitoring and debugging in teleoperation or automation contexts.
 * 
 * @ingroup tod_gl_ros_interface
 * @copyright 2024 TUMFTM
 **/
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "tod_network_monitoring_msgs/srv/packet_capture_service.hpp"

using namespace std::chrono_literals;

namespace tod_gl {
  class PacketCaptureComponent{
    public:
    PacketCaptureComponent(std::shared_ptr<rclcpp::Node> subNode);

    void publish_packet_capture_operator(bool setActive);
    void publish_packet_capture_vehicle(bool setActive);
private:
    std::shared_ptr<rclcpp::Client<tod_network_monitoring_msgs::srv::PacketCaptureService>> _client_operator;
    std::shared_ptr<rclcpp::Client<tod_network_monitoring_msgs::srv::PacketCaptureService>> _client_vehicle;
    std::shared_ptr<rclcpp::Node> _node;
  };
} // namespace tod_gl