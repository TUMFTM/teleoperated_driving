/**
 * @file network_monitor_component.hpp
 * @ingroup tod_gl_ros_interface
 **/
 #pragma once

 #include <rclcpp/rclcpp.hpp>
 
 #include "tod_network_monitoring_msgs/srv/network_monitor_service.hpp"
 
 namespace tod_gl {
 
 class NetworkMonitorComponent{
   public:
   NetworkMonitorComponent(std::shared_ptr<rclcpp::Node> subNode);
     void SetMonitorStatus(const std::string &vehicleIp, bool set_active);
 
   private:
   std::shared_ptr<rclcpp::Node> node_;
   std::shared_ptr<rclcpp::Client<tod_network_monitoring_msgs::srv::NetworkMonitorService>> client;
   };
 
 } // namespace tod_gl