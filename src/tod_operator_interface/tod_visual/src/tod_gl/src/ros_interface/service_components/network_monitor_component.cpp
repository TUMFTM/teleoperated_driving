/**
* @file network_monitor_component.cpp
* @copyright 2025 TUMFTM
*/

#include "tod_gl/ros_interface/service_components/network_monitor_component.hpp"

using namespace std::chrono_literals;
 
namespace tod_gl {
 
NetworkMonitorComponent::NetworkMonitorComponent(std::shared_ptr<rclcpp::Node> subNode)
    : node_(subNode)  // Store subNode in node_ member variable
{
    // Wait for service forwarder to start and get absolute name of set_monitoring_status
    rclcpp::sleep_for(100ms);

    auto services = subNode->get_service_names_and_types();
    std::string service_name = "/set_monitoring_status";

    for (const auto& service : services) {
        if (service.first.find("to_vehicle/set_monitoring_status") != std::string::npos) {
            service_name = service.first;
            break;
        }
    }

    // create client, if service was not found fall back to default name:
    // "/set_monitoring_status"
    client = node_->create_client<tod_network_monitoring_msgs::srv::NetworkMonitorService>(service_name);
    if (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service.");
        }
        RCLCPP_ERROR(subNode->get_logger(), "Service \"%s\" not available. Not creating a request", service_name.c_str());
        return;
    }
    RCLCPP_INFO(subNode->get_logger(), "Service \"%s\" connected.", service_name.c_str());
}

void NetworkMonitorComponent::SetMonitorStatus(const std::string &vehicleIp, bool set_active) {
    auto request = std::make_shared<tod_network_monitoring_msgs::srv::NetworkMonitorService::Request>();
    request->vehicle_ip_address = vehicleIp;
    request->set_monitor_mode = set_active;

            client->async_send_request(
                request,
                [this, set_active](rclcpp::Client<tod_network_monitoring_msgs::srv::NetworkMonitorService>::SharedFuture future) {
                    try {
                        auto response = future.get();
                        if (response->is_active != set_active) {
                            RCLCPP_ERROR(this->node_->get_logger(), "failed to change monitoring status on operator!");
                        }
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->node_->get_logger(), "Exception caught: %s", e.what());
                    }
                }
            );

    
    // Capture `this` to access node_ in the lambda function
}

} // namespace tod_gl
 