/**
 * @file packet_capture_component.cpp
 * @brief Provides functionality to send asynchronous requests to enable or disable packet capture
 *        on both operator and vehicle nodes via corresponding ROS services.
 *        Useful for network monitoring and debugging in teleoperation or automation contexts.
 * 
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/service_components/packet_capture_component.hpp"

namespace tod_gl {

PacketCaptureComponent::PacketCaptureComponent(std::shared_ptr<rclcpp::Node> subNode) : _node(subNode){
  // Wait for service forwarder to start and get absolute name of set_capture_status
  rclcpp::sleep_for(100ms);

  auto services = subNode->get_service_names_and_types();
  std::string service_name = "/packet_logger/set_capture_status";  
  for (const auto& service : services) {
    if (service.first.find("packet_logger/set_capture_status") != std::string::npos) {
      service_name = service.first;
      break;
    }
  }
  
  // create client, if service was not found fall back to default name:
  // "/set_capture_status"
  _client_operator = subNode->create_client<tod_network_monitoring_msgs::srv::PacketCaptureService>(service_name);

  if (!_client_operator->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for the operator service.");
        }
        RCLCPP_ERROR(_node->get_logger(), "Operator Service not available. Not creating a operator request");
  }
  else { 
    RCLCPP_INFO(_node->get_logger(), "Operator Service \"%s\" connected.", service_name.c_str());
  }

  service_name = "/to_vehicle/set_capture_status";  
  for (const auto& service : services) {
    if (service.first.find("to_vehicle/set_capture_status") != std::string::npos) {
      service_name = service.first;
      break;
    }
  }

  _client_vehicle = _node->create_client<tod_network_monitoring_msgs::srv::PacketCaptureService>(service_name);
  if (!_client_vehicle->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for the vehicle service.");
        }
        RCLCPP_ERROR(_node->get_logger(), "Vehicle Service not available. not creating a request");
  }
  else { 
    RCLCPP_INFO(subNode->get_logger(), "Vehicle Service \"%s\" connected.", service_name.c_str());
  }
}

void PacketCaptureComponent::publish_packet_capture_operator(bool set_active) {
    if (_client_operator) {
    if (!_client_operator->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for the service.");
            return;
        }
        RCLCPP_ERROR(_node->get_logger(), "Service not available, not creating a request.");
        return;
    }
    } else {
        RCLCPP_INFO(this->_node->get_logger(), "Client Operator is null");
    }

    auto request = std::make_shared<tod_network_monitoring_msgs::srv::PacketCaptureService::Request>();

    request->set_capture_mode = set_active;

    _client_operator->async_send_request(
        request,
        [this, set_active](rclcpp::Client<tod_network_monitoring_msgs::srv::PacketCaptureService>::SharedFuture future) {
            try {
                auto response = future.get();

                if (response->is_active != set_active) {
                    RCLCPP_ERROR(this->_node->get_logger(), "failed to change capture status on operator!");
                }

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->_node->get_logger(), "Exception caught: %s", e.what());
            }
        }
    );
}

void PacketCaptureComponent::publish_packet_capture_vehicle(bool set_active) {

    if (!_client_vehicle->wait_for_service(1s)) {
        if (!rclcpp::ok()) {    
            RCLCPP_ERROR(this->_node->get_logger(), "Interrupted while waiting for the vehicle service.");
            return;
        }
        RCLCPP_ERROR(this->_node->get_logger(), "Vehicle Service not available, not creating a request.");
        return;
    }

    auto request = std::make_shared<tod_network_monitoring_msgs::srv::PacketCaptureService::Request>();

    request->set_capture_mode = set_active;

    _client_vehicle->async_send_request(
        request,
        [this, set_active](rclcpp::Client<tod_network_monitoring_msgs::srv::PacketCaptureService>::SharedFuture future) {
            try {
                auto response = future.get();

                if (response->is_active != set_active) {
                    RCLCPP_ERROR(this->_node->get_logger(), "failed to change capture status on vehicle!");
                }

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->_node->get_logger(), "Exception caught: %s", e.what());
            }
        }
    );
}

} // namespace tod_gl