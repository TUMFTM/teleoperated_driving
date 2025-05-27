/**
 * @file input_device_component.cpp
 * @brief Service to change the input device before starting the teleoperation
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/service_components/input_device_component.hpp"

using namespace std::chrono_literals;

namespace tod_gl {

InputDeviceComponent::InputDeviceComponent(std::shared_ptr<rclcpp::Node> subNode) {
  // Wait for input_devices to start and get absolute name of change_input_device_service
  rclcpp::sleep_for(100ms);

  auto services = subNode->get_service_names_and_types();
  std::string service_name = "/change_input_device";

  for (const auto& service : services) {
    if (service.first.find("change_input_device") != std::string::npos) {
      service_name = service.first;
      break;
    }
  }

  // create client, if service was not found fall back to default name:
  // "/change_input_device"
  _client = subNode->create_client<tod_operator_msgs::srv::InputDevice>(service_name);
  if (!_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(subNode->get_logger(), "Interrupted while waiting for the service.");
        }
        RCLCPP_ERROR(subNode->get_logger(), "Service \"%s\" not available. Not creating a request", service_name.c_str());
        return;
  }
  RCLCPP_INFO(subNode->get_logger(), "Service \"%s\" connected.", service_name.c_str());
}

void InputDeviceComponent::publish_input_device(const std::string& inputDevice){
  auto request = std::make_shared<tod_operator_msgs::srv::InputDevice::Request>();
  request->input_device_directory = inputDevice;
  auto response = _client->async_send_request(request);
}

} // namespace tod_gl