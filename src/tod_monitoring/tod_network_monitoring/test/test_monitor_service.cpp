/**
 * @file test_monitor_service.cpp
 * @brief ROS2 wrapper node to test network_monitor_service
 * @copyright 2024 TUMFTM
 */

#include "tod_network_monitoring_msgs/srv/network_monitor_service.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	if (argc != 3) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "usage: client_node vehicle_ip_address set_monitor_mode");
		return 1;
	}

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_tester_monitoring");
	rclcpp::Client<tod_network_monitoring_msgs::srv::NetworkMonitorService>::SharedPtr client =
		node->create_client<tod_network_monitoring_msgs::srv::NetworkMonitorService>("network_monitor/set_monitoring_status");

	auto request = std::make_shared<tod_network_monitoring_msgs::srv::NetworkMonitorService::Request>();

	request->vehicle_ip_address = argv[1];
	request->set_monitor_mode = atol(argv[2]); // omit checking for boolean type, as false == 0 and true != 0

	while (!client->wait_for_service(1s)) {

		if (!rclcpp::ok()) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "interrupted while waiting for service. goodbye.");
			return 0;
		}

		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
	}

	// send asynchronous request
	auto result = client->async_send_request(request);

	// wait for the result
	if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {

		auto response = result.get();

		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "received response:");
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "is_active: %d", response->is_active);

		auto expected = request->set_monitor_mode;
		auto observed = response->is_active;

		if (expected != observed) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "monitoring status did not change as expected!");
		}

	} else {

		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "failed to call network_monitor_service!");

	}

	rclcpp::shutdown();
	return 0;
}