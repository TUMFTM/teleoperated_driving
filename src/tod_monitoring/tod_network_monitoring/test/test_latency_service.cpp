/**
 * @file test_latency_service.cpp
 * @brief ROS2 wrapper node to test latency_service
 * @copyright 2024 TUMFTM
 */

#include "tod_network_monitoring_msgs/srv/latency_service.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	if (argc != 2) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "usage: client_node hostname");
		return 1;
	}

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_tester_latency");
	rclcpp::Client<tod_network_monitoring_msgs::srv::LatencyService>::SharedPtr client =
		node->create_client<tod_network_monitoring_msgs::srv::LatencyService>("network_tester/latency_service");

	auto request = std::make_shared<tod_network_monitoring_msgs::srv::LatencyService::Request>();

	request->hostname = argv[1];

	while (!client->wait_for_service(1s)) {

		if (!rclcpp::ok()) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return 0;
		}

		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
	}

	// send asynchronous request
	auto result = client->async_send_request(request);

	// wait for the result
	if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {

		auto response = result.get();

		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received response:");
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "latency: %f", response->latency);


	} else {

		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call latency_service!");

	}

	rclcpp::shutdown();
	return 0;
}