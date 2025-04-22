/**
 * @file test_packet_capture_service.cpp
 * @brief ROS2 wrapper node to test packet_capture_service
 * @copyright 2024 TUMFTM
 */

#include "tod_network_monitoring_msgs/srv/packet_capture_service.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	if (argc != 2) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "usage: client_node set_capture_mode");
		return 1;
	}

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_tester_packet_capture");
	rclcpp::Client<tod_network_monitoring_msgs::srv::PacketCaptureService>::SharedPtr client =
		node->create_client<tod_network_monitoring_msgs::srv::PacketCaptureService>("packet_logger/set_capture_status");

	auto request = std::make_shared<tod_network_monitoring_msgs::srv::PacketCaptureService::Request>();

	request->set_capture_mode = atol(argv[1]); // omit checking for boolean type, as false == 0 and true != 0

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

		auto expected = request->set_capture_mode;
		auto observed = response->is_active;

		if (expected != observed) {
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "capture status did not change as expected!");
		}

	} else {

		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "failed to call packet_capture_service!");

	}

	rclcpp::shutdown();
	return 0;
}