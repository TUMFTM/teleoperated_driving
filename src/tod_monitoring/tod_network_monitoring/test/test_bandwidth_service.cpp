/**
 * @file test_bandwidth_service.cpp
 * @brief ROS2 wrapper node to test bandwidth_service
 * @copyright 2024 TUMFTM
 */

#include "tod_network_monitoring_msgs/srv/bandwidth_service.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	if (argc != 3) {
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "usage: client_node hostname test_vehicle_upload");
		return 1;
	}

	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_tester_bandwidth");
	rclcpp::Client<tod_network_monitoring_msgs::srv::BandwidthService>::SharedPtr client =
		node->create_client<tod_network_monitoring_msgs::srv::BandwidthService>("network_tester/bandwidth_service");

	auto request = std::make_shared<tod_network_monitoring_msgs::srv::BandwidthService::Request>();

	request->hostname = argv[1];
	request->test_vehicle_upload = atol(argv[2]); // omit checking for boolean type, as false == 0 and true > 0

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
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "bitrate_mbps: %ld", response->bitrate_mbps);
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "transferred_bytes: %ld", response->transferred_bytes);

	} else {

		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service bandwidth_service!");

	}

	rclcpp::shutdown();
	return 0;
}