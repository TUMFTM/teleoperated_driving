/**
 * @file network_tester_node.cpp
 * @brief ROS2 wrapper node for network_tester
 * @copyright 2024 TUMFTM
 */

#include "tod_network_monitoring/network_tester.hpp"
#include "tod_network_monitoring_msgs/srv/latency_service.hpp"
#include "tod_network_monitoring_msgs/srv/bandwidth_service.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace tod_network_monitoring {
/**
 * @brief Network Testing Node
 * @ingroup tod_network_monitoring
 */
class NetworkTesterNode : public rclcpp::Node {

//================================================================================
// Public Functions
//================================================================================

public:
    NetworkTesterNode() : Node("network_tester"), network_tester_(std::make_unique<NetworkTester>()) {

    this->declare_parameter<float>("update_time_interval", UPDATE_TIME_INTERVAL); // default from config.hpp
    this->get_parameter("update_time_interval", update_time_interval_);

    this->declare_parameter<int>("ping_timeout", PING_TIMEOUT); // default from config.hpp
    this->get_parameter("ping_timeout", ping_timeout_);

    this->declare_parameter<int>("iperf3_port", IPERF3_PORT); // default from config.hpp
    this->get_parameter("iperf3_port", iperf3_port_);

    this->declare_parameter<int>("bandwidth_measurement_time", BANDWIDTH_MEASUREMENT_TIME); // default from config.hpp
    this->get_parameter("bandwidth_measurement_time", bandwidth_measurement_time_);

    // Latency service
    latency_service_ = this->create_service<tod_network_monitoring_msgs::srv::LatencyService>("network_tester/latency_service",
        std::bind(&NetworkTesterNode::latency_service_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Bandwidth service
    bandwidth_service_ = this->create_service<tod_network_monitoring_msgs::srv::BandwidthService>("network_tester/bandwidth_service",
        std::bind(&NetworkTesterNode::bandwidth_service_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "network_tester_node initialized");

    }

//================================================================================
// Private Functions
//================================================================================

private:
    rclcpp::Service<tod_network_monitoring_msgs::srv::LatencyService>::SharedPtr latency_service_;
    rclcpp::Service<tod_network_monitoring_msgs::srv::BandwidthService>::SharedPtr bandwidth_service_;

    std::mutex mutex_;

    std::unique_ptr<NetworkTester> network_tester_;

    float update_time_interval_;
    int ping_timeout_;
    int iperf3_port_;
    int bandwidth_measurement_time_;

    // Latency service
    void latency_service_callback(const std::shared_ptr<tod_network_monitoring_msgs::srv::LatencyService::Request> request,
                                std::shared_ptr<tod_network_monitoring_msgs::srv::LatencyService::Response> response) {

        std::lock_guard<std::mutex> lock(mutex_);

        std::string hostname = request->hostname;

        std::optional<float> latencyOpt = network_tester_->test_latency(hostname, update_time_interval_, ping_timeout_);

        // check if value exists
        if (latencyOpt.has_value()) {
            response->latency = latencyOpt.value();
            return;

        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "failed to get latency to host '%s'", hostname.c_str());
            response->latency = -1;
            return;
        }
    }

    // Bandwidth service
    void bandwidth_service_callback(const std::shared_ptr<tod_network_monitoring_msgs::srv::BandwidthService::Request> request,
                                std::shared_ptr<tod_network_monitoring_msgs::srv::BandwidthService::Response> response) {

        std::lock_guard<std::mutex> lock(mutex_);

        std::string hostname = request->hostname;
        const bool is_reverse = request->test_vehicle_upload;

        BandwidthStats stats = network_tester_->test_bandwidth(hostname, iperf3_port_, bandwidth_measurement_time_, is_reverse);

        // no bytes were transferred during the bandwidth test => e.g. the test failed
        if (stats.transferred_bytes == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to execute bandwidth test to host '%s'", hostname.c_str());
            response->bitrate_mbps = -1;
            response->transferred_bytes = -1;
            return;

        } else {
            response->bitrate_mbps = stats.bitrate_mbps;
            response->transferred_bytes = stats.transferred_bytes;
            return;
        }
    }
};
}
//================================================================================
// ROS2 main()
//================================================================================

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tod_network_monitoring::NetworkTesterNode>());
    rclcpp::shutdown();
    return 0;
}
