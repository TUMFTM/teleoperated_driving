/**
 * @file network_monitor_node.cpp
 * @brief ROS2 wrapper node for network monitor
 * @copyright 2024 TUMFTM
 */

#include "tod_network_monitoring/network_monitor.hpp"
#include "tod_network_monitoring_msgs/srv/network_monitor_service.hpp"
#include "tod_network_monitoring_msgs/msg/network_metrics.hpp"

#include <memory>
#include <atomic>

#include "rclcpp/rclcpp.hpp"

namespace tod_network_monitoring {
/**
 * @brief Network Monitoring Node
 * @ingroup tod_network_monitoring
 */
class NetworkMonitorNode : public rclcpp::Node {

public:
    /**
     * @brief Construct a new Network Monitor Node object
     */
    NetworkMonitorNode() : Node("network_monitor"), network_monitor_(std::make_unique<NetworkMonitor>()) {

    // NetworkMonitor service
    network_monitor_service_ = this->create_service<tod_network_monitoring_msgs::srv::NetworkMonitorService>("network_monitor/set_monitoring_status",
        std::bind(&NetworkMonitorNode::network_monitor_service_callback, this, std::placeholders::_1, std::placeholders::_2));

    this->declare_parameter<std::string>("network_interface", NETWORK_INTERFACE); // default from config.hpp
    this->get_parameter("network_interface", network_interface_);

    this->declare_parameter<float>("update_timeout", UPDATE_TIMEOUT); // default from config.hpp
    this->get_parameter("update_timeout", update_timeout_);

    this->declare_parameter<float>("update_time_interval", UPDATE_TIME_INTERVAL); // default from config.hpp
    this->get_parameter("update_time_interval", update_time_interval_);

    // NetworkMetrics publisher
    network_metrics_publisher_ = this->create_publisher<tod_network_monitoring_msgs::msg::NetworkMetrics>("output/network_metrics", 10);

    // timer for publish interval
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&NetworkMonitorNode::publish_network_metrics, this));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "network_monitor_node initialized. using network interface: %s", network_interface_.c_str());

    }

private:
    rclcpp::Service<tod_network_monitoring_msgs::srv::NetworkMonitorService>::SharedPtr network_monitor_service_;
    rclcpp::Publisher<tod_network_monitoring_msgs::msg::NetworkMetrics>::SharedPtr network_metrics_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<bool> publish_enabled_ = false;
    std::mutex mutex_;

    std::unique_ptr<NetworkMonitor> network_monitor_;
    std::string network_interface_;

    float update_timeout_;
    float update_time_interval_;

    // NetworkMonitor service
    void network_monitor_service_callback(const std::shared_ptr<tod_network_monitoring_msgs::srv::NetworkMonitorService::Request> request,
                                std::shared_ptr<tod_network_monitoring_msgs::srv::NetworkMonitorService::Response> response) {

        std::lock_guard<std::mutex> lock(mutex_);

        bool desired_status = (request->set_monitor_mode == true);
        bool observed_status = network_monitor_->is_monitoring();

        // we do not want to change status of the monitoring
        if (desired_status == observed_status) {

            response->is_active = observed_status;
            return;
        }

        // we want to start monitoring
        if (desired_status && !observed_status) {

            std::string hostname = request->vehicle_ip_address;

            if (network_monitor_->start_monitoring(hostname, network_interface_, update_timeout_, update_time_interval_) != 0) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "failed to start monitoring");
                response->is_active = 0;
                return;
            }

            publish_enabled_ = true;
            response->is_active = 1;
            return;
        }

        // we want to stop monitoring
        else if (!desired_status && observed_status) {

            if (network_monitor_->stop_monitoring() != 0) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "failed to stop monitoring");
                response->is_active = 1;
            }

            publish_enabled_ = false;
            response->is_active = 0;
            return;
        }
    }

    // NetworkMetrics publisher
    void publish_network_metrics() {

        if (!publish_enabled_) {
            return;
        }

        NetworkMetrics metrics = network_monitor_->get_network_metrics();

        tod_network_monitoring_msgs::msg::NetworkMetrics msg;

        msg.header.stamp = this->now();
        msg.rx_bitrate_mbps = metrics.rx_bitrate_mbps;
        msg.tx_bitrate_mbps = metrics.tx_bitrate_mbps;
        msg.rx_packets_s = metrics.rx_packets_s;
        msg.tx_packets_s = metrics.tx_packets_s;
        msg.latency = metrics.latency;
        msg.link_quality = metrics.link_quality;

        network_metrics_publisher_->publish(msg);
    }
};

}

//================================================================================
// ROS2 main()
//================================================================================

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tod_network_monitoring::NetworkMonitorNode>());
    rclcpp::shutdown();
    return 0;
}
