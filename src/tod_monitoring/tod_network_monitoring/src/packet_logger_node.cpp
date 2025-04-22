/**
 * @file packet_logger_node.cpp
 * @brief ROS2 wrapper node for packet_logger
 * @copyright 2024 TUMFTM
 */

#include "tod_network_monitoring/packet_logger.hpp"
#include "tod_network_monitoring_msgs/srv/packet_capture_service.hpp"

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

namespace tod_network_monitoring {

/**
 * @brief Packet Logger Node
 * @ingroup tod_network_monitoring
 */
class PacketLoggerNode : public rclcpp::Node {

//================================================================================
// Public Functions
//================================================================================

public:
    PacketLoggerNode() : Node("packet_logger"), packet_logger_(std::make_unique<PacketLogger>()) {

    // PacketCapture service
    packet_capture_service_ = this->create_service<tod_network_monitoring_msgs::srv::PacketCaptureService>("packet_logger/set_capture_status",
        std::bind(&PacketLoggerNode::packet_capture_service_callback, this, std::placeholders::_1, std::placeholders::_2));

    this->declare_parameter<std::string>("network_interface", NETWORK_INTERFACE); // default from config.hpp
    this->get_parameter("network_interface", network_interface_);

    this->declare_parameter<std::string>("logging_directory", LOGGING_DIRECTORY); // default from config.hpp
    this->get_parameter("logging_directory", logging_directory_);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "packet_logger_node initialized. using network interface: %s", network_interface_.c_str());
    }

//================================================================================
// Private Functions
//================================================================================

private:
    rclcpp::Service<tod_network_monitoring_msgs::srv::PacketCaptureService>::SharedPtr packet_capture_service_;
    std::unique_ptr<PacketLogger> packet_logger_;
    std::string network_interface_;
    std::string logging_directory_;
    std::mutex mutex_;


    // PacketCapture service
    void packet_capture_service_callback(const std::shared_ptr<tod_network_monitoring_msgs::srv::PacketCaptureService::Request> request,
                                std::shared_ptr<tod_network_monitoring_msgs::srv::PacketCaptureService::Response> response) {

        std::lock_guard<std::mutex> lock(mutex_);
        bool desired_status = (request->set_capture_mode == true);
        bool observed_status = packet_logger_->is_running();

        // we do not want to change status of the capturing
        if (desired_status == observed_status) {

            response->is_active = observed_status;
            return;
        }

        // we want to start the packet capture
        if (desired_status && !observed_status) {

            // start a new thread, so that we do not block
            std::thread(&PacketLogger::start_capture, packet_logger_.get(), network_interface_, logging_directory_).join();

            observed_status = packet_logger_->is_running();

            if (observed_status != 1) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "failed to start packet capture");
            }
        }

        // we want to stop the packet capture
        else if (!desired_status && observed_status) {

            // no need for threading here
            packet_logger_->stop_capture();

            observed_status = packet_logger_->is_running();

            if (observed_status != 0) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "failed to stop packet capture");
            }
        }

        response->is_active = observed_status;
    }
};
}

//================================================================================
// ROS2 main()
//================================================================================

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tod_network_monitoring::PacketLoggerNode>());
    rclcpp::shutdown();
    return 0;
}
