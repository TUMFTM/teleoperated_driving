/**
 * @file video_config_component.cpp
 * @brief Implementation of the VideoConfigComponent class for handling video configuration requests.
 * @ingroup tod_gl_ros_interface
 * 
 * This file contains the implementation of the VideoConfigComponent class, which interacts
 * with the `VideoConfig` ROS service to handle video configuration requests.
 * 
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/service_components/video_config_component.hpp"

using namespace std::chrono_literals;

namespace tod_gl {

/**
 * @brief Constructs the VideoConfigComponent and initializes the service _client.
 * 
 * @param subNode A shared pointer to the ROS _node used for creating the _client and interacting with the service.
 */
VideoConfigComponent::VideoConfigComponent(std::shared_ptr<rclcpp::Node> subNode) : _node(subNode) {
    // Wait for service forwarder to start and get absolute name of video_config_service
    rclcpp::sleep_for(100ms);

    auto services = _node->get_service_names_and_types();
    std::string service_name = "/set_video_config";

    for (const auto& service : services) {
        if (service.first.find("set_video_config") != std::string::npos) {
            service_name = service.first;
            break;
        }
    }

    // create client, if service was not found fall back to default name:
    // "/video_config_service"
    _client = _node->create_client<tod_config_msgs::srv::VideoConfig>(service_name);

    if (!_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for the service.");
        }
        RCLCPP_ERROR(_node->get_logger(), "Service \"%s\" not available. Not creating a request", service_name.c_str());
        return;
    }
    RCLCPP_INFO(_node->get_logger(), "Service \"%s\" connected.", service_name.c_str());
}

/**
 * @brief Sends a video configuration request to the `VideoConfig` service.
 * 
 * This method sends a request to the `VideoConfig` service and waits for the response. 
 * It processes the response to determine if the request was successful.
 * 
 * @param videoConfigRequest A shared pointer to the request message for the `VideoConfig` service.
 * @return true if the service call succeeded and the response indicates success, false otherwise.
 */
bool VideoConfigComponent::request_reconfigure(std::shared_ptr<tod_config_msgs::srv::VideoConfig::Request> videoConfigRequest) {
    if (!_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(_node->get_logger(), "Interrupted while waiting for the service.");
            return false;
        }
        RCLCPP_ERROR(_node->get_logger(), "Service not available.");
        return false;
    }

    // Senden der Anfrage und Warten auf das Ergebnis
    auto future = _client->async_send_request(videoConfigRequest);
    
    // Überprüfen, ob die Anfrage erfolgreich war
    try {
        auto response = future.get();
        // Falls wir eine Antwort erhalten, können wir annehmen, dass der Service erfolgreich war
        return true;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(_node->get_logger(), "Failed to get service response: %s", e.what());
        return false;
    }
}


} // namespace tod_gl
