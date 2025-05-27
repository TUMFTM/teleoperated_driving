/**
 * @file video_config_component.hpp
 * @brief Defines the VideoConfigComponent class for handling video configuration requests.
 * @ingroup tod_gl_ros_interface
 *
 * This file provides the interface for the VideoConfigComponent class, which facilitates
 * sending video configuration requests through ROS 2 services.
 * 
 * @copyright 2024 TUMFTM
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "tod_config_msgs/srv/video_config.hpp"

namespace tod_gl {

/**
 * @class VideoConfigComponent
 * @brief Handles video configuration requests via ROS 2 services.
 *
 * The VideoConfigComponent class provides an interface to send video configuration requests
 * using the `tod_msgs::srv::VideoConfig` service. It maintains a client for the service and
 * interacts with the associated ROS node.
 */
class VideoConfigComponent {
public:
    /**
     * @brief Constructs a VideoConfigComponent object.
     *
     * @param subNode A shared pointer to the ROS node used for service interactions.
     */
    VideoConfigComponent(std::shared_ptr<rclcpp::Node> subNode);

    /**
     * @brief Sends a video configuration request to the service.
     *
     * This method sends a `tod_msgs::srv::VideoConfig` request to the associated ROS service
     * and waits for the response.
     *
     * @param videoConfigRequest A shared pointer to the video configuration request message.
     * @return true if the request was successfully processed; false otherwise.
     */
    bool request_reconfigure(std::shared_ptr<tod_config_msgs::srv::VideoConfig::Request> videoConfigRequest);

private:
    std::shared_ptr<rclcpp::Client<tod_config_msgs::srv::VideoConfig>> _client; ///< Client for the video configuration service.
    std::shared_ptr<rclcpp::Node> _node; ///< Shared pointer to the ROS node.
};

} // namespace tod_gl
