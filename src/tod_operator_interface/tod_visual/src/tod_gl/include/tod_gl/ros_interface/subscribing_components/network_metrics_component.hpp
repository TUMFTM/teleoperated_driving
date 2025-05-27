/**
 * @file network_metrics_component.hpp
 * @ingroup tod_gl_ros_interface
 * @brief Component for handling network metrics.
 *
 * This file contains the declaration of the NetworkMetricsComponent class, which subscribes to 
 * network metrics messages and provides methods for accessing various network statistics.
 */

#pragma once

#include "tod_gl/ros_interface/subscribing_component_base.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tod_network_monitoring_msgs/msg/network_metrics.hpp"


namespace tod_gl {

/**
 * @class NetworkMetricsComponent
 * @brief Handles network metrics by subscribing to the corresponding ROS topic.
 *
 * The NetworkMetricsComponent subscribes to the `tod_msgs::msg::NetworkMetrics` topic 
 * and extracts various network statistics, such as bitrate, packet rate, latency, and link quality.
 */
class NetworkMetricsComponent : public SubscribingComponent<tod_network_monitoring_msgs::msg::NetworkMetrics> {
  public:
    explicit NetworkMetricsComponent(std::shared_ptr<rclcpp::Node> sub_node)
      : SubscribingComponent(sub_node, "input/network_metrics"),
        rx_bitrate_mbps_(0.f),
        tx_bitrate_mbps_(0.f),
        rx_packets_s_(0.f),
        tx_packets_s_(0.f),
        latency_(0.f),
        link_quality_(0.f)
    {}
 
    /**
     * @brief Gets the received bitrate in Mbps.
     * @return The received bitrate in Mbps.
     */
    float get_rx_bitrate_mbps() const;

    /**
     * @brief Gets the transmitted bitrate in Mbps.
     * @return The transmitted bitrate in Mbps.
     */
    float get_tx_bitrate_mbps() const;

    /**
     * @brief Gets the number of received packets per second.
     * @return The number of received packets per second.
     */
    float get_rx_packets_per_second() const;

    /**
     * @brief Gets the number of transmitted packets per second.
     * @return The number of transmitted packets per second.
     */
    float get_tx_packets_per_second() const;

    /**
     * @brief Gets the latency in milliseconds.
     * @return The latency in milliseconds.
     */
    float get_latency() const;

    /**
     * @brief Gets the link quality as a percentage.
     * @return The link quality as a percentage.
     */
    float get_link_quality() const;

  private:
    /// The received bitrate in Mbps.
    float rx_bitrate_mbps_;

    /// The transmitted bitrate in Mbps.
    float tx_bitrate_mbps_;

    /// The number of received packets per second.
    float rx_packets_s_;

    /// The number of transmitted packets per second.
    float tx_packets_s_;
   /**
    * @brief The current link quality as a percentage.
    */
    float link_quality_;

    /**
    * @brief The current latency in milliseconds.
    */
    float latency_;

    void cb_message(const tod_network_monitoring_msgs::msg::NetworkMetrics::SharedPtr msg) override ;
};
}