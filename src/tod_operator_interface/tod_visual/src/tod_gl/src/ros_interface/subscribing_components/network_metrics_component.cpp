/**
 * @file network_metrics_component.cpp
 * @brief Implementation of the NetworkMetricsComponent class for handling network metrics subscriptions.
 * @ingroup tod_gl_ros_interface
 * 
 * This file contains the implementation of the NetworkMetricsComponent class, which subscribes to the 
 * `NetworkMetrics` ROS topic and provides access to network-related metrics such as bitrate, latency, and link quality.
 * 
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/subscribing_components/network_metrics_component.hpp"

#include <iostream>

namespace tod_gl {


float NetworkMetricsComponent::get_rx_bitrate_mbps() const {
    return rx_bitrate_mbps_;
}


float NetworkMetricsComponent::get_tx_bitrate_mbps() const {
    return tx_bitrate_mbps_;
}


float NetworkMetricsComponent::get_rx_packets_per_second() const {
    return rx_packets_s_;
}


float NetworkMetricsComponent::get_tx_packets_per_second() const {
    return tx_packets_s_;
}


float NetworkMetricsComponent::get_latency() const {
    return latency_;
}


float NetworkMetricsComponent::get_link_quality() const {
    return link_quality_;
}

/**
 * @brief Callback function for processing received `NetworkMetrics` messages.
 * 
 * This function is called whenever a new message is published on the `NetworkMetrics` topic.
 * It extracts and updates the network metrics such as link quality, latency, and bitrates.
 * 
 * @param msg A shared pointer to the received `NetworkMetrics` message.
 */


void NetworkMetricsComponent::cb_message(const tod_network_monitoring_msgs::msg::NetworkMetrics::SharedPtr msg) {
    rx_bitrate_mbps_ = msg->rx_bitrate_mbps;
    tx_bitrate_mbps_ = msg->tx_bitrate_mbps;
    rx_packets_s_ = msg->rx_packets_s;
    tx_packets_s_ = msg->tx_packets_s;
    latency_ = msg->latency; // ms 
    link_quality_ = msg->link_quality;
}

}  // namespace tod_gl