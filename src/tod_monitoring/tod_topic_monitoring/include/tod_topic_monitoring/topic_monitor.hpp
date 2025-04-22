/**
 * @file topic_monitor.hpp
 * @brief header file for topic_monitor
 * @copyright 2024 TUMFTM
 */

#pragma once

#include "rclcpp/create_generic_subscription.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tod_status_msgs/msg/status.hpp"
#include "tod_topic_monitoring_msgs/msg/topic_state.hpp"
#include "tod_topic_monitoring/topic_watchdog.hpp"

namespace tod_topic_monitoring {

/**
 * @brief Class for monitoring the timeouts of topics
 * @ingroup tod_topic_monitoring
 */
class TopicMonitoringNode : public rclcpp::Node {
  public:
    TopicMonitoringNode();

  private:
    rclcpp::Subscription<tod_status_msgs::msg::Status>::SharedPtr status_subscriber_;
    rclcpp::Publisher<tod_topic_monitoring_msgs::msg::TopicState>::SharedPtr topic_status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    tod_topic_watchdog::TopicWatchdog::UniquePtr topic_watchdog_;
    tod_topic_monitoring_msgs::msg::TopicState topic_status_;

    std::vector<std::string> topic_array_;
    std::vector<std::string> type_array_;
    std::vector<int64_t> timeout_array_;
    std::vector<std::vector<int64_t>> control_modes_array_;
    float warning_factor_;
    uint8_t current_control_mode_;

    void timer_callback_();
    void callback_generic_(std::shared_ptr<rclcpp::SerializedMessage> msg);
    void timeout_callback_(bool timeout, std::chrono::milliseconds timeout_now);
};

}  // namespace tod_topic_monitoring