/**
 * @file safety_gate.cpp
 * @brief ROS2 node for a safety gate
 * @copyright 2025 TUM-FTM
 */

#include "tod_safety_gate/safety_gate.hpp"
#include <algorithm>

tod_safety_gate::SafetyGateNode::SafetyGateNode() : Node("safety_gate") {
    this->declare_parameter<float>("warning_velocity", 2.7778);  // default from config.hpp
    this->get_parameter("warning_velocity", warning_velocity_);

    primary_control_cmd_subscriber_ = this->create_subscription<tod_vehicle_msgs::msg::PrimaryControlCmd>(
        "input/primary_control_cmd", 10,
        std::bind(&SafetyGateNode::primary_control_command_callback, this, std::placeholders::_1));

    secondary_control_cmd_subscriber_ = this->create_subscription<tod_vehicle_msgs::msg::SecondaryControlCmd>(
        "input/secondary_control_cmd", 10,
        std::bind(&SafetyGateNode::secondary_control_command_callback, this, std::placeholders::_1));

    status_subscriber_ = this->create_subscription<tod_status_msgs::msg::Status>(
        "input/status", 10, std::bind(&SafetyGateNode::status_callback, this, std::placeholders::_1));

    topic_state_subscriber_ = this->create_subscription<tod_topic_monitoring_msgs::msg::TopicState>(
        "input/topic_monitoring_status", 10,
        std::bind(&SafetyGateNode::topic_state_callback, this, std::placeholders::_1));

    primary_control_cmd_publisher_ =
        this->create_publisher<tod_vehicle_msgs::msg::PrimaryControlCmd>("output/primary_control_cmd", 10);
    secondary_control_cmd_publisher_ =
        this->create_publisher<tod_vehicle_msgs::msg::SecondaryControlCmd>("output/secondary_control_cmd", 10);
}

tod_safety_gate::SafetyGateNode::~SafetyGateNode() {}

void tod_safety_gate::SafetyGateNode::primary_control_command_callback(
    const tod_vehicle_msgs::msg::PrimaryControlCmd::SharedPtr msg) {
    if (tod_status_ == tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION) {
        switch (topic_state_) {
            case tod_topic_monitoring_msgs::msg::TopicState::STATE_NOT_RECEIVED:
                RCLCPP_WARN_STREAM_ONCE(this->get_logger(),
                                        "not all monitored topics have been received, stopping vehicle");
                msg->velocity = 0.0;
                break;
            case tod_topic_monitoring_msgs::msg::TopicState::STATE_WARN:
                RCLCPP_WARN_STREAM(this->get_logger(),
                                   "warning state received from tod_topic_monitoring, limiting velocity");
                msg->velocity = std::min(msg->velocity, warning_velocity_);
                break;
            case tod_topic_monitoring_msgs::msg::TopicState::STATE_ERROR:
                RCLCPP_ERROR_STREAM(this->get_logger(),
                                    "error state received from tod_topic_monitoring, stopping vehicle");
                msg->velocity = 0.0;
                break;
        }
        primary_control_cmd_publisher_->publish(*msg);
    }
}

void tod_safety_gate::SafetyGateNode::secondary_control_command_callback(
    const tod_vehicle_msgs::msg::SecondaryControlCmd::SharedPtr msg) {
    if (tod_status_ == tod_status_msgs::msg::Status::TOD_STATUS_TELEOPERATION &&
            (topic_state_ == tod_topic_monitoring_msgs::msg::TopicState::STATE_OK) ||
        (topic_state_ == tod_topic_monitoring_msgs::msg::TopicState::STATE_WARN)) {
        secondary_control_cmd_publisher_->publish(*msg);
    }
}

void tod_safety_gate::SafetyGateNode::status_callback(const tod_status_msgs::msg::Status::SharedPtr msg) {
    tod_status_ = msg->tod_vehicle_status;
}

void tod_safety_gate::SafetyGateNode::topic_state_callback(
    const tod_topic_monitoring_msgs::msg::TopicState::SharedPtr msg) {
    topic_state_ = msg->state;
}