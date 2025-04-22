/**
 * @file topic_monitor.cpp
 * @brief topic monitoring node implementation
 * @copyright 2024 TUMFTM
 */

#include "tod_topic_monitoring/topic_monitor.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace tod_topic_monitoring
{

    TopicMonitoringNode::TopicMonitoringNode() : Node("topic_monitor")
    {
        this->declare_parameter<float>("warning_factor", 0.5);
        this->get_parameter("warning_factor", warning_factor_);

        this->declare_parameter<std::vector<std::string>>("topic_names", {"input/status"});
        this->get_parameter("topic_names", topic_array_);

        for (std::string &name : topic_array_)
        {
            this->declare_parameter<std::string>("topics." + name + ".type");
            this->declare_parameter<int>("topics." + name + ".timeout");
            this->declare_parameter<std::vector<int64_t>>("topics." + name + ".control_modes", {0});
            type_array_.push_back(this->get_parameter("topics." + name + ".type").as_string());
            timeout_array_.push_back(this->get_parameter("topics." + name + ".timeout").as_int());
            control_modes_array_.push_back(this->get_parameter("topics." + name + ".control_modes").as_integer_array());
        }

        if (topic_array_.size() != type_array_.size() || topic_array_.size() != timeout_array_.size())
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "topic_array, type_array and timeout_array must have the same size");
            return;
        }

        topic_status_publisher_ =
            this->create_publisher<tod_topic_monitoring_msgs::msg::TopicState>("output/topic_monitoring_status", 10);
        topic_watchdog_ = std::make_unique<tod_topic_watchdog::TopicWatchdog>(this, warning_factor_);

        for (size_t i = 0; i < topic_array_.size(); ++i)
        {
            topic_watchdog_->add_subscription(topic_array_[i], type_array_[i], 10,
                                              std::bind(&TopicMonitoringNode::callback_generic_, this, _1),
                                              std::bind(&TopicMonitoringNode::timeout_callback_, this, _1, _2),
                                              std::chrono::milliseconds(timeout_array_[i]));
        }

        current_control_mode_ = tod_status_msgs::msg::Status::CONTROL_MODE_NONE;
        status_subscriber_ = this->create_subscription<tod_status_msgs::msg::Status>(
            "input/status", 10, [this](const tod_status_msgs::msg::Status::SharedPtr msg)
            {
            int vehicle_control_mode = msg->vehicle_control_mode;
            if (vehicle_control_mode != current_control_mode_) {
                std::vector<bool> watched_callbacks_bool;

                RCLCPP_INFO(this->get_logger(), "Watched Topics: ");
                for (size_t i = 0; i < control_modes_array_.size(); ++i) {
                    const auto &control_modes = control_modes_array_[i];
                    bool found = std::find(control_modes.begin(), control_modes.end(), vehicle_control_mode) !=
                                 control_modes.end();
                    watched_callbacks_bool.push_back(found);
                    if (found) {
                        RCLCPP_INFO(this->get_logger(), "Topic: %s", topic_array_[i].c_str());
                    }
                }
                current_control_mode_ = vehicle_control_mode;
                topic_watchdog_->update_watched_callbacks(watched_callbacks_bool);
            } });

        timer_ = this->create_wall_timer(100ms, std::bind(&TopicMonitoringNode::timer_callback_, this));

        topic_status_.state = tod_topic_monitoring_msgs::msg::TopicState::STATE_NOT_RECEIVED;

        RCLCPP_INFO(this->get_logger(), "Initialized topic_monitor with the following topics and timeout rates:");
        for (size_t i = 0; i < topic_array_.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "Topic: %s, Type: %s, Timeout: %ld ms", topic_array_[i].c_str(),
                        type_array_[i].c_str(), timeout_array_[i]);
        }
    }

    void TopicMonitoringNode::timer_callback_()
    {
        topic_watchdog_->check_timeouts();
        topic_status_.state = topic_watchdog_->get_overall_status();
        topic_status_.header.stamp = this->now();
        topic_status_publisher_->publish(topic_status_);
    }

    void TopicMonitoringNode::callback_generic_(std::shared_ptr<rclcpp::SerializedMessage> msg) {}

    void TopicMonitoringNode::timeout_callback_(bool timeout, std::chrono::milliseconds timeout_now) {}

} // namespace tod_topic_monitoring