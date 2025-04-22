/**
 * @file topic_watchdog.hpp
 * @brief header file for topic_watchdog
 * @copyright 2024 TUMFTM
 */

#pragma once
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

namespace tod_topic_watchdog {

enum TopicStatus { NOT_RECEIVED = 0, OK = 1, WARN = 2, ERROR = 3 };

struct TimeoutDescriptor {
    std::function<void(bool, std::chrono::milliseconds)> timeout_callback;
    std::chrono::milliseconds timeout;
    rclcpp::Time last_update;
    int status;
    TimeoutDescriptor(std::function<void(bool, std::chrono::milliseconds)> timeout_callback_,
                      std::chrono::milliseconds timeout_, rclcpp::Time last_update_, int status_)
        : timeout_callback{timeout_callback_}, timeout{timeout_}, last_update{last_update_}, status{status_} {}
};

/**
 * @brief Watchdog implementation for topic timeouts
 * @ingroup tod_topic_watchdog
 */
class TopicWatchdog {
  private:
    rclcpp::Node *node_;
    std::vector<std::shared_ptr<TimeoutDescriptor>> all_callbacks_;
    std::vector<std::shared_ptr<TimeoutDescriptor>> watched_callbacks_;
    // Just store them so they don't go out of scope
    std::vector<rclcpp::SubscriptionBase::SharedPtr> created_ros_subs_;
    float warning_factor_;
    std::vector<bool> watched_callbacks_bool_;

  public:
    void check_timeouts() {
        if (get_overall_status() == TopicStatus::NOT_RECEIVED) {
            return;
        }

        auto now = this->node_->get_clock()->now();
        for (auto &watchdog_sub : this->watched_callbacks_) {
            std::chrono::milliseconds timeout_now =
                (now - watchdog_sub->last_update).to_chrono<std::chrono::milliseconds>();
            watchdog_sub->timeout_callback(timeout_now > watchdog_sub->timeout, timeout_now);
            if (timeout_now > watchdog_sub->timeout) {
                watchdog_sub->status = TopicStatus::ERROR;
            } else if (timeout_now > watchdog_sub->timeout * warning_factor_) {
                watchdog_sub->status = TopicStatus::WARN;
            } else {
                watchdog_sub->status = TopicStatus::OK;
            }
        }
    };

    explicit TopicWatchdog(rclcpp::Node *node, float warning_factor = 0.5) {
        this->node_ = node;
        warning_factor_ = warning_factor;
    };
    /**
     * @param topic returns a std::function of the TopicWatchdogs update function.
     */
    std::function<void()> get_update_function() { return std::bind(&TopicWatchdog::check_timeouts, this); };
    /**
     * @param topic Name of the subscribed topic
     * @param qos QoS settings of the subscription
     * @param subscription_callback Function that handles incoming messages, will be passed to the
     * underlying ROS 2 subscription
     * @param timeout_callback Function that is called whenever a timeout is detected
     * @param timeout Timeout for the subscription, e.g. 500ms
     * @returns Subscription Handle of the underlying ROS 2 subscription
     */

    template <typename CallbackT>
    auto timeout_callback(CallbackT &&callback, std::function<void(bool, std::chrono::milliseconds)> timeout_callback,
                          std::chrono::milliseconds timeout) {
        auto timeout_descriptor = std::make_shared<TimeoutDescriptor>(
            TimeoutDescriptor{timeout_callback, timeout, this->node_->get_clock()->now(), TopicStatus::NOT_RECEIVED});
        this->watched_callbacks_.emplace_back(timeout_descriptor);

        return [callback = std::forward<CallbackT>(callback), timeout_descriptor, this](auto &&...args) {
            timeout_descriptor->last_update = this->node_->get_clock()->now();
            return callback(std::forward<decltype(args)>(args)...);
        };
    }

    template <typename CallbackT>
    auto create_timeout_descriptor(std::function<void(bool, std::chrono::milliseconds)> timeout_callback,
                                   std::chrono::milliseconds timeout, CallbackT &&callback) {
        auto timeout_descriptor = std::make_shared<TimeoutDescriptor>(
            TimeoutDescriptor{timeout_callback, timeout, this->node_->get_clock()->now(), TopicStatus::NOT_RECEIVED});

        auto wrapped_callback = [this, timeout_descriptor,
                                 callback = std::forward<CallbackT>(callback)](auto &&...args) {
            timeout_descriptor->last_update = this->node_->get_clock()->now();
            if (timeout_descriptor->status == TopicStatus::NOT_RECEIVED) {
                timeout_descriptor->status = TopicStatus::OK;
            }
            callback(std::forward<decltype(args)>(args)...);
        };

        this->all_callbacks_.emplace_back(timeout_descriptor);
        return std::make_pair(timeout_descriptor, wrapped_callback);
    }

    template <typename T, typename CallbackT>
    typename rclcpp::Subscription<T>::SharedPtr add_subscription(
        const std::string &topic, const rclcpp::QoS &qos, CallbackT &&subscription_callback,
        std::function<void(bool, std::chrono::milliseconds)> timeout_callback, std::chrono::milliseconds timeout,
        bool watched = true) {
        auto [timeout_descriptor, wrapped_callback] =
            create_timeout_descriptor(timeout_callback, timeout, std::forward<CallbackT>(subscription_callback));

        using rclcpp::AnySubscriptionCallback;
        std::shared_ptr<AnySubscriptionCallback<T>> any_subscription_callback =
            std::make_shared<AnySubscriptionCallback<T>>();
        any_subscription_callback->set(wrapped_callback);

        typename rclcpp::Subscription<T>::SharedPtr ros_sub = node_->create_subscription<T>(
            topic, qos, [any_subscription_callback](std::shared_ptr<T> msg, const rclcpp::MessageInfo &message_info) {
                any_subscription_callback->dispatch(msg, message_info);
            });

        created_ros_subs_.push_back(ros_sub);
        watched_callbacks_bool_.push_back(watched);
        update_watched_callbacks(watched_callbacks_bool_);
        return ros_sub;
    }

    typename rclcpp::GenericSubscription::SharedPtr add_subscription(
        const std::string &topic, const std::string &type, const rclcpp::QoS &qos,
        std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback,
        std::function<void(bool, std::chrono::milliseconds)> timeout_callback, std::chrono::milliseconds timeout,
        bool watched = true) {
        auto [timeout_descriptor, wrapped_callback] =
            create_timeout_descriptor(timeout_callback, timeout, std::move(callback));

        auto ros_sub = rclcpp::create_generic_subscription(
            node_->get_node_topics_interface(), topic, type, qos,
            [wrapped_callback](std::shared_ptr<rclcpp::SerializedMessage> msg) { wrapped_callback(msg); });

        created_ros_subs_.push_back(ros_sub);
        watched_callbacks_bool_.push_back(watched);
        update_watched_callbacks(watched_callbacks_bool_);
        return ros_sub;
    }

    TopicStatus get_overall_status() const {
        if (std::any_of(watched_callbacks_.begin(), watched_callbacks_.end(),
                        [](const std::shared_ptr<TimeoutDescriptor> &descriptor) {
                            return descriptor->status == TopicStatus::NOT_RECEIVED;
                        })) {
            return TopicStatus::NOT_RECEIVED;
        } else if (std::any_of(watched_callbacks_.begin(), watched_callbacks_.end(),
                               [](const std::shared_ptr<TimeoutDescriptor> &descriptor) {
                                   return descriptor->status == TopicStatus::ERROR;
                               })) {
            return TopicStatus::ERROR;
        } else if (std::any_of(watched_callbacks_.begin(), watched_callbacks_.end(),
                               [](const std::shared_ptr<TimeoutDescriptor> &descriptor) {
                                   return descriptor->status == TopicStatus::WARN;
                               })) {
            return TopicStatus::WARN;
        } else {
            return TopicStatus::OK;
        }
    }

    void update_watched_callbacks(std::vector<bool> watched_callbacks) {
        watched_callbacks_.clear();
        for (size_t i = 0; i < watched_callbacks.size(); ++i) {
            if (watched_callbacks[i]) {
                watched_callbacks_.emplace_back(all_callbacks_[i]);
            }
        }
    }

    using SharedPtr = std::shared_ptr<tod_topic_watchdog::TopicWatchdog>;
    using UniquePtr = std::unique_ptr<tod_topic_watchdog::TopicWatchdog>;
};
}  // namespace tod_topic_watchdog