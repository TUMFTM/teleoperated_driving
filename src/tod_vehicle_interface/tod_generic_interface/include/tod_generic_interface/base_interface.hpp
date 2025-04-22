/**
 * @file base_interface.hpp
 * @brief Base class for all generic interfaces.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_generic_interface
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "rclcpp/serialized_message.hpp"

#include <unordered_map>
#include <map>

namespace tod_generic_interface {
/**
 * @ingroup tod_generic_interface
 * @brief Base classes for generic interfaces.
 */

/**
 * @brief Time stamped attribute class for base interface.
 */
class StampedAttributeBase 
{
    public:
        rclcpp::Time timestamp;

        StampedAttributeBase(rclcpp::Time ts) : timestamp(ts) {}
        virtual ~StampedAttributeBase() = default;
};

/**
 * @brief Child class specific attribute types so that the timestamp is accessable without type information.
 */
template <typename T>
class StampedAttribute : public StampedAttributeBase 
{
    public:
        T attribute;
        StampedAttribute(T attr, rclcpp::Time ts)
            : StampedAttributeBase(ts), attribute(attr) {}
};

/**
 * @brief Base class for generic interfaces.
 */
class BaseInterface
{
    public:
        BaseInterface(rclcpp::Node::SharedPtr node);

        template <typename T>
        void add_attribute(const std::string& attribute_name, T attribute_value) 
        {
            auto stamped_attr = std::make_shared<StampedAttribute<T>>(attribute_value, clock_.now());
            attributes_[attribute_name] = stamped_attr;
        };

        template <typename T>
        T get_attribute(const std::string& attribute_name) const 
        {
            auto it = attributes_.find(attribute_name);
            if (it == attributes_.end()) {
                throw std::runtime_error("Attribute not found: " + attribute_name);
            }
            auto stamped_attr = std::dynamic_pointer_cast<StampedAttribute<T>>(it->second);
            if (!stamped_attr) {
                throw std::runtime_error("Type mismatch for attribute: " + attribute_name);
            }
            return stamped_attr->attribute;
        };

        template <typename T>
        void update_attribute(const std::string& attribute_name, T attribute_value) 
        {
            rclcpp::Time current_time = clock_.now(); // Get the current time
            auto it = attributes_.find(attribute_name);
            if (it == attributes_.end()) {
                // Add the attribute if it doesn't exist
                add_attribute(attribute_name, attribute_value);
            } else {
                // Update the attribute
                auto stamped_attr = std::dynamic_pointer_cast<StampedAttribute<T>>(it->second);
                if (!stamped_attr) {
                    throw std::runtime_error(
                        "Type mismatch for attribute '" + attribute_name + 
                        "': Expected type '" + typeid(*it->second).name() + 
                        "', but provided type '" + typeid(T).name() + "'."
                    );
                }
                stamped_attr->attribute = attribute_value;
                stamped_attr->timestamp = clock_.now();
            }
        };

        bool has_update(rclcpp::Time timestamp, const std::vector<std::string>& attribute_names) const;

        template <typename MessageType>
        void add_publisher(const std::string& topic_name, 
                        std::function<MessageType()> message_builder,
                        const std::vector<std::string>& attribute_names,
                        const int refresh_time) 
        {
            typename rclcpp::Publisher<MessageType>::SharedPtr publisher = node_->create_publisher<MessageType>(topic_name, 1);
            publishers_[topic_name] = publisher;
            std::shared_ptr<rclcpp::Time> last_check_time = std::make_shared<rclcpp::Time>(clock_.now());
            std::function<void()> timer_callback = [this, publisher, message_builder, attribute_names, last_check_time]() mutable {
                // RCLCPP_INFO(this->node_->get_logger(), "inside the callback");
                if (this->has_update(*last_check_time, attribute_names)) {
                    *last_check_time = clock_.now(); // Update the time via the shared pointer
                    auto message = message_builder();
                    
                    publisher->publish(message);
                    // RCLCPP_INFO(this->node_->get_logger(), "published");
                }
                else{
                    // RCLCPP_INFO(this->node_->get_logger(), "no update");
                }
            };
            rclcpp::TimerBase::SharedPtr timer = node_->create_wall_timer(std::chrono::milliseconds(refresh_time), timer_callback);
            publisher_timers_[topic_name] = timer;
            RCLCPP_INFO(node_->get_logger(), "Publisher created on topic: %s", topic_name.c_str());
        };

        template <typename MessageType>
        void add_subscriber(const std::string& topic_name,
                        std::function<void(const MessageType&)> message_handler) 
        {
            typename rclcpp::Subscription<MessageType>::SharedPtr subscriber =
                node_->create_subscription<MessageType>(
                    topic_name,
                    1,
                    [this, message_handler](const typename MessageType::SharedPtr msg) {
                        message_handler(*msg);
                    });
            subscribers_[topic_name] = subscriber;
            RCLCPP_INFO(node_->get_logger(), "Subscriber created on topic: %s", topic_name.c_str());
        };

        void add_forwarder(const std::string& input_topic, const std::string& output_topic, const std::string& message_type);

        rclcpp::Clock& get_clock() { return clock_; }

    private:
        std::map<std::string, std::shared_ptr<StampedAttributeBase>> attributes_;
        rclcpp::Clock clock_;
        rclcpp::Node::SharedPtr node_;

        std::map<std::string, rclcpp::GenericSubscription::SharedPtr> forwarder_subscribers_;
        std::map<std::string, rclcpp::GenericPublisher::SharedPtr> forwarder_publishers_;
        std::map<std::string, rclcpp::PublisherBase::SharedPtr> publishers_;
        std::map<std::string, rclcpp::TimerBase::SharedPtr> publisher_timers_;
        std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscribers_;
};

} // namespace tod_generic_interface