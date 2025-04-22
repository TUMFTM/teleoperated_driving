/**
 * @file base_interface.cpp
 * @brief Base class for all generic interfaces.
 * @copyright 2024 TUM-FTM
 * @ingroup tod_generic_interface
 */

#include "tod_generic_interface/base_interface.hpp"

namespace tod_generic_interface {

BaseInterface::BaseInterface(rclcpp::Node::SharedPtr node)
        : node_(node), clock_(*node->get_clock()) { }

bool BaseInterface::has_update(rclcpp::Time timestamp, const std::vector<std::string>& attribute_names) const 
{
    if (attribute_names.empty()) {
        return true;    // Return true if the vector is empty
    }
    for (const auto& name : attribute_names) {
        auto it = attributes_.find(name);
        if (it != attributes_.end()) {
            if (it->second->timestamp > timestamp) {
                return true;
            }
        }
    }

    // // Log the comparison of timestamps for all checked attributes
    // RCLCPP_INFO(node_->get_logger(), "No update detected. Checking timestamps:");
    // for (const auto& name : attribute_names) {
    //     auto it = attributes_.find(name);
    //     if (it != attributes_.end()) {
    //         RCLCPP_INFO(node_->get_logger(),
    //                     "Attribute '%s' -> Attribute timestamp: %d.%d, Function timestamp: %d.%d",
    //                     name.c_str(),
    //                     it->second->timestamp.seconds(), it->second->timestamp.nanoseconds(),
    //                     timestamp.seconds(), timestamp.nanoseconds());
    //     } else {
    //         RCLCPP_WARN(node_->get_logger(), "Attribute '%s' not found.", name.c_str());
    //     }
    // }
    return false;
}

void BaseInterface::add_forwarder(const std::string& input_topic, const std::string& output_topic, const std::string& message_type) 
{
    if (input_topic == output_topic) {
        throw std::runtime_error("Input and output topics cannot be the same: " + input_topic);
    }
    auto publisher = node_->create_generic_publisher(
        output_topic,
        message_type,
        rclcpp::SystemDefaultsQoS()
    );
    forwarder_publishers_[output_topic] = publisher;

     auto subscription = node_->create_generic_subscription(
        input_topic,
        message_type,
        rclcpp::SystemDefaultsQoS(),
        [this, publisher, output_topic](std::shared_ptr<rclcpp::SerializedMessage> msg) {
            publisher->publish(*msg);
        }
    );

    RCLCPP_INFO(node_->get_logger(), "Forwarding from '%s' to '%s'", input_topic.c_str(), output_topic.c_str());
    forwarder_subscribers_[input_topic] = subscription;
}

} // namespace tod_generic_interface