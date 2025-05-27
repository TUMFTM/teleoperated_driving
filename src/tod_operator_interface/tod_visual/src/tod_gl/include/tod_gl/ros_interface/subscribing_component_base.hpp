/**
 * @file subscribing_component_base.hpp
 * @brief Base class for subscribing components.
 * @copyright TUMFTM 2024
 **/

#pragma once

#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"

namespace tod_gl {

template<typename MessageType>
class SubscribingComponent {
  public:
    explicit SubscribingComponent(std::shared_ptr<rclcpp::Node> sub_node, std::string topic_name) {
        subscription_ = sub_node->create_subscription<MessageType>(
            topic_name, 1, [this](const typename MessageType::SharedPtr msg) {
              this->cb_message(msg);
          });
    }
  private:
    typename rclcpp::Subscription<MessageType>::SharedPtr subscription_;
    virtual void cb_message(const typename MessageType::SharedPtr msg) { }
};

}  // namespace tod_gl