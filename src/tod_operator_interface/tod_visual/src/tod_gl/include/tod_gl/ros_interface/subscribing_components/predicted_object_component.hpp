/**
 * @file predicted_object_component.hpp
 * @brief Predicted component that manages the subscription and the data for PredictedObject topics such as bounding boxes and predicted trajectories of the objects 
 * @copyright TUMFTM 2024
 **/

#pragma once

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "tod_gl/ros_interface/subscribing_component_base.hpp"

#include "tod_automation_msgs/msg/predicted_objects.hpp"

namespace tod_gl {
  
class PredictedObjectComponent : public SubscribingComponent<tod_automation_msgs::msg::PredictedObjects> {
  public:
    explicit PredictedObjectComponent(std::shared_ptr<rclcpp::Node> sub_node)
      : SubscribingComponent(sub_node, "input/predicted_objects")
    {}
    const std::vector<tod_automation_msgs::msg::PredictedObject>& get_objects() const { return objects_; };

  private:
    void cb_message(const tod_automation_msgs::msg::PredictedObjects::SharedPtr msg) override;
    std::vector<tod_automation_msgs::msg::PredictedObject> objects_;
};

}  // namespace tod_gl