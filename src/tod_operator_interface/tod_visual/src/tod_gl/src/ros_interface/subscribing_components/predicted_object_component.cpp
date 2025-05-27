/**
 * @file predicted_object_component.cpp
 * @brief Predicted component that manages the subscription and the data for PredictedObject topics such as bounding boxes and predicted trajectories of the objects 
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/ros_interface/subscribing_components/predicted_object_component.hpp"

namespace tod_gl {

void PredictedObjectComponent::cb_message(const tod_automation_msgs::msg::PredictedObjects::SharedPtr msg) {
     objects_ = msg->objects; 
};

}  // namespace tod_gl