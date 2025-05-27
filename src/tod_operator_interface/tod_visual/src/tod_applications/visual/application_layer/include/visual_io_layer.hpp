/**
 * @file visual_io_layer.hpp
 * @brief All subscriptions of the ROS Node and the containers in @ref tod_gl_ros_interface are registered in this layer from the SubscriptionManager entity and can be accessed via Entt anywhere in the application
 * @copyright 2024 TUMFTM
 **/

#pragma once

#include "tod_gl/core/scene_layer.hpp"

/**
 * All the Subscribing/Publishing components should be handled under this layer
 */
namespace tod_visual {

class VisualIOLayer : public tod_gl::SceneLayer {
  public:
    VisualIOLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene);
    ~VisualIOLayer() = default;

    virtual void on_attach() override;
};

}  // namespace tod_visual