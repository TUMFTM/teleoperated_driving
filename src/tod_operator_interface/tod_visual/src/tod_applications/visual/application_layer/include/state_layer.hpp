/**
 * @file state_layer.hpp
 * @brief Manages ToD states and handles subscribing/publishing components for the scene
 *        within the ToD visualization context.
 * @copyright 2024 TUMFTM
 */

/**
 * @class StateLayer
 * @brief Manages the operator control mode and other ToD states, updating the global StateManager
 *        based on subscribed status components.
 */


#pragma once

#include "trajectory_guidance_state_layer.hpp"

#include "tod_gl/core/state_manager.hpp"
#include "tod_gl/core/scene_layer.hpp"
#include "tod_gl/ros_interface/subscribing_components/tod_status_component.hpp"

#include "tod_core/param_set/CameraParameters.hpp"

/**
 * All the Subscribing/Publishing components should be handled under this layer
 */
namespace tod_visual {
class StateLayer : public tod_gl::SceneLayer {
  public:
    StateLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene);
    ~StateLayer() = default;

    virtual void on_attach() override;

    virtual void on_update(float ts) override {
        auto& stateManager = tod_gl::StateManager::get_instance();
        tod_gl::Entity SubscriptionManager = _active_scene->find_entity_with_tag("SubscriptionManager");
        if (SubscriptionManager.has_component<tod_gl::TodStatusComponent>()) {
            auto& statusComp = SubscriptionManager.get_component<tod_gl::TodStatusComponent>();
            stateManager.set_state(statusComp.get_operator_control_mode());
        }
    }

  private:
    std::unique_ptr<tod_core::param_set::Camera> _cam_params;
};
}  // namespace tod_visual