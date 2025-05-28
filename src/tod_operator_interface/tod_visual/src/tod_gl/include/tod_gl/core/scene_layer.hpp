/**
 * @file scene_layer.hpp
 * @brief Scene Abstraction of the layer, i.e. if you have a scene application @ref SceneApplication then the layers used will have access to the 3D scenes content
 * @copyright 2024 TUMFTM based on Cherno's Hazel
 */

#pragma once

#include <string>
#include <memory>

#include "tod_gl/events/event.hpp"
#include "tod_gl/ros_interface/ros_interface.hpp"
#include "tod_gl/scene/scene.hpp"

/**
 * SceneLayer is an abstraction for having multiple operation going under the Application level on parallel.
 * We can have a Main SceneLayer, UI SceneLayer, DebugLayer, etc.
 * They all need to be registered into the LayerStack under Application
 */
namespace tod_gl {

class SceneLayer {
  public:
    SceneLayer(const std::string& name = "SceneLayer");
    SceneLayer(std::shared_ptr<RosInterface> ros, std::shared_ptr<Scene> scene, const std::string& name = "SceneLayer");
    virtual ~SceneLayer() = default;

    virtual void on_attach() {}
    virtual void on_detach() {}
    virtual void on_update(float ts) {}
    virtual void on_im_gui_render() {}
    virtual void on_event(Event& event) {}

    const std::string& get_name() const { return _name; }

  protected:
    std::string _name;

    std::shared_ptr<RosInterface> _ros;
    std::shared_ptr<Scene> _active_scene;
};
}  // namespace tod_gl