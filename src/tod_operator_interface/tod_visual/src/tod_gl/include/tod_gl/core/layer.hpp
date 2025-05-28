/**
 * @file layer.hpp
 * @brief Base abstraction for the dearImGui layer architecture. Is part of the application
 * @copyright 2024 TUMFTM based on Cherno's Hazel
 */

#pragma once

#include <string>
#include <memory>

#include "tod_gl/events/event.hpp"
#include "tod_gl/ros_interface/ros_interface.hpp"

/**
 * @brief Layer is an abstraction for having multiple operation going under the Application level on parallel.
 * We can have a Main Layer, UI Layer, DebugLayer, etc.
 * They all need to be registered into the LayerStack under Application
 */
namespace tod_gl {

class Layer {
  public:
    Layer(const std::string& name = "Layer");
    Layer(std::shared_ptr<RosInterface> ros, const std::string& name = "Layer");
    virtual ~Layer() = default;

    virtual void on_attach() {}
    virtual void on_detach() {}
    virtual void on_update(float ts) {}
    virtual void on_im_gui_render() {}
    virtual void on_event(Event& event) {}

    const std::string& get_name() const { return _name; }

  protected:
    std::string _name;
    std::shared_ptr<RosInterface> _ros;
};

}  // namespace tod_gl