/**
 * @file ui_layer.hpp
 * @brief Base class for all UI layers in the visualization framework. 
 *        Provides an interface for rendering UI elements and handling events.
 * @copyright 2024 TUMFTM
**/

#pragma once

#include "view_port_layer.hpp"

#include "tod_gl/layers/imgui_scene_layer.hpp"

namespace tod_visual {
/**
 * @class UILayer
 * @brief Base class for UI elements that interact with the viewport and scene.
 */
class UILayer : public tod_gl::ImGuiSceneLayer {
  public:
    UILayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene, ViewPortLayer* vpl);
    ~UILayer() = default;

    virtual void on_attach() override;
    virtual void on_detach() override;
    virtual void on_im_gui_render() override;
    virtual void on_event(tod_gl::Event& e) override;
    virtual void on_update(float ts) override;

  private:
    float velocity = 0;

  protected:
    ViewPortLayer* view_port_layer;
};
}  // namespace tod_visual