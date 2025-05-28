/**
 * @file debug_layer.hpp
 * @brief dearImGui layer definition containing debug information about the application
 * @copyright 2024 TUMFTM
 */

#pragma once

#include <memory>

#include "tod_gl/layers/imgui_layer.hpp"

namespace tod_gl {

class DebugLayer : public ImGuiLayer {
  public:
    DebugLayer(std::shared_ptr<RosInterface> ros);
    ~DebugLayer() = default;

    virtual void on_attach() override;
    virtual void on_detach() override;
    virtual void on_im_gui_render() override;
    virtual void on_event(Event& e) override;
    virtual void on_update(float ts) override;

  private:
    float _current_fps = 0;
};

}  // namespace tod_gl