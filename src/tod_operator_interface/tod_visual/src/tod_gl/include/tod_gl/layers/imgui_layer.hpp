/**
 * @file imgui_layer.hpp
 * @brief Integration between @ref Layer and the dearImGui Context. This class sets up the imgui context between application and layer
 * @copyright 2024 TUMFTM
 */

#pragma once

#include <memory>
#include <string>

#include "tod_gl/core/layer.hpp"

/*
 * MUST HAVE
 * The layer responsible by setting up the ImGui
 * There is nothing rendered on this layer.
 */
namespace tod_gl {

class ImGuiLayer : public Layer {
  public:
    ImGuiLayer(std::shared_ptr<RosInterface> ros);
    ~ImGuiLayer() = default;

    virtual void on_attach() override;
    virtual void on_detach() override;
    virtual void on_event(Event& e) override;

    void begin();
    void end();

    void block_events(bool block) { _block_events = block; }
    GLuint load_texture(const std::string& imagePath);

    void set_dark_theme_colors();

    uint32_t get_active_widget_ID() const;

  protected:
    bool _block_events = false;
};

}  // namespace tod_gl