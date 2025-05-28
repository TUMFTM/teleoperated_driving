/**
 * @file settings_layer.hpp
 * @brief Provides a layer to manage and render application settings within an ImGui interface.
 * @copyright 2024 TUMFTM
**/

#pragma once

#include "tod_gl/layers/docking_scene_layer.hpp"

#include "tod_gl/core/state_manager.hpp"

#include "imgui/imgui.h"

namespace tod_visual {

class SettingsLayer : public tod_gl::DockingSceneLayer {
  public:
    SettingsLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene, ImGuiDir split_dir);
    ~SettingsLayer() = default;

    virtual void on_im_gui_render() override;
    virtual void on_event(tod_gl::Event &e) override;
};

}  // namespace tod_visual