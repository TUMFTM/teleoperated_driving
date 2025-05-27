/**
 * @file settings_layer.cpp
 * @brief Provides a layer to manage and render application settings within an ImGui interface.
 * @copyright 2024 TUMFTM
**/

#include "settings_layer.hpp"

#include <fstream>

#include "yaml-cpp/yaml.h"

namespace tod_visual {

SettingsLayer::SettingsLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene,
                             ImGuiDir split_dir)
    : DockingSceneLayer(ros, scene, split_dir) {
    _name = "SettingsLayer";
}

void SettingsLayer::on_im_gui_render() {
    ImGui::Begin(_dock_space_window_name.c_str());
    ImGui::Begin(_name.c_str());
    auto& stateManager = tod_gl::StateManager::get_instance();
    for (auto& it : stateManager.get_toggle_settings()) {
        bool value = it.second;
        if (ImGui::Button((it.first + ": " + (value ? "ON" : "OFF")).c_str())) {
            value = !value;
            it.second = value;
        }
        stateManager.set_toggle_setting(it.first, it.second);
    }
    ImGui::End();
    ImGui::End();
}

void SettingsLayer::on_event(tod_gl::Event& e) {
    ImGuiSceneLayer::on_event(e);
}

}  // namespace tod_visual