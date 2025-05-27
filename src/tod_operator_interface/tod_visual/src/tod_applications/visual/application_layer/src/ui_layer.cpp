/**
 * @file ui_layer.cpp
 * @brief Base class for all UI layers in the visualization framework. 
 *        Provides an interface for rendering UI elements and handling events.
 * @copyright 2024 TUMFTM
**/

#include "ui_layer.hpp"

#include "tod_gl/ros_interface/subscribing_components/odometry_component.hpp"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

namespace tod_visual {

UILayer::UILayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene,
                 ViewPortLayer* view_port_layer)
    : tod_gl::ImGuiSceneLayer(ros, scene), view_port_layer(view_port_layer) {
    _name = "UILayer";
}

void UILayer::on_attach() {}

void UILayer::on_detach() {}

void UILayer::on_im_gui_render() {
    ImGui::Begin("UI", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoScrollbar);

    auto &odometry = _active_scene->find_entity_with_tag("SubscriptionManager").get_component<tod_gl::OdometryComponent>();
    const std::string SpeedText = "Speed: " + std::to_string(glm::length(odometry.get_linear_velocities()));

    ImGui::Text(SpeedText.c_str());

    ImGui::End();
}

void UILayer::on_event(tod_gl::Event& e) {
    (void)e;
}

void UILayer::on_update(float ts) {}

}  // namespace tod_visual