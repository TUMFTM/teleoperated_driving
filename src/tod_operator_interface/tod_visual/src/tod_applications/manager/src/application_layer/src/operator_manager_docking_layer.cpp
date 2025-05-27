/**
 * @file operator_manager_docking_layer.cpp
 * @brief Implementation of the OperatorManagerDockingLayer class for dynamic UI docking layout management.
 * 
 * This file contains the implementation of the OperatorManagerDockingLayer class, which provides functionality 
 * for managing and updating the dynamic docking layout of the application's user interface. It defines 
 * methods to attach/detach layers, render UI elements, handle events, and update the docking layout.
 * 
 * @copyright 2024 
 * TUMFTM. All rights reserved.
 */

#include "operator_manager_docking_layer.hpp"


#include <map>
#include <algorithm>

#include "tod_gl/events/application_event.hpp"
#include "tod_gl/events/event.hpp"
#include "tod_gl/events/layer_event.hpp"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_internal.h"

namespace tod_gl {

OperatorManagerDockingLayer::OperatorManagerDockingLayer(std::shared_ptr<RosInterface> ros, ImGuiDir split_dir)
    : ImGuiLayer(ros) {
    _name = "OperatorManagerDockingLayer";
    split_direction = split_dir;
}

void OperatorManagerDockingLayer::on_attach() {
    should_update_layout = true;
    
}

void OperatorManagerDockingLayer::on_detach() {
    should_update_layout = true;
}

void OperatorManagerDockingLayer::on_im_gui_render() {
    static bool opt_fullscreen = true;
   // RCLCPP_INFO(_ros->get_logger(), "Wo Layer Render?");
    static bool opt_padding = false;
    static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None;
    bool dockOpen = true;

    // We are using the ImGuiWindowFlags_NoDocking flag to make the parent window not dockable into,
    // because it would be confusing to have two docking targets within each others.
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    window_flags |=
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    window_flags |=
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_HorizontalScrollbar;

    // When using ImGuiDockNodeFlags_PassthruCentralNode, DockSpace() will render our background
    // and handle the pass-thru hole, so we ask Begin() to not render a background.
    if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
        window_flags |= ImGuiWindowFlags_NoBackground;

    ImGui::Begin(_dock_space_window_name.c_str(), &dockOpen, window_flags);

    if (opt_fullscreen)
        ImGui::PopStyleVar(2);

    // Submit the DockSpace
    ImGuiIO& io = ImGui::GetIO();

    if (io.ConfigFlags & ImGuiConfigFlags_DockingEnable) {
        ImGuiID dockspace_id = ImGui::GetID(_dock_space_name.c_str());
        ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);
    }

    if (should_update_layout) {
        update_layout();
        should_update_layout = false;
    }

    ImGui::End();  // dockspace
}

void OperatorManagerDockingLayer::on_event(Event& e) {
    ImGuiLayer::on_event(e);
    if (e.get_event_type() == EventType::WindowResize) {
        should_update_layout = true;
    }

    if (e.get_event_type() != EventType::LayerStackChanged)
        return;

    LayerStackChangedEvent event = static_cast<LayerStackChangedEvent&>(e);

    auto* layer = dynamic_cast<OperatorManagerDockingLayer*>(event.get_layer());
    if (!layer || layer == this)
        return;

    if (event.is_deleted()) {
        _layers_to_dock.erase(std::remove(_layers_to_dock.begin(), _layers_to_dock.end(), layer), _layers_to_dock.end());
    } else {
        _layers_to_dock.push_back(layer);
    }

    should_update_layout = true;
}
void OperatorManagerDockingLayer::update_layout() {
    ImGuiID dockspace_id = ImGui::GetID(_dock_space_name.c_str());


    ImGui::DockBuilderRemoveNode(dockspace_id);
    ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->WorkSize);



    ImGuiID dock_left, dock_right;
    ImGui::DockBuilderSplitNode(dockspace_id, ImGuiDir_Left, 0.5f, &dock_left, &dock_right); // 50:50 Spaltenlayout


    ImGuiID dock_right_top, dock_right_bottom;
    ImGui::DockBuilderSplitNode(dock_right, ImGuiDir_Up, 0.5f, &dock_right_top, &dock_right_bottom); // 50:50 vertikal



  
    int layer_index = 0; 
    for (OperatorManagerDockingLayer* layer : _layers_to_dock) {
        if (layer == nullptr) {
            RCLCPP_ERROR(_ros->get_logger(), "Null layer in _layers_to_dock!");
            continue;
        }

        // Fenster sequentiell an die Bereiche zuweisen
        if (layer_index == 0) {
            ImGui::DockBuilderDockWindow(layer->get_name().c_str(), dock_left);
        } else if (layer_index == 1) {
            ImGui::DockBuilderDockWindow(layer->get_name().c_str(), dock_right_top);
        } else if (layer_index == 2) {
            ImGui::DockBuilderDockWindow(layer->get_name().c_str(), dock_right_bottom);
        } else {
            RCLCPP_WARN(_ros->get_logger(), "Layer %s could not be docked. Max 3 layers supported in this layout.", layer->get_name().c_str());
        }
        layer_index++;
    }

    ImGui::DockBuilderFinish(dockspace_id);
}


}  // namespace tod_gl