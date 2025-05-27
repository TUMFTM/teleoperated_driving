/**
 * @file docking_layer.cpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/layers/docking_layer.hpp"


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

DockingLayer::DockingLayer(std::shared_ptr<RosInterface> ros, ImGuiDir split_dir)
    : ImGuiLayer(ros) {
    _name = "DockingLayer";
    split_direction = split_dir;
}

void DockingLayer::on_attach() {
    should_update_layout = true;
}

void DockingLayer::on_detach() {
    should_update_layout = true;
}

void DockingLayer::on_im_gui_render() {
    static bool opt_fullscreen = true;
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

void DockingLayer::on_event(Event& e) {
    ImGuiLayer::on_event(e);
    if (e.get_event_type() == EventType::WindowResize) {
        should_update_layout = true;
    }

    if (e.get_event_type() != EventType::LayerStackChanged)
        return;

    LayerStackChangedEvent event = static_cast<LayerStackChangedEvent&>(e);

    auto* layer = dynamic_cast<DockingLayer*>(event.get_layer());
    if (!layer || layer == this)
        return;

    if (event.is_deleted()) {
        _layers_to_dock.erase(std::remove(_layers_to_dock.begin(), _layers_to_dock.end(), layer), _layers_to_dock.end());
    } else {
        _layers_to_dock.push_back(layer);
    }

    should_update_layout = true;
}

void DockingLayer::update_layout() {
    ImGuiID dockspace_id = ImGui::GetID(_dock_space_name.c_str());

    ImGui::DockBuilderRemoveNode(dockspace_id);                             // Clear out existing layout
    ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);  // Add empty node

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->WorkSize);

    ImGuiID dock_main_id = dockspace_id;
    ImGuiID dock_id_new = dockspace_id;

    struct DockDirectionInfo {
        ImGuiID lastDocked = 0;
        int count = 0;
        int processed = 0;
        std::string layerName;
    };

    std::map<ImGuiDir, DockDirectionInfo> dockDirections = {};

    // first pass to get the global info we need.
    for (DockingLayer* layer : _layers_to_dock) {
        // Niklas: ugly
        if (layer->is_video_layer() && !show_videos)
            continue;

        if (dockDirections.find(layer->split_direction) == dockDirections.end()) {
            dockDirections[layer->split_direction] = DockDirectionInfo{dock_main_id, 0, 0};
        }
        dockDirections[layer->split_direction].count++;
    }

    // second pass to set the layout
    for (DockingLayer* layer : _layers_to_dock) {
        if (layer->is_video_layer() && !show_videos)
            continue;

        const ImGuiDir direction = layer->split_direction;
        DockDirectionInfo& info = dockDirections[direction];

        if (info.processed == 0) {
            // split only if direction specified otherwise just take the center node.

            dock_id_new = direction != ImGuiDir_None
                              ? ImGui::DockBuilderSplitNode(info.lastDocked, direction, 0.3f, NULL, &dock_main_id)
                              : dock_main_id;
            info.lastDocked = dock_id_new;
            info.processed++;
        } else {
            // if we're adding to top, we want to layout to the opposite orientation (vertical, horizontal)
            // default first choice is to the right and to the top.
            ImGuiDir opposite_dir = direction < 2 ? ImGuiDir_Up : ImGuiDir_Right;

            float ratioForOneCell = 1.f / (info.count - info.processed + 1);
            // Since we split for the new cell we need to consider future cells for the ratio
            // ex. 3 cells and we're building the second, ratioForOneCell is 1/3 and the second should take 2/3 of the
            // space.
            float ratioForTheRest = 1.f - ratioForOneCell;

            dock_id_new =
                ImGui::DockBuilderSplitNode(info.lastDocked, opposite_dir, ratioForTheRest, NULL, &dock_main_id);
            info.lastDocked = dock_id_new;
            info.processed++;
        }
        ImGui::DockBuilderDockWindow(layer->get_name().c_str(), dock_id_new);
    }

    ImGui::DockBuilderFinish(dockspace_id);
}

}  // namespace tod_gl