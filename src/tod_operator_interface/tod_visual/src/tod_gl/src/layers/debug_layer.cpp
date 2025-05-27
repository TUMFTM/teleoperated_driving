/**
 * @file devug_layer.cpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/layers/debug_layer.hpp"

#include <sys/sysinfo.h>

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#define IM_GREEN IM_COL32(0, 255, 0, 255)
#define IM_YELLOW IM_COL32(255, 255, 0, 255)
#define IM_RED IM_COL32(255, 0, 0, 255)

namespace tod_gl {

DebugLayer::DebugLayer(std::shared_ptr<RosInterface> ros) : ImGuiLayer(ros) {
    _name = "DebugLayer";
}

void DebugLayer::on_attach() {}

void DebugLayer::on_detach() {}

void DebugLayer::on_im_gui_render() {
    ImGui::Begin("Debug", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoScrollbar);

    const std::string fpsText = "FPS: " + std::to_string(_current_fps);
    auto color = (_current_fps < 30) ? IM_RED : (_current_fps < 60 ? IM_YELLOW : IM_GREEN);
    ImGui::PushStyleColor(ImGuiCol_Text, color);
    ImGui::Text(fpsText.c_str());
    ImGui::PopStyleColor();

    struct sysinfo memInfo;

    sysinfo(&memInfo);
    long long totalVirtualMem = memInfo.totalram;
    // Add other values in next statement to avoid int overflow on right hand side...
    totalVirtualMem += memInfo.totalswap;
    totalVirtualMem *= memInfo.mem_unit;
    totalVirtualMem = totalVirtualMem >> 20;

    long long virtualMemUsed = memInfo.totalram - memInfo.freeram;
    // Add other values in next statement to avoid int overflow on right hand side...
    virtualMemUsed += memInfo.totalswap - memInfo.freeswap;
    virtualMemUsed *= memInfo.mem_unit;
    virtualMemUsed = virtualMemUsed >> 20;

    const std::string memoryText =
        "Memory Used: " + std::to_string(virtualMemUsed) + " / " + std::to_string(totalVirtualMem);
    ImGui::Text(memoryText.c_str());
    ImGui::End();
}

void DebugLayer::on_event(Event &e) {
    (void)e;
}

void DebugLayer::on_update(float ts) {
    _current_fps = 1 / ts;
}

} // namespace tod_gl