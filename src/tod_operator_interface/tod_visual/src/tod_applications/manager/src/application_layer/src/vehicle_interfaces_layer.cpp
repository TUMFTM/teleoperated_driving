#include "vehicle_interfaces_layer.hpp"

bool VehicleInterfacesLayer::emergencyBreakReleased = false;
bool VehicleInterfacesLayer::long_released = false;
bool VehicleInterfacesLayer::lat_released = false;

VehicleInterfacesLayer::VehicleInterfacesLayer(std::shared_ptr<tod_gl::RosInterface> ros, ImGuiDir split_dir)
    :  OperatorManagerDockingLayer(ros, split_dir), 
    sections({
        {"Emergency Break Released", &emergencyBreakReleased},
        {"Long Released", &long_released},
        {"Lat Released", &lat_released}
    }), _status(ros) {
    _name = "VehicleInterfacesLayer";
        split_direction = split_dir;
}

void VehicleInterfacesLayer::on_attach() {}

void VehicleInterfacesLayer::on_detach() {}

void VehicleInterfacesLayer::on_im_gui_render() {
    emergencyBreakReleased = _status.is_emergency_stop_released();
    long_released = _status.is_long_approved();
    lat_released = _status.is_lat_approved();
     ImGui::Begin(_dock_space_window_name.c_str());
    ImGui::Begin(_name.c_str());

    for (const auto& section : sections) {

        ImVec4 color = *section.trigger ? ImVec4(0.0f, 1.0f, 0.0f, 1.0f) : ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_Button, color);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, color);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, color);


        ImGui::Button("##Cube", ImVec2(30, 30));
        ImGui::PopStyleColor(3);



        ImGui::SameLine(50);  
        ImGui::Text("%s", section.label); 

    }

    ImGui::End();
    ImGui::End();
}

void VehicleInterfacesLayer::on_event(tod_gl::Event& e) {
    (void)e;
}

void VehicleInterfacesLayer::on_update(float ts) {glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
}
