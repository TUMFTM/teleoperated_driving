/**
 * @file vehicle_state_layer.cpp
 * @brief TODO: Brief
 * @copyright 2024 TUMFTM
 **/

#include "vehicle_state_layer.hpp"

VehicleStateLayer::VehicleStateLayer(std::shared_ptr<tod_gl::RosInterface> _ros, ImGuiDir split_dir)
    :  OperatorManagerDockingLayer(_ros, split_dir), 
    _bandwidth_tested(false), _status(_ros), _network(_ros){
    _name = "VehicleStateLayer";
    // Labels und Werte als Paare initialisieren
    label_value_pairs = {
        {"gps/fix [hz]", "0"},
        {"odometry [hz]", "0"},
        {"nav_status", "Unknown"},
        {"pos_type", "Unknown"},
        {"latency [ms]", "0"},
        {"link_quality", "0"},
        {"TX [packets/s]", "0"},
        {"TX [Mbit/s]", "0"},
        {"RX [packets/s]", "0"},
        {"RX [Mbit/s]", "0"},
        {"upload [Mbit/s]", "0"},
        {"download [Mbit/s]", "0"}
    };
    split_direction = split_dir;
}

void VehicleStateLayer::on_attach() {}

void VehicleStateLayer::on_detach() {}

void VehicleStateLayer::on_im_gui_render() {
    update_values();
    ImGui::Begin(_dock_space_window_name.c_str());

    ImGui::Begin(_name.c_str());

    float label_column_width = 150.0f; // Breite für Labels

    // Zeichne Labels und Werte bis "upload"
    for (const auto& pair : label_value_pairs) {
        const std::string& label = pair.first;
        const std::string& value = pair.second;

        if (label == "upload [Mbit/s]") {
            // Zeichne den Button "Test Bandwidth"
            if (ImGui::Button("Test Bandwidth")) {
                _bandwidth_tested = true;
            }

            if (!_bandwidth_tested) {
                break; // Zeige keine weiteren Items, wenn der Test nicht ausgeführt wurde
            }
        }

        // Zeichne Label
        ImGui::Text("%s:", label.c_str());
        ImGui::SameLine(label_column_width);

        // Zeichne Wert
        ImGui::Text("%s", value.c_str());
    }

    ImGui::End();
    ImGui::End();
}

void VehicleStateLayer::on_event(tod_gl::Event& e) {
    (void)e;
}

void VehicleStateLayer::on_update(float ts) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
}

void VehicleStateLayer::update_values() {
    label_value_pairs = {
        {"gps/fix [hz]", "0"},
        {"odometry [hz]", "0"},
        {"nav_status", _status.get_vehicle_nav_status().empty() ? "Unknown" : _status.get_vehicle_nav_status()},
        {"pos_type", _status.get_vehicle_gps_pos_type().empty() ? "Unknown" : _status.get_vehicle_gps_pos_type()},
        {"latency [ms]", std::to_string(static_cast<int>(_network.get_latency() * 100) / 100.0)}, // limit precision to two decimals
        {"link_quality", std::to_string(static_cast<int>(_network.get_link_quality() * 100)) }, 
        {"TX [packets/s]", std::to_string(static_cast<int>(_network.get_tx_packets_per_second() ))},
        {"TX [Mbit/s]", std::to_string(_network.get_tx_bitrate_mbps())},
        {"RX [packets/s]", std::to_string(static_cast<int>(_network.get_rx_packets_per_second() ))},
        {"RX [Mbit/s]", std::to_string(_network.get_tx_bitrate_mbps() )},
        {"upload [Mbit/s]", "0"},
        {"download [Mbit/s]", "0"}
    };
}