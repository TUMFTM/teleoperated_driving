/**
 * @file vehicle_state_layer.hpp
 * @ingroup tod_visual_application
 * @brief Defines the Vehicle State Layer for the application.
 * 
 * This layer handles the graphical user interface (GUI) and data updates 
 * for monitoring the state of the vehicle, including network metrics 
 * and status information.
 * 
 * @copyright 2024 TUMFTM
 */
#pragma once
#include <string>
#include <filesystem> 

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include "tod_gl/layers/imgui_layer.hpp"
#include "tinyfiledialogs.h"
#include "tinyfiledialogs.h"
#include "tod_gl/ros_interface/subscribing_components/tod_status_component.hpp"
#include "tod_gl/ros_interface/subscribing_components/network_metrics_component.hpp"

#include "operator_manager_docking_layer.hpp"


/**
 * @class vehicle_state_layer
 * @brief Represents the vehicle state layer in the application.
 * 
 * This layer manages the GUI elements for displaying the state of the vehicle,
 * including network and status metrics.
 */
class VehicleStateLayer : public tod_gl::OperatorManagerDockingLayer {
  public:
    VehicleStateLayer(std::shared_ptr<tod_gl::RosInterface> _ros, ImGuiDir split_dir);
    ~VehicleStateLayer() = default;

    void on_attach() override;
    void on_detach() override;
    void on_im_gui_render() override;
    void on_event(tod_gl::Event& e) override;
    void on_update(float ts) override;
    std::vector<std::pair<std::string, std::string>> label_value_pairs;
private:
    bool _bandwidth_tested; 
    tod_gl::TodStatusComponent _status; 
    tod_gl::NetworkMetricsComponent _network;
    void update_values();

};
