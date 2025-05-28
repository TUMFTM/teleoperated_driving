/**
 * @file vehicle_interfaces_layer.hpp
 * @ingroup tod_visual_application
 * @brief Defines the Vehicle Interfaces Layer for the application.
 * 
 * This class provides the GUI and functionality for interacting with 
 * vehicle interfaces, such as releasing emergency brakes and monitoring statuses.
 * 
 * @copyright 2024 TUMFTM
 */

#pragma once

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include "tod_gl/layers/imgui_layer.hpp"
#include "tinyfiledialogs.h"
#include <string>
#include <filesystem> 
#include "tinyfiledialogs.h"
#include "tod_gl/ros_interface/subscribing_components/tod_status_component.hpp"

#include "operator_manager_docking_layer.hpp"
/**
 * @struct Section
 * @brief Represents a GUI section with a label and trigger flag.
 */
struct Section {
        const char* label;
        bool* trigger;
    };


/**
 * @class vehicle_interfaces_layer
 * @brief Represents the vehicle interfaces layer in the application.
 * 
 * This layer handles the GUI elements for vehicle interface management,
 * .
 */
class VehicleInterfacesLayer : public tod_gl::OperatorManagerDockingLayer{
  public:
    VehicleInterfacesLayer(std::shared_ptr<tod_gl::RosInterface> ros, ImGuiDir split_dir);
    ~VehicleInterfacesLayer() = default;

    void on_attach() override;
    void on_detach() override;
    void on_im_gui_render() override;
    void on_event(tod_gl::Event& e) override;
    void on_update(float ts) override;
    static bool emergencyBreakReleased;
    static bool long_released;
    static bool lat_released;
    std::vector<Section> sections;
  private:
    tod_gl::TodStatusComponent _status; 


};
