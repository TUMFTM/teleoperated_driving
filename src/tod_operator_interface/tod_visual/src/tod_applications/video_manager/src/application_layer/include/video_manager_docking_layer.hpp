/**
 * @file video_manager_docking_layer.hpp
 * @brief Defines the VideoManagerDockingLayer class for dynamic UI docking management in video applications.
 * 
 * This file declares the VideoManagerDockingLayer class, which provides functionality for managing 
 * dynamic layouts in the user interface through ImGui's docking system. Layers inheriting from this 
 * base class can define their specific docking behavior and layout updates.
 * 
 * @note Layers using this docking feature must inherit from VideoManagerDockingLayer.
 * 
 * Key Features:
 * - Dynamic layout management using ImGui's docking capabilities.
 * - Support for video and non-video layers.
 * - Flexible configuration of docking directions.
 * 
 * @copyright 2024 
 * TUMFTM. All rights reserved.
 */

#pragma once

#include <memory>
#include <vector>
#include <string>
#include "tod_gl/layers/imgui_layer.hpp"
#include "imgui/imgui.h"

/**
 * @namespace tod_gl
 * @brief Namespace for core components of the TUM teleoperation project.
 */
namespace tod_gl {

/**
 * @class VideoManagerDockingLayer
 * @brief Base class for managing dynamic docking layouts in the UI.
 * 
 * The VideoManagerDockingLayer class provides a foundation for layers that require dynamic 
 * UI layouts with ImGui's docking capabilities. Derived classes can customize their docking 
 * behavior and update layouts dynamically.
 */
class VideoManagerDockingLayer : public ImGuiLayer {
  public:
    /**
     * @brief Constructs a VideoManagerDockingLayer instance.
     * 
     * @param _ros Shared pointer to the RosInterface for ROS communication.
     * @param split_dir Direction in which the layer should dock (e.g., ImGuiDir_Left, ImGuiDir_Right).
     */
    VideoManagerDockingLayer(std::shared_ptr<RosInterface> _ros, ImGuiDir split_dir);

    /**
     * @brief Default destructor.
     */
    ~VideoManagerDockingLayer() = default;

    /**
     * @brief Called when the layer is attached to the application.
     * 
     * Initializes resources or sets up the state when the layer is activated.
     */
    virtual void on_attach() override;

    /**
     * @brief Called when the layer is detached from the application.
     * 
     * Cleans up resources or resets states when the layer is deactivated.
     */
    virtual void on_detach() override;

    /**
     * @brief Renders the ImGui elements for this layer.
     * 
     * Override this method to define the specific UI components and layout for the layer.
     */
    virtual void on_im_gui_render() override;

    /**
     * @brief Handles events for this layer.
     * 
     * @param e Reference to the event to handle.
     */
    virtual void on_event(Event& e) override;

    /**
     * @brief Flag to enable or disable layout updates.
     */
    bool should_update_layout = true;

    /**
     * @brief Specifies the direction for docking this layer.
     */
    ImGuiDir split_direction = ImGuiDir_None;

  protected:
    /**
     * @brief Name of the primary docking space used by this layer.
     */
    std::string _dock_space_name = "DockSpace";

    /**
     * @brief Name of the window associated with the docking space.
     */
    std::string _dock_space_window_name = "DockSpaceDemo";

  private:
    /**
     * @brief Updates the docking layout for all associated layers.
     */
    void update_layout();

    /**
     * @brief List of layers included in the docking layout.
     */
    std::vector<VideoManagerDockingLayer*> _layers_to_dock;
};

}  // namespace tod_gl
