/**
 * @file operator_manager_docking_layer.hpp
 * @brief Defines the base class for managing dynamic layouts with docking functionality in the application.
 * 
 * This file contains the declaration of the OperatorManagerDockingLayer class, which acts as a base class 
 * for layers requiring dynamic layout and docking functionality in the UI. Any derived class can utilize 
 * the docking features by inheriting from this class.
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
 * @brief Namespace for all core components of the TUM teleoperation project.
 */
namespace tod_gl {

/**
 * @class OperatorManagerDockingLayer
 * @brief Base class for managing dynamic UI layout and docking functionality.
 * 
 * The OperatorManagerDockingLayer class provides a foundation for UI layers that require 
 * docking functionality. Layers inheriting from this class can define their docking 
 * behavior by setting the docking direction and dynamically updating the layout.
 * 
 * Derived classes should override specific methods to implement custom UI elements and functionality.
 */
class OperatorManagerDockingLayer : public ImGuiLayer {
  public:
    /**
     * @brief Constructor to initialize the OperatorManagerDockingLayer.
     * 
     * @param _ros Shared pointer to the RosInterface, used for communication within the ROS environment.
     * @param split_dir The docking direction for this layer. 
     *                  Possible values: ImGuiDir_Left, ImGuiDir_Right, ImGuiDir_Up, ImGuiDir_Down.
     */
    OperatorManagerDockingLayer(std::shared_ptr<RosInterface> _ros, ImGuiDir split_dir);

    /**
     * @brief Destructor for the OperatorManagerDockingLayer.
     * 
     * The destructor ensures that any resources allocated by the layer are released.
     */
    ~OperatorManagerDockingLayer() = default;

    /**
     * @brief Called when the layer is attached to the application.
     * 
     * This method is used to initialize any resources or state when the layer becomes active.
     */
    virtual void on_attach() override;

    /**
     * @brief Called when the layer is detached from the application.
     * 
     * This method is used to clean up resources or reset states when the layer is no longer active.
     */
    virtual void on_detach() override;

    /**
     * @brief Renders the ImGui elements for this layer.
     * 
     * Derived classes should override this method to define their specific UI elements.
     */
    virtual void on_im_gui_render() override;

    /**
     * @brief Handles events for the layer.
     * 
     * @param e Reference to the Event object representing the event to handle.
     * 
     * Derived classes can override this method to handle specific events, such as mouse or keyboard inputs.
     */
    virtual void on_event(Event& e) override;

    /**
     * @brief Indicates whether this layer is a video layer.
     * 
     * By default, this method returns false. Derived classes should override it if they represent video layers.
     * 
     * @return True if this layer is a video layer, otherwise false.
     */
    virtual bool is_video_layer() const { return false; }

    /**
     * @brief Whether the layout of the layer should be updated.
     * 
     * When set to true, the layer's layout will be updated dynamically during rendering.
     */
    bool should_update_layout = true;

    /**
     * @brief Flag indicating whether video elements should be displayed.
     */
    bool show_videos = true;

    /**
     * @brief The direction in which this layer should be docked.
     * 
     * This determines where the layer will appear in the docking layout.
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
     * 
     * This method dynamically arranges the layers in the defined docking layout.
     */
    void update_layout();

    /**
     * @brief List of layers to be included in the docking layout.
     */
    std::vector<OperatorManagerDockingLayer*> _layers_to_dock;
};

}  // namespace tod_gl
