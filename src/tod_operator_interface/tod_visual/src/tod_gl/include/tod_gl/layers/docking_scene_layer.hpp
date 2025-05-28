/**
 * @file docking_scene_layer.hpp
 * @brief Base layer for manageing the dockerspace for the scene application's docking layer such as dock events and direction 
 * @copyright 2024 TUMFTM
 */

#pragma once

#include <memory>
#include <vector>
#include <string>

#include "tod_gl/layers/imgui_scene_layer.hpp"

#include "imgui/imgui.h"

/**
 * The layer responsible of managing the dynamic layout changes
 * Any layer that wants to use the docking feature must derive from this.
 */
namespace tod_gl {

class DockingSceneLayer : public ImGuiSceneLayer {
  public:
    DockingSceneLayer(std::shared_ptr<RosInterface> ros, std::shared_ptr<Scene> scene, ImGuiDir split_dir);
    ~DockingSceneLayer() = default;

    virtual void on_attach() override;
    virtual void on_detach() override;
    virtual void on_im_gui_render() override;
    virtual void on_event(Event& e) override;

    /**
     * Which part of the dock this layer should take
     * There can be at most 3 layers on each direction.
     */
    ImGuiDir split_direction = ImGuiDir_None;
    bool show_videos = true;
    virtual bool is_video_layer() const { return false; }
    bool should_update_layout = true;

  protected:
    std::string _dock_space_name = "DockSpace";
    std::string _dock_space_window_name = "DockSpaceDemo";

  private:
    void update_layout();

    std::vector<DockingSceneLayer*> _layers_to_dock;
};

}  // namespace tod_gl