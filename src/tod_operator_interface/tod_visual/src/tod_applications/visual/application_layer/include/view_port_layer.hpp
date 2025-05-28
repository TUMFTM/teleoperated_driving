/**
 * @file view_port_layer.hpp
 * @brief Manages the viewport for rendering visual elements, handling layouts, and displaying scene content.
 *        Supports multiple layout modes and manages framebuffer rendering.
 * @copyright 2024 TUMFTM
**/
#pragma once

#include <set>
#include <string>
#include <memory>

#include "tod_gl/layers/docking_scene_layer.hpp"

#include "tod_gl/renderer/data_container.hpp"
#include "tod_gl/scene/entity.hpp"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include "imgui/imgui_internal.h"

namespace tod_visual {

  /**
 * @enum Layout
 * @brief Defines different viewport layout modes.
 */
enum class Layout { Default, NoVideo, Custom };

/**
 * @class ViewPortLayer
 * @brief Manages the viewport rendering, layout, and framebuffer handling.
 */
class ViewPortLayer : public tod_gl::DockingSceneLayer {
  public:
     /**
     * @brief Constructs a ViewPortLayer instance.
     * @param ros Shared pointer to the ROS interface.
     * @param scene Shared pointer to the scene.
     * @param split_dir ImGui docking direction.
     */
    ViewPortLayer(std::shared_ptr<tod_gl::RosInterface> ros, std::shared_ptr<tod_gl::Scene> scene, ImGuiDir split_dir);
    ~ViewPortLayer() = default;

    virtual void on_im_gui_render() override;
    virtual void on_attach() override;
    virtual void on_event(tod_gl::Event &e) override;

    ImVec2 getSize();
    ImVec2 getPos();
    ImVec2 getAvail();

  private:
    void SetFrameBuffer();

    int counter = 0;
    tod_gl::Entity ImageEntity;

    Layout layout = Layout::Default;

    ImVec2 _window_position;
    ImVec2 _window_size;
    ImVec2 _avail;
    tod_gl::Texture _texture;
    unsigned int _shader;

    const std::string viewPortWindow = "Viewport";
    std::set<std::string> windowsToDraw;
};

}  // namespace tod_visual