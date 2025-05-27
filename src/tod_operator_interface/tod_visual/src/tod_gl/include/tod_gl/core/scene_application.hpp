/**
 * @file scene_application.hpp
 * @brief SceneApplication Base Class that creates a GLFW Window, the ImGui Layers and holds the ROS interface reference.
    This class constructs a 3D scene as well see @ref tod_visual_application for a usage of of the scene context. 
        The scene renders all visual information (images, pointcloud, environment-model informations) 
 * @copyright 2024 TUMFTM
 */

#pragma once

#include <memory>
#include <string>

#include <glad/glad.h>
#include "GLFW/glfw3.h"

#include "tod_gl/core/scene_layer.hpp"
#include "tod_gl/core/scene_layer_stack.hpp"
#include "tod_gl/core/window.hpp"
#include "tod_gl/events/event.hpp"
#include "tod_gl/layers/docking_scene_layer.hpp"

#include "tod_gl/renderer/renderer_command.hpp"
#include "tod_gl/ros_interface/ros_interface.hpp"
#include "tod_gl/scene/scene.hpp"

namespace tod_gl {

class ImGuiSceneLayer;
class MainLayer;
class SceneLayer;
class DockingLayer;

/*
* @ingroup tod_gl
* @brief Scene application, same as @ref Application but also contains a 3D Scene @ref Scene
*/
class SceneApplication {
  public:
    SceneApplication(int argc, char** argv, const std::string& name = "tod_visual");
    virtual ~SceneApplication() = default;

    /* Called after the constructs. */
    void initialize();
    void on_event(Event& e);
    Window& get_window() { return *_window; }
    void run();
    void close();

    void push_layer(SceneLayer* layer);
    void push_overlay(SceneLayer* layer);
    void pop_layer(SceneLayer* layer);
    void pop_overlay(SceneLayer* layer);

    static SceneApplication& get() { return *_app_instance; }

  protected:
    std::shared_ptr<RosInterface> _ros;
    std::shared_ptr<Scene> _active_scene;

  private:
    std::unique_ptr<Window> _window;
    bool _is_running = true;
    float _last_frame_time = 0.0f;
    float _delta_time = 0.0f;
    static SceneApplication* _app_instance;
    ImGuiSceneLayer* _imGui_layer;
    SceneLayerStack _layer_stack;

};
}  // namespace tod_gl