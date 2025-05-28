/**
 * @file application.hpp
 * @brief Creates Application Window using GLFW and with a render and ros thread
 * @copyright 2024 TUMFTM
 */

#pragma once

#include <string>
#include <memory>

#include "tod_gl/events/event.hpp"
#include "tod_gl/core/layer.hpp"
#include "tod_gl/core/layer_stack.hpp"
#include "tod_gl/core/window.hpp"
#include "tod_gl/layers/imgui_layer.hpp"
#include "tod_gl/ros_interface/ros_interface.hpp"

namespace tod_gl {
/*
* @ingroup tod_gl
*/
class Application {
  public:
    Application(int argc, char** argv, const std::string& name = "tod_application");
    virtual ~Application() = default;

    /* Called after the constructs. */
    void initialize();
    void on_event(Event& e);
    Window& get_window() { return *_window; }
    void run();
    void close();

    void push_layer(Layer* layer);
    void push_overlay(Layer* layer);
    void pop_layer(Layer* layer);
    void pop_overlay(Layer* layer);

    static Application& get() { return *_app_instance; }

  protected:
    std::shared_ptr<RosInterface> _ros;

  private:
    std::unique_ptr<Window> _window;
    bool _is_running = true;
    float _last_frame_time = 0.0f;
    float _delta_time = 0.0f;
    static Application* _app_instance;
    ImGuiLayer* _imGui_layer;
    LayerStack _layer_stack;

    // TODO: Need this?
    void setDebugMode();
};

}  // namespace tod_gl