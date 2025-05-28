/**
 * @file application.cpp
 * @brief Application base class that creates an GLFW Window, the ImGui Layers and holds the ROS 2 interface reference.
 * @copyright 2024 TUMFTM based on Cherno's Hazel engine
 */

#include "tod_gl/core/application.hpp"

#include "tod_gl/renderer/renderer_command.hpp"
#include "tod_gl/events/layer_event.hpp"

#include <glad/glad.h>
#include "GLFW/glfw3.h"
#include "rclcpp/rclcpp.hpp"

namespace tod_gl {
/**
 * @ingroup tod_visual
 * @defgroup tod_gl ToD Graphics Library

 * @brief ToD Library to create both application window and a 3D scene using an entity component system (ENTT) based on Cherno's Hazel Engine. 
    It is integrated with ROS 2. We use dearImGui for our Gui elements.
 * @link https://github.com/GloriousPtr/ArcGameEngine/tree/main/Arc/src/Arc @endlink
 * @link https://github.com/TheCherno/Hazel/tree/master/Hazel/src/Hazel @endlink
 * @link https://github.com/skypjack/entt @endlink
 * @link https://github.com/ocornut/imgui @endlink
*/

Application *Application::_app_instance = nullptr;

/*
* @ingroup tod_visual
* @defgroup tod_gl_core Core functions like windows, layers and scnee management of the TOD Graphics Library 
* @brief Constructor of Application, Initialized window, renderer context, imgui context and ros thread
*/
Application::Application(int argc, char **argv, const std::string &name) {
    _window = Window::create(WindowProps(name));
    _window->set_event_callback([this](auto && PH1) { on_event(std::forward<decltype(PH1)>(PH1)); });
    _app_instance = this;
    RenderCommand::init();

    _ros = std::make_shared<RosInterface>(argc, argv);
    _ros->init();
    _imGui_layer = new ImGuiLayer(_ros);
    push_overlay(_imGui_layer);
}

/*
* @brief render loop of the application, performs update of data based on new ros data, imgui layer render loop and window update loop 
*/
void Application::run() {
    RenderCommand::set_clear_color({0.1f, 0.1f, 0.1f, 1});

    rclcpp::Rate r(60);
    while (rclcpp::ok() && _is_running) {
        float time_now = (float)glfwGetTime();
        _delta_time = time_now - _last_frame_time;
        _last_frame_time = time_now;

        for (Layer *layer : _layer_stack) {
            layer->on_update(_delta_time);
        }

        _imGui_layer->begin();
        for (Layer *layer : _layer_stack) {
            layer->on_im_gui_render();
        }
        _imGui_layer->end();
        _window->on_update();
        // upon mode change, check if videos should be rendered and update next frame
        r.sleep();
    }
}

void Application::close() {
    _is_running = false;
}

void Application::push_layer(Layer *layer) {
    _layer_stack.push_layer(layer);
    LayerStackChangedEvent event(layer, false);
    on_event(event);
}

void Application::push_overlay(Layer *layer) {
    _layer_stack.push_overlay(layer);
    LayerStackChangedEvent event(layer, false);
    on_event(event);
}

void Application::pop_layer(Layer *layer) {
    LayerStackChangedEvent event(layer, false);
    _layer_stack.pop_layer(layer);
    on_event(event);
}

void Application::pop_overlay(Layer *layer) {
    LayerStackChangedEvent event(layer, false);
    _layer_stack.pop_overlay(layer);
    on_event(event);
}

void Application::initialize() {}

void Application::on_event(Event &e) {
    for (auto it = _layer_stack.rbegin(); it != _layer_stack.rend(); ++it) {
        if (e.Handled)
            break;
        (*it)->on_event(e);
    }
}

}  // namespace tod_gl