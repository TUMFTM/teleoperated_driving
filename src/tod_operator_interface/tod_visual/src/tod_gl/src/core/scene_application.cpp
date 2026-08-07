/**
 * @file scene_application.cpp
 * @brief SceneApplication base blass that creates a GLFW Window, the ImGui layers and holds the ROS 2 interface reference.
    This class constructs a 3D scene as well see @ref tod_visual_application for a usage of the scene context. 
    The scene renders all visual information (images, pointcloud, environment-model informations).
 * @copyright 2024 TUMFTM
 */

#include "tod_gl/core/scene_application.hpp"

#include <functional>
#include <mutex>
#include <iostream>
#include <csignal> 

#include "tod_gl/core/state_manager.hpp"
#include "tod_gl/events/scene_layer_event.hpp"
#include "tod_gl/scene/components.hpp"

#include "tod_status_msgs/msg/status.hpp"

namespace tod_gl {

std::mutex crash_mutex;

SceneApplication *SceneApplication::_app_instance = nullptr;

SceneApplication::SceneApplication(int argc, char **argv, const std::string &name) {
    _window = Window::create(WindowProps(name));
    _window->set_event_callback([this](auto && PH1) { on_event(std::forward<decltype(PH1)>(PH1)); });
    _app_instance = this;
    RenderCommand::init();

    _ros = std::make_shared<RosInterface>(argc, argv);
    _active_scene = std::make_shared<Scene>();
    _ros->init();

    // std::this_thread::sleep_for (std::chrono::seconds(10));
    _imGui_layer = new tod_gl::ImGuiSceneLayer(_ros, _active_scene);
    push_overlay(_imGui_layer);
}

void SceneApplication::run() {
    RenderCommand::set_clear_color({0.1f, 0.1f, 0.1f, 1});

    int count = 0;

    rclcpp::Rate r(144);
    while (rclcpp::ok() && _is_running) {
        float time_now = (float)glfwGetTime();
        _delta_time = time_now - _last_frame_time;
        _last_frame_time = time_now;

        auto &_stateManager = StateManager::get_instance();
        auto priorMode = _stateManager.get_current_mode();
        try {
            for (SceneLayer *layer : _layer_stack) {
                if (_stateManager.contains_layer(layer->get_name())) {
                    if (!_stateManager.should_render_layer(layer->get_name())) {
                        continue;
                    }
                }
                layer->on_update(_delta_time);
            }
        } catch (const std::exception &e) {
            std::cerr << "Exception in on_update " << e.what() << std::endl;
        }
        try {
            _imGui_layer->begin();
            for (SceneLayer *layer : _layer_stack) {
                // Niklas: Maybe it makes more sense to create a "hot stack" of layers to render and then only iterate
                // those instead of performing those chekcs but unsure
                if (_stateManager.contains_layer(layer->get_name())) {
                    if (!_stateManager.should_render_layer(layer->get_name())) {
                        continue;
                    }
                }
                layer->on_im_gui_render();
            }
            _imGui_layer->end();
        } catch (const std::exception &e) {
            std::cerr << "Exception in layers " << e.what() << std::endl;
        }

        try {
            _window->on_update();
        } catch (const std::exception &e) {
            std::cerr << "Exception in window on_update " << e.what() << std::endl;
        }

        // upon mode change, check if videos should be rendered and update next frame
        auto _current_mode = _stateManager.get_current_mode();
       

        if (priorMode != _current_mode) {
            auto showImguiVideos = true;
            for (SceneLayer *layer : _layer_stack) {
                if (auto *dockingSceneLayer = dynamic_cast<DockingSceneLayer *>(layer)) {
                    dockingSceneLayer->show_videos = showImguiVideos;
                    dockingSceneLayer->should_update_layout = true;
                }
            }
        }

        r.sleep();
    }
}

void SceneApplication::close() {
    _is_running = false;
}

void SceneApplication::push_layer(SceneLayer *layer) {
    _layer_stack.push_layer(layer);
    SceneLayerStackChangedEvent event(layer, false);
    on_event(event);
}

void SceneApplication::push_overlay(SceneLayer *layer) {
    _layer_stack.push_overlay(layer);
    SceneLayerStackChangedEvent event(layer, false);
    on_event(event);
}

void SceneApplication::pop_layer(SceneLayer *layer) {
    SceneLayerStackChangedEvent event(layer, false);
    _layer_stack.pop_layer(layer);
    on_event(event);
}

void SceneApplication::pop_overlay(SceneLayer *layer) {
    SceneLayerStackChangedEvent event(layer, false);
    _layer_stack.pop_overlay(layer);
    on_event(event);
}

void SceneApplication::initialize() {
    _active_scene->init(_window->get_width(), _window->get_height());
}

void SceneApplication::on_event(Event &e) {
    for (auto it = _layer_stack.rbegin(); it != _layer_stack.rend(); ++it) {  
        if (e.Handled)
            break;
        (*it)->on_event(e);
    }
}

}  // namespace tod_gl
