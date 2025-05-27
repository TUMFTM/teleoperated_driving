/**
 * @file glfw_window.hpp
 * @brief Implementation of the GLFW window, opengl graphics contesxt and window events
 * @copyright 2024 TUMFTM based on Cherno's Hazel engine
 */

#pragma once

#include "tod_gl/core/window.hpp"
#include "tod_gl/core/graphics_context.hpp"

namespace tod_gl {

class GLFWWindow : public Window {
  public:
    explicit GLFWWindow(const WindowProps &props);
    ~GLFWWindow() override;

    void on_update() override;
    unsigned int get_width() const override { return _data.width; }
    unsigned int get_height() const override { return _data.height; }
    void set_event_callback(const EventCallbackFn &callback) override { _data.EventCallback = callback; }
    void *get_native_window() const override { return _window; }

  private:
    GLFWwindow *_window;
    std::unique_ptr<GraphicsContext> _graphics_context;
    struct WindowData {
        std::string title;
        unsigned int width, height;
        bool VSync;
        EventCallbackFn EventCallback;
    };
    WindowData _data;

    virtual void init(const WindowProps &props);
    virtual void shutdown();
};

}  // namespace tod_gl