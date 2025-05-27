/**
 * @file window.hpp
 * @brief Base abstraction for GLFW window, and event system
 * @copyright 2024 TUMFTM based on Cherno's Hazel
 */

#pragma once

#include <functional>
#include <memory>
#include <string>

#include "tod_gl/events/event.hpp"

namespace tod_gl {
  
struct WindowProps {
    std::string title;
    uint32_t width;
    uint32_t height;

    WindowProps(const std::string &title = "tod_visual", uint32_t width = 1280, uint32_t height = 720)
        : title(title), width(width), height(height) {}
};

class Window {
  public:
    using EventCallbackFn = std::function<void(Event &)>;
    virtual ~Window() = default;
    virtual void on_update() = 0;
    virtual uint32_t get_width() const = 0;
    virtual uint32_t get_height() const = 0;
    virtual void set_event_callback(const EventCallbackFn &callback) = 0;
    virtual void *get_native_window() const = 0;

    static std::unique_ptr<Window> create(const WindowProps &props = WindowProps());
};

}  // namespace tod_gl