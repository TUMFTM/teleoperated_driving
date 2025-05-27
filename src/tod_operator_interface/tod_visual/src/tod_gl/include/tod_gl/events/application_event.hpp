/**
 * @file application_event.hpp
 * @brief Event abstraction for the application window
 * @copyright 2024 TUMFTM
 */

#pragma once

#include <sstream>
#include <string>

#include "tod_gl/events/event.hpp"

namespace tod_gl {

class WindowCloseEvent : public Event {
  public:
    WindowCloseEvent() = default;
    static EventType get_static_type() { return EventType::WindowClose; }
    EventType get_event_type() const override { return get_static_type(); }
    const char *get_name() const override { return "WindowClose"; }
    int get_category_flags() const override { return (EventCategoryApplication); }

    std::string to_string() const override {
        std::stringstream ss;
        ss << "WindowCloseEvent";
        return ss.str();
    }
};

class WindowResizeEvent : public Event {
  public:
    WindowResizeEvent(unsigned int width, unsigned int height) : _width(width), _height(height) {}
    inline unsigned int get_width() const { return _width; }
    inline unsigned int get_height() const { return _height; }
    std::string to_string() const override {
        std::stringstream ss;
        ss << "WindowResizeEvent: " << _width << ", " << _height;
        return ss.str();
    }

    static EventType get_static_type() { return EventType::WindowResize; }
    EventType get_event_type() const override { return get_static_type(); }
    const char *get_name() const override { return "WindowResize"; }
    int get_category_flags() const override { return (EventCategoryApplication); }

  private:
    unsigned int _width, _height;
};

}  // namespace tod_gl