/**
 * @file mouse_events.hpp
 * @brief Mouse event abstraction for the application such as press,release and move of the mouse
 * @copyright 2024 TUMFTM
 */

#pragma once

#include <sstream>
#include <string>

#include "tod_gl/events/event.hpp"
#include "tod_gl/events/mouse_codes.hpp"

namespace tod_gl {

class MouseScrolledEvent : public Event {
  public:
    MouseScrolledEvent(float xOffset, float yOffset) : _xOffset(xOffset), _yOffset(yOffset) {}

    float get_x_offset() const { return _xOffset; }
    float get_y_offset() const { return _yOffset; }

    std::string to_string() const override {
        std::stringstream ss;
        ss << "MouseScrolledEvent: " << get_x_offset() << ", " << get_y_offset();
        return ss.str();
    }

    static EventType get_static_type() { return EventType::MouseScrolled; }
    EventType get_event_type() const override { return get_static_type(); }
    const char *get_name() const override { return "MouseScrolled"; }
    int get_category_flags() const override { return (EventCategoryMouse | EventCategoryInput); }

  private:
    float _xOffset, _yOffset;
};

class MouseMovedEvent : public Event {
  public:
    MouseMovedEvent(float x_pos_in_pixel, float y_pos_in_pixel, unsigned int window_height)
        : x_pos_in_pixel(x_pos_in_pixel), y_pos_in_pixel(y_pos_in_pixel), window_height(window_height) {}
    float x_pos_in_pixel;
    float y_pos_in_pixel;
    unsigned int window_height;

    float get_x_position() const { return x_pos_in_pixel; }
    float get_y_position() const { return y_pos_in_pixel; }

    static EventType get_static_type() { return EventType::MouseMoved; }
    EventType get_event_type() const override { return get_static_type(); }
    const char *get_name() const override { return "MouseMoved"; }
    int get_category_flags() const override { return (EventCategoryMouse | EventCategoryInput); }
};

class MouseButtonEvent : public Event {
  public:
    inline MouseCode get_mouse_button() const { return _button; }

    int get_category_flags() const override { return (EventCategoryMouse | EventCategoryInput); }

  protected:
    explicit MouseButtonEvent(MouseCode button) : _button(button) {}

    MouseCode _button;
};

class MouseButtonPressedEvent : public MouseButtonEvent {
  private:
    float _mouse_x, _mouse_y;

  public:
    explicit MouseButtonPressedEvent(MouseCode button) : MouseButtonEvent(button) {}

    float get_x_position() const { return _mouse_x; }
    float get_y_position() const { return _mouse_y; }

    std::string to_string() const override {
        std::stringstream ss;
        ss << "MouseButtonPressedEvent: " << _button;
        return ss.str();
    }

    static EventType get_static_type() { return EventType::MouseButtonPressed; }
    EventType get_event_type() const override { return get_static_type(); }
    const char *get_name() const override { return "MouseButtonPressed"; }
};

class MouseButtonReleasedEvent : public MouseButtonEvent {
  public:
    explicit MouseButtonReleasedEvent(MouseCode button) : MouseButtonEvent(button) {}

    std::string to_string() const override {
        std::stringstream ss;
        ss << "MouseButtonReleasedEvent: " << _button;
        return ss.str();
    }

    static EventType get_static_type() { return EventType::MouseButtonReleased; }
    EventType get_event_type() const override { return get_static_type(); }
    const char *get_name() const override { return "MouseButtonReleased"; }
};

}  // namespace tod_gl