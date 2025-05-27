/**
 * @file key_event.hpp
 * @brief Abstraction for keyboard events within the application such as press and release
 * @copyright 2024 TUMFTM
 */

#pragma once

#include <string>
#include <sstream>

#include "tod_gl/events/event.hpp"
#include "tod_gl/events/key_codes.hpp"

namespace tod_gl {

class KeyEvent : public Event {
  public:
    KeyCode get_key_code() const { return _key_code; }
    int get_category_flags() const override { return (EventCategoryKeyboard | EventCategoryInput); }

  protected:
    explicit KeyEvent(KeyCode keycode) : _key_code(keycode) {}

    KeyCode _key_code;
};

class KeyPressedEvent : public KeyEvent {
  public:
    explicit KeyPressedEvent(KeyCode keycode, int repeatCount) : KeyEvent(keycode), _repeatCount(repeatCount) {}

    int get_repeat_count() const { return _repeatCount; }
    std::string to_string() const override {
        std::stringstream ss;
        ss << "KeyPressedEvent: " << _key_code << " (" << _repeatCount << " repeats)";
        return ss.str();
    }

    static EventType get_static_type() { return EventType::KeyPressed; }
    EventType get_event_type() const override { return get_static_type(); }
    const char *get_name() const override { return "KeyPressed"; }

  private:
    int _repeatCount;
};

class KeyReleasedEvent : public KeyEvent {
  public:
    explicit KeyReleasedEvent(KeyCode keycode) : KeyEvent(keycode) {}

    std::string to_string() const override {
        std::stringstream ss;
        ss << "KeyReleasedEvent: " << _key_code;
        return ss.str();
    }
    static EventType get_static_type() { return EventType::KeyReleased; }
    EventType get_event_type() const override { return get_static_type(); }
    const char *get_name() const override { return "KeyReleased"; }
};

}  // namespace tod_gl