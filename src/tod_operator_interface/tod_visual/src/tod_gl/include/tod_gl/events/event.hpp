/**
 * @file event.hpp
 * @brief Base class for the event system of the application specifically the 3D scene. E.g. a mouse event would trigger a event dispatch and subsequent changes e.g. camera movement and ros topic publishing
 * @copyright 2024 TUMFTM
 */

#pragma once

#include <string>

namespace tod_gl {

#define BIT(x) (1 << x)

enum class EventType {
    None = 0,
    WindowClose,
    WindowResize,
    WindowFocus,
    WindowLostFocus,
    WindowMoved,
    KeyPressed,
    KeyReleased,
    KeyTyped,
    MouseButtonPressed,
    MouseButtonReleased,
    MouseMoved,
    MouseScrolled,
    LayerStackChanged
};

enum EventCategory {
    None = 0,
    EventCategoryApplication = BIT(0),
    EventCategoryInput = BIT(1),
    EventCategoryKeyboard = BIT(2),
    EventCategoryMouse = BIT(3),
    EventCategoryMouseButton = BIT(4)
};

class Event {
  public:
    virtual ~Event() = default;

    bool Handled = false;

    virtual EventType get_event_type() const = 0;
    virtual const char *get_name() const = 0;
    virtual int get_category_flags() const = 0;
    virtual std::string to_string() const { return get_name(); }

    bool is_in_category(EventCategory category) { return get_category_flags() & category; }
};

class EventDispatcher {
  public:
    explicit EventDispatcher(Event &event) : _event(event) {}

    template <typename T, typename F>
    void dispatch(const F &func) {
        if (_event.get_event_type() == T::get_static_type()) {
            func(static_cast<T &>(_event));
        }
    }

  private:
    Event &_event;
};

}  // namespace tod_gl