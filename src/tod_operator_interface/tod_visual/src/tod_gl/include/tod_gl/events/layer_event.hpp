/**
 * @file layer_event.hpp
 * @brief Event abstraction for the dearImGui Layer events such as docking layer moved, removal from the layer stack 
 * @copyright 2024 TUMFTM
 */

#pragma once

#include "tod_gl/core/layer.hpp"
#include "tod_gl/events/event.hpp"

namespace tod_gl {

class LayerStackChangedEvent : public Event {
  public:
    LayerStackChangedEvent(Layer* layer, bool deleted) : _layer(layer), _deleted(deleted){};

    static EventType get_static_type() { return EventType::LayerStackChanged; }
    EventType get_event_type() const override { return get_static_type(); }
    const char* get_name() const override { return "LayerStackChanged"; }
    int get_category_flags() const override { return (EventCategoryApplication); }

    inline Layer* get_layer() const { return _layer; }
    inline bool is_deleted() const { return _deleted; }

  private:
    Layer* _layer;
    bool _deleted;
};

}  // namespace tod_gl